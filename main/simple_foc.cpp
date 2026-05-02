/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "interface.h"
// static const char* TAG = "ffb_foc";
//******************************** SimpleFOC Configuration //********************************
#define BLDC_MOTOR_PP (7)
#define VOLTAGE_POWER (9.0f)
#define VOLTAGE_LIMIT (6.0f)
#define VOLTAGE_SENSOR_ALIGN (3.0f)
#define DAMPING_MAX_VELOCITY (6.0f * PI)
#define MOTOR_A (CONFIG_MOTOR_A)
#define MOTOR_B (CONFIG_MOTOR_B)
#define MOTOR_C (CONFIG_MOTOR_C)
#define MOTOR_EN (CONFIG_MOTOR_EN)
#define WIRE_SDA ((gpio_num_t)CONFIG_IIC_SDA)
#define WIRE_SCL ((gpio_num_t)CONFIG_IIC_SCL)
#define SPI_CSO ((gpio_num_t)CONFIG_SPI_CSO)
#define SPI_CLK ((gpio_num_t)CONFIG_SPI_CLK)
#define SPI_Q ((gpio_num_t)CONFIG_SPI_Q)
#define SPIX_HOST ((spi_host_device_t)CONFIG_SPI_HOST_NUM)
#define FOC_MONITOR_BAUD CONFIG_MONITOR_BAUD
#define SENSOR_MT6701 1
#define SENSOR_AS5600 0
#define SENSOR_STEP_NUM (2.0f)
//******************************** SimpleFOC Input //********************************
TaskHandle_t foc_task_handle;
TaskHandle_t foc_loop_handle;
static float g_constant_force;
static float g_damper;
//******************************** SimpleFOC Output //********************************
static float g_wheel_rad;
void foc_backend_output(float* wheel_rad) { *wheel_rad = g_wheel_rad; }
//******************************** SimpleFOC Function //********************************
#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif
// magnetic sensor
#if SENSOR_MT6701
static MT6701 sensor = MT6701(SPIX_HOST, SPI_CLK, SPI_Q, (gpio_num_t)-1, SPI_CSO);
#define SENSOR_DIRECTION (-1.0f)
#define SENSOR_STEP_MIN (0.0003835f)
#define FOC_LOOP_PERIOD (100)
#elif SENSOR_AS5600
static AS5600 sensor = AS5600(I2C_NUM_0, WIRE_SCL, WIRE_SDA);
#define SENSOR_DIRECTION (1.0f)
#define SENSOR_STEP_MIN (0.001534f)
#define FOC_LOOP_PERIOD (1000)
#endif
// BLDC motor & driver instance
static BLDCMotor motor = BLDCMotor(BLDC_MOTOR_PP);
static BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_EN);
static void get_angle_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        float wheel_rad = sensor.getAngle() * SENSOR_DIRECTION;
        if (fabsf(wheel_rad - g_wheel_rad) < SENSOR_STEP_MIN * SENSOR_STEP_NUM) {
            continue;
        }
        g_wheel_rad = wheel_rad;
        xTaskNotify(*ffb_task_handle, 0, eSetBits);
        // ESP_LOGI(TAG, "A%f", g_wheel_rad);
    }
}
static void foc_init_task(void* arg) {
    // initialise magnetic sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);
    // power supply voltage
    driver.voltage_power_supply = VOLTAGE_POWER;
    driver.voltage_limit = VOLTAGE_LIMIT;
#ifdef USING_MCPWM
    driver.init(0);
#else
    driver.init({1, 2, 3});
#endif
    motor.linkDriver(&driver);
    // aligning voltage
    motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // set motion control loop to be used
    motor.controller = MotionControlType::torque;
    // set torque control loop to be used
    motor.torque_controller = TorqueControlType::voltage;
    // use monitoring with serial
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.begin(FOC_MONITOR_BAUD);
    // comment out if not needed
    vTaskDelay(pdMS_TO_TICKS(100));
    SimpleFOCDebug::enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    motor.useMonitoring(Serial);
    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();
    // div of motor.loopFOC() and motor.move()
    static int div = 0;
    for (;;) {
        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop  ~1kHz
        // Bluepill loop ~10kHz
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        motor.loopFOC();
        if (++div < 2) {
            continue;
        }
        div = 0;
        float damper = g_damper;
        float constant_force = g_constant_force;
        float damping = damper * motor.shaft_velocity / DAMPING_MAX_VELOCITY;
        float torque_ratio = constant_force - damping;
        torque_ratio = torque_ratio > 1.0f ? 1.0f : (torque_ratio < -1.0f ? -1.0f : torque_ratio);
        // voltage set point variable
        float target_voltage = VOLTAGE_LIMIT * torque_ratio;
        // Motion control function
        // current_velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
        motor.move(target_voltage);
    }
}
void foc_input_task(void* arg) {
    for (;;) {
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);
        ffb_output(&g_constant_force, &g_damper);
    }
}
static bool example_timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(foc_loop_handle, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}
void foc_backend_init(void) {
    xTaskCreatePinnedToCore(foc_init_task, "foc_init_task", TASK_STACK_SIZE, NULL, 12, &foc_loop_handle, CORE_1);
    xTaskCreatePinnedToCore(foc_input_task, "foc_input_task", TASK_STACK_SIZE, NULL, 11, &foc_task_handle, CORE_1);
    xTaskCreatePinnedToCore(get_angle_task, "get_angle_task", TASK_STACK_SIZE, NULL, 10, NULL, CORE_1);
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_alarm_config_t alarm_config = {.alarm_count = FOC_LOOP_PERIOD,
                                           .reload_count = 0,
                                           .flags = {
                                               .auto_reload_on_alarm = true,
                                           }};
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = example_timer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    vTaskDelay(pdMS_TO_TICKS(500));
}
