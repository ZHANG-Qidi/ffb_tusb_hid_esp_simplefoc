
/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hidReportDesc.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"

static const char* TAG = "ffb_tusb_hid_esp_simplefoc";

//******************************** Global Define //********************************

#define TASK_STACK_SIZE (4096)

//******************************** tinyUSB Configuration //********************************

/* Flag to indicate if the host has suspended the USB bus */
static bool suspended = false;

/* Flag of possibility to Wakeup Host via Remote Wakeup feature */
static bool wakeup_host = false;
/************* TinyUSB descriptors ****************/
#define TUSB_DESC_IN_OUT_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_INOUT_DESC_LEN)

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},     // 0: is supported language is English (0x0409)
    "TinyUSB",                // 1: Manufacturer
    "TinyUSB Device",         // 2: Product
    "123456",                 // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_IN_OUT_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    // HID Input & Output descriptor
    // Interface number, string index, protocol, report descriptor len, EP OUT & IN address, size & polling interval
    TUD_HID_INOUT_DESCRIPTOR(0, 4, false, sizeof(G_DefaultReportDescriptor), 0x81, 0x01, 64, 1),
};

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return G_DefaultReportDescriptor;
}

//******************************** SimpleFOC Configuration //********************************

#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif

#define MOTOR_A (9)
#define MOTOR_B (10)
#define MOTOR_C (11)

#define WIRE_SDA (GPIO_NUM_13)
#define WIRE_SCL (GPIO_NUM_14)

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C);
AS5600 as5600 = AS5600(I2C_NUM_0, WIRE_SCL, WIRE_SDA);

volatile float motor_target_voltage = 0.0f;
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&motor_target_voltage, cmd); }

LowPassFilter angle_filter(0.02);

#define DAMPING 0.05f

//******************************** USB JOYSTICK INPUT REPORT //********************************

typedef struct __attribute__((packed)) {
    uint32_t axis_x;
    uint32_t axis_y;
    uint32_t axis_z;
    uint32_t axis_rx;
    uint32_t axis_ry;
    uint32_t axis_rz;
    uint32_t slider;
    uint32_t dial;
    uint32_t pov_reserved[4];
    union {
        uint32_t buttons_raw;
        struct {
            uint8_t btn1 : 1;
            uint8_t btn2 : 1;
            uint8_t btn3 : 1;
            uint8_t btn4 : 1;
            uint8_t btn5 : 1;
            uint8_t btn6 : 1;
            uint8_t btn7 : 1;
            uint8_t btn8 : 1;
            uint32_t btn_padding : 24;
        };
    };
    struct {
        uint8_t pov : 4;
        uint8_t pov_padding : 4;
    };
} hid_joystick_input_t;

//******************************** USB OUTPUT REPORT //********************************

// Usage PID Device Control
typedef enum { DC_ENABLE_ACTUATORS = 1, DC_DISABLE_ACTUATORS, DC_STOP_ALL_EFFECTS, DC_DEVICE_RESET, DC_DEVICE_PAUSE, DC_DEVICE_CONTINUE } PID_Device_Control;

// Usage Effect Type
typedef enum { ET_CONSTANT = 1, ET_RAMP, ET_SQUARE, ET_SINE, ET_TRIANGLE, ET_SAWTOOTH_UP, ET_SAWTOOTH_DOWN, ET_SPRING, ET_DAMPER, ET_INERTIA, ET_FRICTION, ET_CUSTOM } Effect_Type;

// Usage Block Status
typedef enum { BLOCK_FREE, BLOCK_ACCLOCATED } BLOCK_STATUS;

// Usage Block Load Status
typedef enum { BLOCK_LOAD_SUCCESS = 1, BLOCK_LOAD_FULL, BLOCK_LOAD_ERROR } BLOCK_LOAD_STATUS;

// Usage Effect Operation
typedef enum {
    EFFECT_START = 1,
    EFFECT_START_SOLO,
    EFFECT_STOP,
} EFFECT_OPERATION;

// Usage Set Effect Report
typedef struct __attribute__((packed)) {
    uint8_t index;                     // Usage Effect Block Index
    uint8_t type;                      // Usage Effect Type
    uint16_t duration;                 // Usage Duration
    uint16_t trigger_repeat_interval;  // Trigger Repeat Interval
    uint16_t sample_period;            // Usage Sample Period
    uint8_t gain;                      // Usage Gain
    uint8_t trigger_button;            // Usage Trigger Button
    union {
        uint8_t axes_direction;
        struct {
            uint8_t axis_enable_x : 1;     // Usage Axes Enable X
            uint8_t axis_enable_y : 1;     // Usage Axes Enable Y
            uint8_t direction_enable : 1;  // Usage Direction Enable
            uint8_t reserved : 5;          // Usage Reserved
        };
    };
    uint8_t direction[2];              // Usage Direction
    uint16_t type_specific_offset[2];  // USAGE (Type Specific Block Offset)
} effect_report_t;

// Usage Effect Operation Report
typedef struct __attribute__((packed)) {
    uint8_t index;       // Usage Effect Block Index
    uint8_t operation;   // Usage Effect Operation
    uint8_t loop_count;  // Usage Loop Count
} operation_report_t;

//******************************** FFB EFFECT POOL //********************************

typedef struct {
    uint8_t allocated;
    union {
        uint8_t effect_report_raw[17];
        effect_report_t effect_report;
    };
    union {
        uint8_t operation_report_raw[3];
        operation_report_t operation_report;
    };
    union {
        struct {
            int16_t magnitude;    // Usage Magnitude
        } constant_force_report;  // Usage Set Constant Force Report
    };
} ffb_effect_t;

#define FFB_EFFECT_COUNT 16
static ffb_effect_t g_effect_pool[FFB_EFFECT_COUNT];
static uint8_t g_effect_type;
static uint8_t g_effect_block_index;
static uint8_t g_effect_block_status;
static uint8_t g_effect_gain;

//******************************** FFB FUNCTION //********************************

#define FFB_VOLTAGE_MAX (4.0f)

void ffb_effect_mixer(void) {
    float magnitude_constant_force = 0.0f;
    for (int i = 0; i < FFB_EFFECT_COUNT; i++) {
        if (g_effect_pool[i].allocated == BLOCK_FREE) {
            continue;
        }
        if (g_effect_pool[i].operation_report.operation == EFFECT_STOP) {
            continue;
        }
        // float duration_ms = g_effect_pool[i].effect_report.duration * 0.1f;
        // float trigger_repeat_interval_ms = g_effect_pool[i].effect_report.trigger_repeat_interval * 0.1f;
        // float sample_period_ms = g_effect_pool[i].effect_report.sample_period * 0.1f;
        // float direction_deg = g_effect_pool[i].effect_report.direction[0] / 255.0f * 360.0f;
        float gain_10000 = g_effect_pool[i].effect_report.gain / 255.0f * 10000.0f;
        float g_effect_gain_10000 = g_effect_gain / 255.0f * 10000.0f;
        switch (g_effect_pool[i].effect_report.type) {
            case ET_CONSTANT: {
                int16_t magnitude_i16 = g_effect_pool[i].constant_force_report.magnitude;
                float magnitude_scaled = magnitude_i16 * (gain_10000 / 10000.0f) * (g_effect_gain_10000 / 10000.0f);
                magnitude_constant_force += magnitude_scaled;
                break;
            }
            default: {
                ESP_LOGI("FFB", "Unimplemented Effect: %d", g_effect_pool[i].effect_report.type);
                break;
            }
        }
    }
    // ESP_LOGI("FFB", "magnitude_constant_force: %f", magnitude_constant_force);
    motor_target_voltage = (magnitude_constant_force / 10000.0f * FFB_VOLTAGE_MAX);
}

//******************************** tinyUSB FUNCTION //********************************

union u8_2_16 {
    uint8_t u8[2];
    int16_t i16;
    uint16_t u16;
} u8_2_16;

static void dump_hex(const uint8_t* buf, int len) {
    char line[256];
    int pos = 0;
    for (int i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", buf[i]);
    }
    ESP_LOGI("HID", "%s", line);
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    ESP_LOGI("HID", "GET_REPORT: inst=%u id=%u type=%u len=%u", instance, report_id, report_type, reqlen);
    dump_hex(buffer, reqlen);
    if (report_type == HID_REPORT_TYPE_INPUT) {
    }
    if (report_type == HID_REPORT_TYPE_FEATURE) {
        if (report_id == (HID_ID_POOLREP + 0x10 * TLID)) {
            buffer[0] = 0xff;
            buffer[1] = 0xff;
            buffer[2] = FFB_EFFECT_COUNT;
            buffer[3] = 0x01;
            return 4;
        }
        if (report_id == (HID_ID_BLKLDREP + 0x10 * TLID)) {
            buffer[0] = g_effect_block_index;
            buffer[1] = g_effect_block_status;
            buffer[2] = 0x00;
            buffer[3] = 0x00;
            return 4;
        }
    }
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
    }
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    // ESP_LOGI("HID", "SET_REPORT: inst=%u id=%u type=%u size=%u", instance, report_id, report_type, bufsize);
    // dump_hex(buffer, bufsize);
    if (report_type == HID_REPORT_TYPE_INPUT) {
    }
    if (report_type == HID_REPORT_TYPE_FEATURE) {
        switch (report_id) {
            case (HID_ID_NEWEFREP + 0x10 * TLID): {
                uint8_t type = buffer[0];
                g_effect_type = type;
                g_effect_block_index = 0;
                g_effect_block_status = BLOCK_LOAD_ERROR;
                for (int i = 0; i < FFB_EFFECT_COUNT; i++) {
                    if (!g_effect_pool[i].allocated) {
                        g_effect_pool[i].allocated = BLOCK_ACCLOCATED;
                        g_effect_block_index = i + 1;
                        g_effect_block_status = BLOCK_LOAD_SUCCESS;
                        break;
                    }
                }
                break;
            }
            default: {
                ESP_LOGI("HID", "SET_REPORT: inst=%u id=%u type=%u size=%u", instance, report_id, report_type, bufsize);
                dump_hex(buffer, bufsize);
                break;
            }
        }
    }
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
        switch (buffer[0]) {
            case (HID_ID_CTRLREP + 0x10 * TLID):
                switch (buffer[1]) {
                    case DC_ENABLE_ACTUATORS:
                        break;
                    case DC_DISABLE_ACTUATORS:
                        break;
                    case DC_STOP_ALL_EFFECTS: {
                        for (int i = 0; i < FFB_EFFECT_COUNT; i++) {
                            g_effect_pool[i].operation_report.operation = EFFECT_STOP;
                        }
                    } break;
                    case DC_DEVICE_RESET:
                        break;
                    case DC_DEVICE_PAUSE:
                        break;
                    case DC_DEVICE_CONTINUE:
                        break;
                    default:
                        break;
                }
                break;
            case (HID_ID_GAINREP + 0x10 * TLID): {
                g_effect_gain = buffer[1];
                break;
            }
            case (HID_ID_CONSTREP + 0x10 * TLID): {
                u8_2_16.u8[0] = buffer[2];
                u8_2_16.u8[1] = buffer[3];
                g_effect_pool[buffer[1] - 1].constant_force_report.magnitude = u8_2_16.i16;
                break;
            }
            case (HID_ID_EFOPREP + 0x10 * TLID): {
                memcpy(g_effect_pool[buffer[1] - 1].operation_report_raw, &buffer[1], sizeof(g_effect_pool[buffer[1] - 1].operation_report_raw));
                break;
            }
            case (HID_ID_BLKFRREP + 0x10 * TLID): {
                ffb_effect_t* e = &g_effect_pool[buffer[1] - 1];
                memset(e, 0, sizeof(ffb_effect_t));
                break;
            }
            case (HID_ID_EFFREP + 0x10 * TLID): {
                memcpy(g_effect_pool[buffer[1] - 1].effect_report_raw, &buffer[1], sizeof(g_effect_pool[buffer[1] - 1].effect_report_raw));
                break;
            }
            default: {
                ESP_LOGI("HID", "SET_REPORT: inst=%u id=%u type=%u size=%u", instance, report_id, report_type, bufsize);
                dump_hex(buffer, bufsize);
                break;
            }
        }
        ffb_effect_mixer();
    }
}

void tud_suspend_cb(bool remote_wakeup_en) {
    ESP_LOGI(TAG, "USB device suspended");
    suspended = true;
    if (remote_wakeup_en) {
        ESP_LOGI(TAG, "Remote wakeup available, press the button to wake up the Host");
        wakeup_host = true;
    } else {
        ESP_LOGI(TAG, "Remote wakeup not available");
    }
}

void tud_resume_cb(void) {
    ESP_LOGI(TAG, "USB device resumed");
    suspended = false;
}

static void usb_task(void* arg) {
    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.descriptor.device = NULL;
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif  // TUD_OPT_HIGH_SPEED
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

//******************************** SimpleFOC Function //********************************

static void foc_read_angle_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
        if (!(tud_mounted() && tud_hid_ready())) {
            continue;
        }
        float raw_angle = as5600.getAngle();
        float wheel_rad = angle_filter(raw_angle);
        // ESP_LOGI("UART", "A%f", wheel_rad);
        hid_joystick_input_t joy = {.pov = 8};
        const uint32_t AXIS_MID = 16383;
        const float WHEEL_HALF = (M_PI * 4.0f);
        float wheel_rad_clamped = wheel_rad > WHEEL_HALF ? WHEEL_HALF : (wheel_rad < -WHEEL_HALF ? -WHEEL_HALF : wheel_rad);
        joy.axis_x = AXIS_MID + wheel_rad_clamped / WHEEL_HALF * AXIS_MID;
        tud_hid_report(TLID, &joy, sizeof(hid_joystick_input_t));
    }
}

static void foc_task(void* arg) {
    SimpleFOCDebug::enable(); /*!< Enable debug */
    Serial.begin(115200);

    as5600.init(); /*!< Enable as5600 */
    motor.linkSensor(&as5600);
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 6;
#ifdef USING_MCPWM
    driver.init(0);
#else
    driver.init({1, 2, 3});
#endif

    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    // motor.controller = MotionControlType::velocity; /*!< Set position control mode */
    /*!< Set velocity pid */
    motor.voltage_limit = 6;
    motor.voltage_sensor_align = 5;
    motor.PID_velocity.P = 0.1f;
    motor.PID_velocity.I = 5.0f;
    motor.PID_velocity.D = 0.0f;
    motor.LPF_velocity.Tf = 0.01f;
    motor.velocity_limit = 200;

    motor.useMonitoring(Serial);
    motor.init();    /*!< Initialize motor */
    motor.initFOC(); /*!<  Align sensor and start FOC */
    // command.add('T', doTarget, const_cast<char*>("target voltage")); /*!< Add serial command */

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        motor.loopFOC();

        float vel = motor.shaft_velocity;
        float damping = DAMPING * vel;
        float output = motor_target_voltage - damping;

        motor.move(output);
        // command.run();
    }
}

//******************************** JOYSTICK TEST FUNCTION //********************************

#define APP_BUTTON (GPIO_NUM_0)  // Use BOOT signal by default

static void joystick_test(void) {
    ESP_LOGI(TAG, "Sending Joystick X-axis sweep demo");
    const uint32_t AXIS_MID = 16384;
    const uint32_t AXIS_MAX = 32767;
    const uint32_t STEP = 256;
    const uint8_t DELAY_MS = 20;
    hid_joystick_input_t joy = {.pov = 8};
    joy.axis_x = AXIS_MID;

    ESP_LOGI(TAG, "X-axis moving from mid to max");
    for (uint32_t x = AXIS_MID; x <= AXIS_MAX; x += STEP) {
        joy.axis_x = x;
        tud_hid_report(TLID, &joy, sizeof(hid_joystick_input_t));
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }

    ESP_LOGI(TAG, "X-axis moving from max back to mid");
    for (uint32_t x = AXIS_MAX; x >= AXIS_MID; x -= STEP) {
        joy.axis_x = x;
        tud_hid_report(TLID, &joy, sizeof(hid_joystick_input_t));
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }

    joy.axis_x = AXIS_MID;
    tud_hid_report(TLID, &joy, sizeof(hid_joystick_input_t));
    ESP_LOGI(TAG, "X-axis sweep demo finished, reset to mid");
}

static void joystick_test_task(void* arg) {
    // Initialize button that will trigger HID reports
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        if (tud_mounted()) {
            static bool send_hid_data = false;
            if (send_hid_data) {
                if (!suspended) {
                    joystick_test();
                } else {
                    if (wakeup_host) {
                        ESP_LOGI(TAG, "Waking up the Host");
                        tud_remote_wakeup();
                        wakeup_host = false;
                    } else {
                        ESP_LOGI(TAG, "USB Host remote wakeup is not available.");
                    }
                }
            }
            send_hid_data = !gpio_get_level(APP_BUTTON);
        }
    }
}

//******************************** MAIN FUNCTION //********************************

extern "C" void app_main(void) {
    xTaskCreate(usb_task, "usb_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(foc_task, "foc_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(foc_read_angle_task, "foc_read_angle_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(joystick_test_task, "joystick_test_task", TASK_STACK_SIZE, NULL, 10, NULL);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}
