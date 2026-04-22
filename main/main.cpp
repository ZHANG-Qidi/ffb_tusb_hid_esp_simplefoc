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

static const char* TAG = "ffb_main";

//******************************** FFB Configuration //********************************

#define FFB_VOLTAGE_MAX (1.0f)
#define WHEEL_HALF (M_PI * 4.0f)

#define GAIN_EFFECT_LOGICAL_MAX (255.0f)
#define GAIN_EFFECT_PHYSICAL_MAX (10000.0f)

#define GAIN_DEVICE_LOGICAL_MAX (255.0f)
#define GAIN_DEVICE_PHYSICAL_MAX (10000.0f)

#define CONSTANT_MANITUDE_MAX (10000.0f)

#define JOYSTIC_AXIS_LOGICAL_MID (16383.5f)
#define JOYSTIC_AXIS_LOGICAL_MAX (32767.0f)

#define DAMPING_MAX_VELOCITY (6.0f * PI)
#define DAMPING_GAIN (0.5f)
#define MOTOR_GAIN (0.5f)

#define ET_DAMPER_DR2 (0x0b)
static float damper;
static float constant_force;

//******************************** SimpleFOC Configuration //********************************

#define BLDC_MOTOR_PP (7)

#define VOLTAGE_POWER (9.0f)
#define VOLTAGE_LIMIT (6.0f)
#define VOLTAGE_SENSOR_ALIGN (3.0f)

#define MOTOR_A (9)
#define MOTOR_B (10)
#define MOTOR_C (11)

#define WIRE_SDA (GPIO_NUM_13)
#define WIRE_SCL (GPIO_NUM_14)

#define FOC_MONITOR_BAUD CONFIG_MONITOR_BAUD

#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif

// magnetic sensor instance - I2C
AS5600 as5600 = AS5600(I2C_NUM_0, WIRE_SCL, WIRE_SDA);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(BLDC_MOTOR_PP);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C);

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
    // HID Input & torque_ratio descriptor
    // Interface number, string index, protocol, report descriptor len, EP OUT & IN address, size & polling interval
    TUD_HID_INOUT_DESCRIPTOR(0, 4, false, sizeof(G_DefaultReportDescriptor), 0x81, 0x01, 64, 1),
};

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return G_DefaultReportDescriptor;
}

tusb_desc_device_t const desc_device = {.bLength = sizeof(tusb_desc_device_t),
                                        .bDescriptorType = TUSB_DESC_DEVICE,
                                        .bcdUSB = 0x0200,

                                        .bDeviceClass = 0x00,
                                        .bDeviceSubClass = 0x00,
                                        .bDeviceProtocol = 0x00,

                                        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

                                        .idVendor = 0x045E,   // ms_sidewinder_ffb
                                        .idProduct = 0x0034,  // ms_sidewinder_ffb

                                        .bcdDevice = 0x0100,

                                        .iManufacturer = 0x01,
                                        .iProduct = 0x02,
                                        .iSerialNumber = 0x03,

                                        .bNumConfigurations = 0x01};

//******************************** FreeRTOS Configuration //********************************

#define TASK_STACK_SIZE (4096)

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

//******************************** USB torque_ratio REPORT //********************************

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

// Usage Set Condition Report
typedef struct __attribute__((packed)) {
    uint8_t index;                       // Usage Effect Block Index
    uint8_t parameter_block_offset : 4;  // Usage Parameter Block Offset
    uint8_t ordinals_instance_1 : 2;     // Usage Ordinals: Instance 1
    uint8_t ordinals_instance_2 : 2;     // Usage Ordinals: Instance 2
    int16_t CP_offset;                   // Usage CP Offset
    int16_t positive_coefficient;        // Usage Positive Coefficient
    int16_t negative_coefficient;        // Usage Negative Coefficient
    uint16_t positive_saturation;        // Usage Positive Saturation
    uint16_t negative_saturation;        // Usage Negative Saturation
    uint16_t dead_band;                  // Usage Dead Band
} condition_report_t;
//******************************** FFB EFFECT POOL //********************************

#define EFFECT_REPORT_LEN (17)
#define OPERATION_REPORT_LEN (3)
#define CONDITION_REPORT_LEN (14)

typedef struct {
    uint8_t allocated;
    union {
        uint8_t effect_report_raw[EFFECT_REPORT_LEN];
        effect_report_t effect_report;
    };
    union {
        uint8_t operation_report_raw[OPERATION_REPORT_LEN];
        operation_report_t operation_report;
    };
    union {
        struct {
            int16_t magnitude;    // Usage Magnitude
        } constant_force_report;  // Usage Set Constant Force Report
        union {
            uint8_t condition_report_raw[CONDITION_REPORT_LEN];
            condition_report_t condition_report;  // Usage Set Condition Report
        };
    };
} ffb_effect_t;

#define FFB_EFFECT_COUNT 32

static ffb_effect_t g_effect_pool[FFB_EFFECT_COUNT];
static uint8_t g_effect_type;
static uint8_t g_effect_block_index;
static uint8_t g_effect_block_status;
static uint8_t g_gain_device;

//******************************** FFB Function //********************************

void ffb_effect_mixer(void) {
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
        float gain_effect_scale = g_effect_pool[i].effect_report.gain / GAIN_EFFECT_LOGICAL_MAX;
        float gain_device_scale = g_gain_device / GAIN_DEVICE_LOGICAL_MAX;
        switch (g_effect_pool[i].effect_report.type) {
            case ET_CONSTANT: {
                int16_t magnitude = g_effect_pool[i].constant_force_report.magnitude;
                constant_force = magnitude * gain_effect_scale * gain_device_scale / CONSTANT_MANITUDE_MAX * MOTOR_GAIN;
                break;
            }
            case ET_DAMPER_DR2: {
                damper = (float)g_effect_pool[i].condition_report.positive_coefficient / (float)g_effect_pool[i].condition_report.positive_saturation;
                break;
            }
            default: {
                ESP_LOGI("FFB", "Unimplemented Effect: %d", g_effect_pool[i].effect_report.type);
                break;
            }
        }
    }
}

//******************************** tinyUSB Function //********************************

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
        switch (report_id) {
            case (HID_ID_POOLREP + 0x10 * TLID): {
                buffer[0] = 0xff;
                buffer[1] = 0xff;
                buffer[2] = FFB_EFFECT_COUNT;
                buffer[3] = 0x01;
                return 4;
            }
            case (HID_ID_BLKLDREP + 0x10 * TLID): {
                buffer[0] = g_effect_block_index;
                buffer[1] = g_effect_block_status;
                buffer[2] = 0x00;
                buffer[3] = 0x00;
                return 4;
            }
            default: {
                ESP_LOGI("HID", "Unimplemented GET_REPORT HID_REPORT_TYPE_FEATURE: %d", report_id);
                break;
            }
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
        ESP_LOGI("HID", "SET_REPORT: inst=%u id=%u type=%u size=%u", instance, report_id, report_type, bufsize);
        dump_hex(buffer, bufsize);
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
                ESP_LOGI("HID", "Unimplemented SET_REPORT HID_REPORT_TYPE_FEATURE: %d", report_id);
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
                        ESP_LOGI("HID", "Unimplemented Usage PID Device Control: %d", buffer[1]);
                        break;
                }
                break;
            case (HID_ID_GAINREP + 0x10 * TLID): {
                g_gain_device = buffer[1];
                break;
            }
            case (HID_ID_CONSTREP + 0x10 * TLID): {
                u8_2_16.u8[0] = buffer[2];
                u8_2_16.u8[1] = buffer[3];
                g_effect_pool[buffer[1] - 1].constant_force_report.magnitude = u8_2_16.i16;
                break;
            }
            case (HID_ID_EFOPREP + 0x10 * TLID): {
                memcpy(g_effect_pool[buffer[1] - 1].operation_report_raw, &buffer[1], OPERATION_REPORT_LEN);
                break;
            }
            case (HID_ID_BLKFRREP + 0x10 * TLID): {
                ffb_effect_t* e = &g_effect_pool[buffer[1] - 1];
                memset(e, 0, sizeof(ffb_effect_t));
                break;
            }
            case (HID_ID_EFFREP + 0x10 * TLID): {
                memcpy(g_effect_pool[buffer[1] - 1].effect_report_raw, &buffer[1], EFFECT_REPORT_LEN);
                break;
            }
            case (HID_ID_CONDREP + 0x10 * TLID): {
                memcpy(g_effect_pool[buffer[1] - 1].condition_report_raw, &buffer[1], CONDITION_REPORT_LEN);
                break;
            }
            default: {
                ESP_LOGI("HID", "Unimplemented SET_REPORT HID_REPORT_TYPE_OUTPUT: %d", buffer[0]);
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
    // tusb_cfg.descriptor.device = NULL;
    tusb_cfg.descriptor.device = &desc_device;
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

static void wheel_read_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        if (!(tud_mounted() && tud_hid_ready())) {
            continue;
        }
        float wheel_rad = as5600.getAngle();
        static float wheel_rad_last;
        if (fabsf(wheel_rad_last - wheel_rad) < 0.002f) {
            continue;
        }
        wheel_rad_last = wheel_rad;
        // ESP_LOGI("FFB", "A%f", wheel_rad);
        hid_joystick_input_t joy = {.axis_x = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_y = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_z = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_rx = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_ry = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_rz = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .slider = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .dial = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .pov = 8};
        float wheel_rad_clamped = wheel_rad > WHEEL_HALF ? WHEEL_HALF : (wheel_rad < -WHEEL_HALF ? -WHEEL_HALF : wheel_rad);
        joy.axis_x = JOYSTIC_AXIS_LOGICAL_MID + wheel_rad_clamped / WHEEL_HALF * JOYSTIC_AXIS_LOGICAL_MID;
        tud_hid_report(TLID, &joy, sizeof(hid_joystick_input_t));
    }
}

static void foc_task(void* arg) {
    // initialise magnetic sensor hardware
    as5600.init();
    // link the motor to the sensor
    motor.linkSensor(&as5600);

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

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));

        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop  ~1kHz
        // Bluepill loop ~10kHz
        motor.loopFOC();

        damper = damper < 0.2f ? 0.2f : damper;
        float damping = damper * DAMPING_GAIN * motor.shaft_velocity / DAMPING_MAX_VELOCITY;
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

//******************************** MAIN Function //********************************

extern "C" void app_main(void) {
    xTaskCreate(usb_task, "usb_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(foc_task, "foc_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(wheel_read_task, "wheel_read_task", TASK_STACK_SIZE, NULL, 10, NULL);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}
