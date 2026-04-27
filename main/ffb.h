#ifndef _FFB_H_
#define _FFB_H_

#include <math.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//******************************** USB JOYSTICK INPUT REPORT //********************************

#define JOYSTIC_AXIS_LOGICAL_MID (16383.5f)
#define JOYSTIC_AXIS_LOGICAL_MAX (32767.0f)
#define WHEEL_HALF (M_PI * 4.0f)

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

//******************************** USB PID REPORT //********************************

#define GAIN_EFFECT_LOGICAL_MAX (255.0f)
#define GAIN_EFFECT_PHYSICAL_MAX (10000.0f)

#define GAIN_DEVICE_LOGICAL_MAX (255.0f)
#define GAIN_DEVICE_PHYSICAL_MAX (10000.0f)

#define CONSTANT_MANITUDE_MAX (10000.0f)

#define ET_DAMPER_DR2 (0x0b)

// Usage PID Device Control
typedef enum { DC_ENABLE_ACTUATORS = 1, DC_DISABLE_ACTUATORS, DC_STOP_ALL_EFFECTS, DC_DEVICE_RESET, DC_DEVICE_PAUSE, DC_DEVICE_CONTINUE } PID_Device_Control;

// Usage Effect Type
typedef enum { ET_CONSTANT = 1, ET_RAMP, ET_SQUARE, ET_SINE, ET_TRIANGLE, ET_SAWTOOTH_UP, ET_SAWTOOTH_DOWN, ET_SPRING, ET_DAMPER, ET_INERTIA, ET_FRICTION, ET_CUSTOM } Effect_Type;

// Usage Block Status
typedef enum { BLOCK_FREE, BLOCK_ACCLOCATED } BLOCK_STATUS;

// Usage Block Load Status
typedef enum { BLOCK_LOAD_SUCCESS = 1, BLOCK_LOAD_FULL, BLOCK_LOAD_ERROR } BLOCK_LOAD_STATUS;

// Usage Device Managed Pool or Usage Shared Parameter Blocks
typedef enum { DEVICE_MANAGED_POOL = 1, SHARED_PARAMETER_BLOCKS } POOL_TYPE;

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

// Usage Set Constant Force Report
typedef struct __attribute__((packed)) {
    uint8_t index;      // Usage Effect Block Index
    int16_t magnitude;  // Usage Magnitude
} constant_force_report_t;

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
#define CONSTANT_FORCE_REPORT_LEN (3)
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
        union {
            uint8_t constant_force_report_raw[CONSTANT_FORCE_REPORT_LEN];
            constant_force_report_t constant_force_report;  // Usage Set Constant Force Report
        };
        union {
            uint8_t condition_report_raw[CONDITION_REPORT_LEN];
            condition_report_t condition_report;  // Usage Set Condition Report
        };
    };
} ffb_effect_t;

#define FFB_EFFECT_COUNT 32

uint16_t ffb_get_feature(uint8_t report_id, uint8_t* buffer);
void ffb_set_feature(uint8_t report_id, const uint8_t* buffer);
void ffb_set_output(const uint8_t* buffer);

#endif
