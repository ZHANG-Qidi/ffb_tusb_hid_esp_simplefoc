#include "ffb.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hidReportDesc.h"
#include "interface.h"
static const char* TAG = "ffb_ffb";
//******************************** FFB Configuration //********************************
#define MOTOR_GAIN_DAMPING (0.5f)
#define MOTOR_GAIN_CONSTANT (0.2f)
//******************************** FFB Private //********************************
static uint8_t g_effect_type;
static uint8_t g_effect_block_index;
static uint8_t g_effect_block_status;
static uint8_t g_gain_device;
static ffb_effect_t g_effect_pool[FFB_EFFECT_COUNT];
//******************************** FFB Output //********************************
static float g_constant_force;
static float g_damper = MOTOR_DAMPING_MIN;
void tiny_usb_output(float* constant_force, float* damper) {
    *constant_force = g_constant_force;
    *damper = g_damper;
}
//******************************** FFB Shared //********************************
//******************************** FFB Function //********************************
void ffb_mixer(void) {
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
                float constant_force = magnitude * gain_effect_scale * gain_device_scale / CONSTANT_MANITUDE_MAX * MOTOR_GAIN_CONSTANT;
                g_constant_force = constant_force;
                break;
            }
            case ET_DAMPER_DR2: {
                float damper = (float)g_effect_pool[i].condition_report.positive_coefficient / (float)g_effect_pool[i].condition_report.positive_saturation * MOTOR_GAIN_DAMPING;
                g_damper = damper < MOTOR_DAMPING_MIN ? MOTOR_DAMPING_MIN : damper;
                break;
            }
            default: {
                ESP_LOGI(TAG, "Unimplemented Effect: %d", g_effect_pool[i].effect_report.type);
                break;
            }
        }
    }
    xTaskNotify(*motor_task_handle, 0, eSetBits);
}
uint16_t ffb_get_feature(uint8_t report_id, uint8_t* buffer) {
    switch (report_id) {
        case (HID_ID_POOLREP + 0x10 * TLID): {
            buffer[0] = 0xff;
            buffer[1] = 0xff;
            buffer[2] = FFB_EFFECT_COUNT;
            buffer[3] = DEVICE_MANAGED_POOL;
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
            ESP_LOGI(TAG, "Unimplemented GET_REPORT HID_REPORT_TYPE_FEATURE: %d", report_id);
            break;
        }
    }
    return 0;
}
void ffb_set_feature(uint8_t report_id, const uint8_t* buffer) {
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
            ESP_LOGI(TAG, "Unimplemented SET_REPORT HID_REPORT_TYPE_FEATURE: %d", report_id);
            break;
        }
    }
}
void ffb_set_output(const uint8_t* buffer) {
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
                    break;
                }
                case DC_DEVICE_RESET:
                    break;
                case DC_DEVICE_PAUSE:
                    break;
                case DC_DEVICE_CONTINUE:
                    break;
                default:
                    ESP_LOGI(TAG, "Unimplemented Usage PID Device Control: %d", buffer[1]);
                    break;
            }
            break;
        case (HID_ID_GAINREP + 0x10 * TLID): {
            g_gain_device = buffer[1];
            break;
        }
        case (HID_ID_CONSTREP + 0x10 * TLID): {
            memcpy(g_effect_pool[buffer[1] - 1].constant_force_report_raw, &buffer[1], CONSTANT_FORCE_REPORT_LEN);
            break;
        }
        case (HID_ID_EFOPREP + 0x10 * TLID): {
            memcpy(g_effect_pool[buffer[1] - 1].operation_report_raw, &buffer[1], OPERATION_REPORT_LEN);
            ffb_mixer();
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
            ESP_LOGI(TAG, "Unimplemented SET_REPORT HID_REPORT_TYPE_OUTPUT: %d", buffer[0]);
            break;
        }
    }
}
