#include "ffb.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//******************************** FFB Configuration //********************************

#define MOTOR_GAIN_DAMPING (1.0f)
#define MOTOR_GAIN_CONSTANT (0.2f)

//******************************** FFB Input //********************************

uint8_t g_effect_type;
uint8_t g_effect_block_index;
uint8_t g_effect_block_status;
uint8_t g_gain_device;
ffb_effect_t g_effect_pool[FFB_EFFECT_COUNT];

//******************************** FFB Output //********************************

float g_damper;
float g_constant_force;

//******************************** FFB Shared //********************************

portMUX_TYPE ffb_spinlock = portMUX_INITIALIZER_UNLOCKED;

//******************************** FFB Function //********************************

void ffb_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
        portENTER_CRITICAL(&ffb_spinlock);
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
                    float damper =
                        (float)g_effect_pool[i].condition_report.positive_coefficient / (float)g_effect_pool[i].condition_report.positive_saturation * MOTOR_GAIN_DAMPING;
                    g_damper = damper < 0.1f ? 0.1f : damper;
                    break;
                }
                default: {
                    ESP_EARLY_LOGI("FFB", "Unimplemented Effect: %d", g_effect_pool[i].effect_report.type);
                    break;
                }
            }
        }
        portEXIT_CRITICAL(&ffb_spinlock);
    }
}
