#ifndef _INTERFACE_H_
#define _INTERFACE_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//******************************** FreeRTOS Configuration //********************************
#define TASK_STACK_SIZE (4096)
//******************************** Motor Backend Configuration //********************************
#define MOTOR_DAMPING_MIN (0.1f)
#define ESPNOW_BACKEND 1
extern TaskHandle_t* motor_task_handle;
extern void (*motor_output)(float* wheel_rad);
extern void (*motor_init)(void);
extern TaskHandle_t* ffb_task_handle;
extern void (*ffb_output)(float* constant_force, float* damper);
extern void (*ffb_init)(void);
extern void interface_init(void);
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S3
#define CORE_0 (0)
#define CORE_1 (1)
#elif CONFIG_IDF_TARGET_ESP32S2
#define CORE_0 (tskNO_AFFINITY)
#define CORE_1 (tskNO_AFFINITY)
#endif
#endif
