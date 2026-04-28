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
#ifdef __cplusplus
extern "C" {
#endif
extern void interval_print(const char* TAG);
#ifdef __cplusplus
}
#endif
#endif
