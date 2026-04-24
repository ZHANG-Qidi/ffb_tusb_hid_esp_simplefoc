#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t* motor_task_handle;
extern void (*motor_output)(float* wheel_rad);
extern void (*motor_init)(void);

extern TaskHandle_t* usb_task_handle;
extern void (*usb_init)(void);

extern void interface_init(void);

#endif