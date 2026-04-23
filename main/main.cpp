/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//******************************** FreeRTOS Configuration //********************************

#define TASK_STACK_SIZE (4096)

//******************************** MAIN Shared //********************************

TaskHandle_t usb_task_handle;
extern void usb_task(void* arg);
extern void ffb_task(void* arg);
extern void get_angle_task(void* arg);
extern void foc_task(void* arg);

//******************************** MAIN Function //********************************

extern "C" void app_main(void) {
    xTaskCreate(usb_task, "usb_task", TASK_STACK_SIZE, NULL, 10, &usb_task_handle);
    xTaskCreate(ffb_task, "ffb_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(get_angle_task, "get_angle_task", TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(foc_task, "foc_task", TASK_STACK_SIZE, NULL, 10, NULL);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}
