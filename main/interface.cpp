#include "interface.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//******************************** Motor Backend //********************************
extern TaskHandle_t uart_write_task_handle;
extern void uart_backend_output(float* wheel_rad);
extern void uart_backend_init(void);
extern TaskHandle_t foc_task_handle;
extern void foc_backend_output(float* wheel_rad);
extern void foc_backend_init(void);
extern TaskHandle_t espnow_write_task_handle;
extern "C" {
extern void espnow_backend_output(float* wheel_rad);
extern void espnow_backend_init(void);
}
TaskHandle_t* motor_task_handle;
void (*motor_output)(float* wheel_rad);
void (*motor_init)(void);
//******************************** FFB Backend //********************************
extern TaskHandle_t tiny_usb_task_handle;
extern void tiny_usb_init(void);
extern void tiny_usb_output(float* constant_force, float* damper);
TaskHandle_t* ffb_task_handle;
void (*ffb_init)(void);
void (*ffb_output)(float* constant_force, float* damper);
//******************************** Interface Function //********************************
void interface_init(void) {
#if ESPNOW_BACKEND
    motor_task_handle = &espnow_write_task_handle;
    motor_output = espnow_backend_output;
    motor_init = espnow_backend_init;
#elif CONFIG_IDF_TARGET_ESP32S2
    motor_task_handle = &uart_write_task_handle;
    motor_output = uart_backend_output;
    motor_init = uart_backend_init;
#elif CONFIG_IDF_TARGET_ESP32S3
    motor_task_handle = &foc_task_handle;
    motor_output = foc_backend_output;
    motor_init = foc_backend_init;
#endif
    ffb_task_handle = &tiny_usb_task_handle;
    ffb_output = tiny_usb_output;
    ffb_init = tiny_usb_init;
}
void interval_print(const char* TAG) {
    static uint64_t last = 0;
    uint64_t now = esp_timer_get_time();
    uint64_t dt = now - last;
    last = now;
    ESP_LOGI(TAG, "dt = %llu us", dt);
}
