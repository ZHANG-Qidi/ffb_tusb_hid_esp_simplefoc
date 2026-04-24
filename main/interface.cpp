#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t uart_write_task_handle;
extern void uart_output(float* wheel_rad);
extern void uart_foc_init(void);

extern TaskHandle_t foc_task_handle;
extern void foc_output(float* wheel_rad);
extern void foc_init(void);

TaskHandle_t* motor_task_handle;
void (*motor_output)(float* wheel_rad);
void (*motor_init)(void);

extern void tiny_usb_init(void);
extern TaskHandle_t tiny_usb_task_handle;

TaskHandle_t* usb_task_handle;
void (*usb_init)(void);

void interface_init(void) {
#ifdef CONFIG_IDF_TARGET_ESP32S2
    motor_task_handle = &uart_write_task_handle;
    motor_output = uart_output;
    motor_init = uart_foc_init;
#elif CONFIG_IDF_TARGET_ESP32S3
    motor_task_handle = &foc_task_handle;
    motor_output = foc_output;
    motor_init = foc_init;
#endif
    usb_init = tiny_usb_init;
    usb_task_handle = &tiny_usb_task_handle;
}
