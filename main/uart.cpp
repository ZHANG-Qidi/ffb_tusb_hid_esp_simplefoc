#include "driver/uart.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "interface.h"
static const char* TAG = "ffb_uart";
//******************************** UART Configure //********************************
#define UART_TXD (CONFIG_FFB_UART_TXD)
#define UART_RXD (CONFIG_FFB_UART_RXD)
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)
#define UART_PORT_NUM ((uart_port_t)CONFIG_FFB_UART_PORT_NUM)
#define UART_BAUD_RATE (CONFIG_FFB_UART_BAUD_RATE)
#define UART_TASK_STACK_SIZE (CONFIG_FFB_UART_TASK_STACK_SIZE)
#define BUF_SIZE (1024)
//******************************** UART Input //********************************
TaskHandle_t uart_write_task_handle;
//******************************** UART Output //********************************
static float g_wheel_rad;
void uart_backend_output(float* wheel_rad) { *wheel_rad = g_wheel_rad; }
//******************************** UART Function //********************************
static void uart_read_task(void* arg) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 20, &uart_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT_NUM, 1));
    // Configure a temporary buffer for the incoming data
    char buf[BUF_SIZE];
    uart_event_t event;
    for (;;) {
        // Read data from the UART
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    // ESP_LOGI(TAG, "RX %d bytes\n", event.size);
                    int len = uart_read_bytes(UART_PORT_NUM, buf, event.size, 0);
                    buf[len] = '\0';
                    // ESP_LOGI(TAG, "%s", buf);
                    if (buf[0] == 'A') {
                        g_wheel_rad = strtof(&buf[1], NULL);
                        xTaskNotify(*ffb_task_handle, 0, eSetBits);
                    }
                    break;
                }
                case UART_FIFO_OVF: {
                    ESP_LOGI(TAG, "FIFO overflow");
                    break;
                }
                case UART_BUFFER_FULL: {
                    ESP_LOGI(TAG, "Buffer full");
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
}
static void uart_write_task(void* arg) {
    for (;;) {
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);
        float constant_force;
        float damper;
        ffb_output(&constant_force, &damper);
        // Write data to the UART
        char data[64];
        sprintf(data, "F%f\n", constant_force);
        uart_write_bytes(UART_PORT_NUM, (const char*)data, strlen(data));
        sprintf(data, "D%f\n", damper);
        uart_write_bytes(UART_PORT_NUM, (const char*)data, strlen(data));
    }
}
void uart_backend_init(void) {
    xTaskCreatePinnedToCore(uart_read_task, "uart_read_task", TASK_STACK_SIZE, NULL, 10, NULL, CORE_0);
    xTaskCreatePinnedToCore(uart_write_task, "uart_write_task", TASK_STACK_SIZE, NULL, 10, &uart_write_task_handle, CORE_0);
    vTaskDelay(pdMS_TO_TICKS(500));
}
