#include "Arduino_interface.h"

#include <stdio.h>

#include "driver/uart.h"

void arduino_serial_init(void) {}

void arduino_spi_init(void) {}

void arduino_tim_init(void) {}

void arduino_hi2c_init(void) {}

int __io_putchar(int ch) {
    uart_write_bytes(UART_MASTER_NUM, &ch, 1);
    return ch;
}
