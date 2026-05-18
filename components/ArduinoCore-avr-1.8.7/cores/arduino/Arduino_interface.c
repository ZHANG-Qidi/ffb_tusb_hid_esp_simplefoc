#include "Arduino_interface.h"

#include <stdio.h>

#include "driver/uart.h"

int __io_putchar(int ch) {
    uart_write_bytes(UART_MASTER_NUM, &ch, 1);
    return ch;
}
