/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#include "HardwareSerial.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Arduino.h"
#include "HardwareSerial_private.h"
#include "arduino_interface.h"
#include "driver/uart.h"

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerial::_rx_complete_irq(void) {}

void HardwareSerial::_tx_udr_empty_irq(void) {}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerial::begin(unsigned long baud, byte config) {
    (void)baud;
    (void)config;
    _written = false;
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_driver_install(UART_MASTER_NUM, SERIAL_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_MASTER_NUM, &cfg);
    uart_set_pin(UART_MASTER_NUM, UART_MASTER_TX_IO, UART_MASTER_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void HardwareSerial::end() {
    if (uart_is_driver_installed(UART_MASTER_NUM)) {
        uart_driver_delete(UART_MASTER_NUM);
    }
}

int HardwareSerial::available(void) {
    size_t available;
    uart_get_buffered_data_len(UART_MASTER_NUM, &available);
    return available;
}

int HardwareSerial::peek(void) { return 1; }

int HardwareSerial::read(void) {
    uint8_t c = 0;
    if (uart_read_bytes(UART_MASTER_NUM, &c, 1, 10 / portTICK_PERIOD_MS) == 1) {
        return c;
    }
    return -1;
}

int HardwareSerial::availableForWrite(void) { return 1; }

void HardwareSerial::flush() {}

size_t HardwareSerial::write(uint8_t c) {
    uart_write_bytes(UART_MASTER_NUM, &c, 1);
    return 1;
}

HardwareSerial Serial;