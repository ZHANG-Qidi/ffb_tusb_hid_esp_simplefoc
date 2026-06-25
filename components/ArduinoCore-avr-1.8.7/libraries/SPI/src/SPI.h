/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

#include "arduino_interface.h"
#include "driver/spi_master.h"

class SPISettings {
   public:
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {}
    SPISettings() {}

   private:
    friend class SPIClass;
};

class SPIClass {
   public:
    // Initialize the SPI library
    static void begin();

    // If SPI is used from within an interrupt, this function registers
    // that interrupt with the SPI library, so beginTransaction() can
    // prevent conflicts.  The input interruptNumber is the number used
    // with attachInterrupt.  If SPI is used from a different interrupt
    // (eg, a timer), interruptNumber should be 255.
    static void usingInterrupt(uint8_t interruptNumber);
    // And this does the opposite.
    static void notUsingInterrupt(uint8_t interruptNumber);
    // Note: the usingInterrupt and notUsingInterrupt functions should
    // not to be called from ISR context or inside a transaction.
    // For details see:
    // https://github.com/arduino/Arduino/pull/2381
    // https://github.com/arduino/Arduino/pull/2449

    // Before using SPI.transfer() or asserting chip select pins,
    // this function is used to gain exclusive access to the SPI bus
    // and configure the correct settings.
    inline static void beginTransaction(SPISettings settings) {}

    // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
    inline static uint8_t transfer(uint8_t data) { return data; }

    inline uint16_t transfer16(uint16_t data) {
        uint8_t tx[2] = {uint8_t(data >> 8), uint8_t(data & 0xFF)};
        uint8_t rx[2] = {0, 0};
        spi_transaction_t t = {};
        t.length = 16;
        t.tx_buffer = tx;
        t.rxlength = 16;
        t.rx_buffer = rx;
        spi_device_polling_transmit(mt6701, &t);
        return (uint16_t(rx[0]) << 8) | rx[1];
    }
    inline static void transfer(void *buf, size_t count) {}
    // After performing a group of transfers and releasing the chip select
    // signal, this function allows others to access the SPI bus
    inline static void endTransaction(void) {}

    // Disable the SPI bus
    static void end();

    // This function is deprecated.  New applications should use
    // beginTransaction() to configure SPI settings.
    inline static void setBitOrder(uint8_t bitOrder) {}
    // This function is deprecated.  New applications should use
    // beginTransaction() to configure SPI settings.
    inline static void setDataMode(uint8_t dataMode) {}
    // This function is deprecated.  New applications should use
    // beginTransaction() to configure SPI settings.
    inline static void setClockDivider(uint8_t clockDiv) {}
    // These undocumented functions should not be used.  SPI.transfer()
    // polls the hardware flag which is automatically cleared as the
    // AVR responds to SPI's interrupt
    inline static void attachInterrupt() {}
    inline static void detachInterrupt() {}

   private:
    static spi_device_handle_t mt6701;
};

extern SPIClass SPI;

#endif
