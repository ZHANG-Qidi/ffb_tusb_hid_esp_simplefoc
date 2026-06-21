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

#include "SPI.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

SPIClass SPI;

spi_device_handle_t SPIClass::mt6701 = nullptr;

void SPIClass::begin() {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI_MASTER_MOSI_IO;
    buscfg.miso_io_num = SPI_MASTER_MISO_IO;
    buscfg.sclk_io_num = SPI_MASTER_SCLK_IO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    spi_bus_initialize(SPI_MASTER_NUM, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 2 * 1000 * 1000;
    devcfg.mode = 1;
    devcfg.spics_io_num = SPI_MASTER_CS_IO;
    devcfg.queue_size = 1;
    spi_bus_add_device(SPI_MASTER_NUM, &devcfg, &mt6701);
}

void SPIClass::end() {
    spi_bus_remove_device(mt6701);
    spi_bus_free(SPI_MASTER_NUM);
    gpio_reset_pin((gpio_num_t)SPI_MASTER_MOSI_IO);
    gpio_reset_pin((gpio_num_t)SPI_MASTER_MISO_IO);
    gpio_reset_pin((gpio_num_t)SPI_MASTER_SCLK_IO);
    gpio_reset_pin((gpio_num_t)SPI_MASTER_CS_IO);
}

void SPIClass::usingInterrupt(uint8_t interruptNumber) {}

void SPIClass::notUsingInterrupt(uint8_t interruptNumber) {}
