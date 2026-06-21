#ifndef _ARDUINO_HAL_H_
#define _ARDUINO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// pgm_read_*
#define pgm_read_byte(addr) (*(const uint8_t *)(uintptr_t)(addr))
#define pgm_read_word(addr) (*(const uint16_t *)(uintptr_t)(addr))

// PSTR / PGM_P
#define PSTR(s) (s)
typedef const char *PGM_P;

// SPI modes
#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

// Arduino pin helper
#define PIN(port, num) (((port[0] - 'A') * 16) + (num))

#ifdef __cplusplus
}
#endif

#endif
