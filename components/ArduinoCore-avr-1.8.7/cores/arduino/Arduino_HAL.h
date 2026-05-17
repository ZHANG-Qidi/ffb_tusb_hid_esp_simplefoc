#ifndef _ARDUINO_HAL_H_
#define _ARDUINO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// PROGMEM / pgm_read_*
#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t *)(uintptr_t)(addr))
#define pgm_read_word(addr) (*(const uint16_t *)(uintptr_t)(addr))

// Interrupt control (dummy)
#define sei() ((void)0)
#define cli() ((void)0)

// PSTR / PGM_P
#define PSTR(s) (s)
typedef const char *PGM_P;

// Bit utilities
#define _BV(bit) (1U << (bit))
#define _SFR_BYTE(sfr) (*(volatile uint8_t *)(uintptr_t)(sfr))
#define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & _BV(bit)))

// CPU settings
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define SREG_I 7
extern volatile uint8_t SREG;

// SPI registers (dummy externs)
extern volatile uint8_t SPCR;
extern volatile uint8_t SPSR;
extern volatile uint8_t SPDR;

#ifdef SPI_AVR_EIMSK
extern volatile uint8_t SPI_AVR_EIMSK;
#endif

// SPI bit definitions
#define SPE 6
#define MSTR 4
#define DORD 5
#define CPOL 3
#define CPHA 2
#define SPR1 1
#define SPR0 0

#define SPI2X 0
#define SPIF 7
#define SPIE 7

// SPI clock dividers
#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

// SPI modes
#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

// SPI masks
#define SPI_MODE_MASK 0x0C
#define SPI_CLOCK_MASK 0x03
#define SPI_2XCLOCK_MASK 0x01

// USART registers (dummy externs)
extern volatile uint8_t UBRRH;
extern volatile uint8_t UBRRL;
extern volatile uint8_t UCSRA;
extern volatile uint8_t UCSRB;
extern volatile uint8_t UCSRC;
extern volatile uint8_t UDR;

extern volatile uint8_t UBRR0H;
extern volatile uint8_t UBRR0L;
extern volatile uint8_t UCSR0A;
extern volatile uint8_t UCSR0B;
extern volatile uint8_t UCSR0C;
extern volatile uint8_t UDR0;

// USART bit definitions (UCSR0A)
#define MPCM0 0
#define U2X0 1
#define UPE0 2
#define DOR0 3
#define FE0 4
#define UDRE0 5
#define TXC0 6
#define RXC0 7

// USART bit definitions (UCSR0B)
#define TXB80 0
#define RXB80 1
#define UCSZ02 2
#define TXEN0 3
#define RXEN0 4
#define UDRIE0 5
#define TXCIE0 6
#define RXCIE0 7

// USART bit definitions (UCSR0C)
#define UCPOL0 0
#define UCSZ00 1
#define UCSZ01 2
#define USBS0 3
#define UPM00 4
#define UPM01 5
#define UMSEL00 6
#define UMSEL01 7

// sbi / cbi
#define sbi(reg, bit) ((reg) |= _BV(bit))
#define cbi(reg, bit) ((reg) &= ~_BV(bit))

// ATOMIC_BLOCK dummy
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(x) for (int _i = 0; _i < 1; _i++)

// Interrupt vectors (dummy)
#define USART_RX_vect
#define USART0_RX_vect
#define USART_RXC_vect
#define UART0_UDRE_vect
#define UART_UDRE_vect
#define USART0_UDRE_vect
#define USART_UDRE_vect

// Arduino pin helper
#define PIN(port, num) (((port[0] - 'A') * 16) + (num))

// AVR port macros (needed by pins_arduino.h)
#define PB 2
#define PC 3
#define PD 4

#define ARDUINO_MAIN

#ifdef __cplusplus
}
#endif

#endif
