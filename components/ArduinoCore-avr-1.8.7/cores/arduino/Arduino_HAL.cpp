#include "Arduino_HAL.h"

#include "Arduino.h"
#include "Arduino_interface.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CPU / Global settings
volatile uint8_t SREG;

// USART registers (dummy AVR compatibility)
volatile uint8_t UBRRH;
volatile uint8_t UBRRL;
volatile uint8_t UCSRA;
volatile uint8_t UCSRB;
volatile uint8_t UCSRC;
volatile uint8_t UDR;

volatile uint8_t UBRR0H;
volatile uint8_t UBRR0L;
volatile uint8_t UCSR0A;
volatile uint8_t UCSR0B;
volatile uint8_t UCSR0C;
volatile uint8_t UDR0;

// SPI registers (dummy AVR compatibility)
volatile uint8_t SPCR;
volatile uint8_t SPSR;
volatile uint8_t SPDR;

void delayMicroseconds(unsigned int us) { esp_rom_delay_us(us); }

void delay(unsigned long ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

unsigned long micros(void) { return (unsigned long)esp_timer_get_time(); }

void digitalWrite(uint8_t pin, uint8_t val) { gpio_set_level((gpio_num_t)pin, val ? 1 : 0); }

void pinMode(uint8_t pin, uint8_t mode) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    switch (mode) {
        case OUTPUT:
            cfg.mode = GPIO_MODE_OUTPUT;
            break;
        case INPUT:
            cfg.mode = GPIO_MODE_INPUT;
            break;
        case INPUT_PULLUP:
            cfg.mode = GPIO_MODE_INPUT;
            cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            break;
        case INPUT_PULLDOWN:
            cfg.mode = GPIO_MODE_INPUT;
            cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
    }
    gpio_config(&cfg);
}

void analogWrite(uint8_t pin, int value) {
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    uint8_t channel = pin % 8;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

void analogWriteInit(uint8_t pin) {
    uint8_t channel = pin % 8;
    static bool ledc_initialized[8] = {0};
    if (!ledc_initialized[channel]) {
        ledc_timer_config_t timer = {};
        timer.speed_mode = LEDC_LOW_SPEED_MODE;
        timer.duty_resolution = LEDC_TIMER_8_BIT;
        timer.timer_num = LEDC_TIMER_0;
        timer.freq_hz = 20000;
        timer.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timer));

        ledc_channel_config_t ch = {};
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel = (ledc_channel_t)channel;
        ch.timer_sel = LEDC_TIMER_0;
        ch.gpio_num = pin;
        ch.duty = 0;
        ch.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        ledc_initialized[channel] = true;
    }
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
void _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
    (void)(pwm_frequency);
    analogWriteInit(pinA);
    analogWriteInit(pinB);
    analogWriteInit(pinC);
}
