#include "Arduino_HAL.h"

#include <stdint.h>

#include "Arduino.h"
#include "Arduino_interface.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
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

#define FOC_MCPWM_TIMER_RESOLUTION_HZ (40000000)
#define FOC_MCPWM_PERIOD (1000)

#if CONFIG_SOC_MCPWM_SUPPORTED

static mcpwm_cmpr_handle_t comparators[3];

static void analogWriteInitMcpwm(uint8_t pin_u, uint8_t pin_v, uint8_t pin_w) {
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_config = {};
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.group_id = 0;
    timer_config.resolution_hz = FOC_MCPWM_TIMER_RESOLUTION_HZ;
    timer_config.period_ticks = FOC_MCPWM_PERIOD;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t operators[3];
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0;  // operator should be in the same group of the above timers
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    mcpwm_comparator_config_t compare_config = {};
    compare_config.flags.update_cmp_on_tez = true;
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 0));
    }

    mcpwm_gen_handle_t generators[3];
    int gen_gpios[3];
    gen_gpios[pin_u % 3] = pin_u;
    gen_gpios[pin_v % 3] = pin_v;
    gen_gpios[pin_w % 3] = pin_w;
    mcpwm_generator_config_t gen_config = {};
    for (int i = 0; i < 3; i++) {
        gen_config.gen_gpio_num = gen_gpios[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i]));
    }

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators[i], MCPWM_GEN_ACTION_HIGH)));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void analogWrite(uint8_t pin, int value) {
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    uint8_t index = pin % 3;
    uint32_t level = value / 255.0f * (float)(FOC_MCPWM_PERIOD) / 2.0f;
    // Regular uvw data to (0 ~ (EXAMPLE_FOC_MCPWM_PERIOD/2))
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[index], level));
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
void _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
    (void)(pwm_frequency);
    analogWriteInitMcpwm(pinA, pinB, pinC);
}

#else

#define FOC_PWM_TIMER_FREQ_HZ (FOC_MCPWM_TIMER_RESOLUTION_HZ / FOC_MCPWM_PERIOD)

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
        timer.freq_hz = FOC_PWM_TIMER_FREQ_HZ;
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
#endif
