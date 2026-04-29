#include "debug.h"
void interval_print(const char* TAG) {
    static uint64_t last = 0;
    uint64_t now = esp_timer_get_time();
    uint64_t dt = now - last;
    last = now;
    ESP_LOGI(TAG, "dt = %llu us", dt);
}
