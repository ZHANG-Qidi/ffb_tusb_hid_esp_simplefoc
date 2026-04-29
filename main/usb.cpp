/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "esp_log.h"
#include "ffb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hidReportDesc.h"
#include "interface.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
static const char* TAG = "ffb_usb";
//******************************** tinyUSB Input //********************************
TaskHandle_t tiny_usb_task_handle;
//******************************** tinyUSB Output //********************************
//******************************** tinyUSB Configuration //********************************
/* Flag to indicate if the host has suspended the USB bus */
static bool suspended = false;
/* Flag of possibility to Wakeup Host via Remote Wakeup feature */
static bool wakeup_host = false;
/************* TinyUSB descriptors ****************/
#define TUSB_DESC_IN_OUT_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_INOUT_DESC_LEN)
/**
 * @brief String descriptor
 */
static const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},     // 0: is supported language is English (0x0409)
    "TinyUSB",                // 1: Manufacturer
    "TinyUSB Device",         // 2: Product
    "123456",                 // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};
/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_IN_OUT_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    // HID Input & torque_ratio descriptor
    // Interface number, string index, protocol, report descriptor len, EP OUT & IN address, size & polling interval
    TUD_HID_INOUT_DESCRIPTOR(0, 4, false, sizeof(G_DefaultReportDescriptor), 0x81, 0x01, 64, 1),
};
// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return G_DefaultReportDescriptor;
}
// USB device descriptor for the Microsoft SideWinder FFB (VID 0x045E, PID 0x0034)
static tusb_desc_device_t const desc_device = {.bLength = sizeof(tusb_desc_device_t),
                                               .bDescriptorType = TUSB_DESC_DEVICE,
                                               .bcdUSB = 0x0200,
                                               .bDeviceClass = 0x00,
                                               .bDeviceSubClass = 0x00,
                                               .bDeviceProtocol = 0x00,
                                               .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
                                               .idVendor = 0x045E,   // ms_sidewinder_ffb
                                               .idProduct = 0x0034,  // ms_sidewinder_ffb
                                               .bcdDevice = 0x0100,
                                               .iManufacturer = 0x01,
                                               .iProduct = 0x02,
                                               .iSerialNumber = 0x03,
                                               .bNumConfigurations = 0x01};
//******************************** tinyUSB Function //********************************
static void dump_hex(const uint8_t* buf, int len) {
    char line[256];
    int pos = 0;
    for (int i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", buf[i]);
    }
    ESP_LOGI(TAG, "%s", line);
}
// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    ESP_LOGI(TAG, "GET_REPORT: inst=%u id=%u type=%u len=%u", instance, report_id, report_type, reqlen);
    dump_hex(buffer, reqlen);
    if (report_type == HID_REPORT_TYPE_INPUT) {
    }
    if (report_type == HID_REPORT_TYPE_FEATURE) {
        return ffb_get_feature(report_id, buffer);
    }
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
    }
    return 0;
}
// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    // ESP_LOGI(TAG, "SET_REPORT: inst=%u id=%u type=%u size=%u", instance, report_id, report_type, bufsize);
    // dump_hex(buffer, bufsize);
    if (report_type == HID_REPORT_TYPE_INPUT) {
    }
    if (report_type == HID_REPORT_TYPE_FEATURE) {
        ESP_LOGI(TAG, "SET_REPORT: inst=%u id=%u type=%u size=%u", instance, report_id, report_type, bufsize);
        dump_hex(buffer, bufsize);
        ffb_set_feature(report_id, buffer);
    }
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
        ffb_set_output(buffer);
    }
}
void tud_suspend_cb(bool remote_wakeup_en) {
    ESP_LOGI(TAG, "USB device suspended");
    suspended = true;
    if (remote_wakeup_en) {
        ESP_LOGI(TAG, "Remote wakeup available, press the button to wake up the Host");
        wakeup_host = true;
    } else {
        ESP_LOGI(TAG, "Remote wakeup not available");
    }
}
void tud_resume_cb(void) {
    ESP_LOGI(TAG, "USB device resumed");
    suspended = false;
}
static void usb_task(void* arg) {
    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    // tusb_cfg.descriptor.device = NULL;
    tusb_cfg.descriptor.device = &desc_device;
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif  // TUD_OPT_HIGH_SPEED
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    for (;;) {
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);
        if (!(tud_mounted() && tud_hid_ready())) {
            continue;
        }
        hid_joystick_input_t joy = {.axis_x = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_y = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_z = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_rx = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_ry = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .axis_rz = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .slider = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .dial = (uint32_t)JOYSTIC_AXIS_LOGICAL_MID,
                                    .pov = 8};
        float wheel_rad;
        motor_output(&wheel_rad);
        float wheel_rad_clamped = wheel_rad > WHEEL_HALF ? WHEEL_HALF : (wheel_rad < -WHEEL_HALF ? -WHEEL_HALF : wheel_rad);
        joy.axis_x = JOYSTIC_AXIS_LOGICAL_MID + wheel_rad_clamped / WHEEL_HALF * JOYSTIC_AXIS_LOGICAL_MID;
        tud_hid_report(TLID, &joy, sizeof(hid_joystick_input_t));
    }
}
void tiny_usb_init(void) {
    xTaskCreate(usb_task, "usb_task", TASK_STACK_SIZE, NULL, 10, &tiny_usb_task_handle);
    vTaskDelay(pdMS_TO_TICKS(500));
}
