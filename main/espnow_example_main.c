/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_crc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "espnow_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "interface.h"
#include "nvs_flash.h"

#define ESPNOW_MAXDELAY 512

static const char* TAG = "espnow_example";

TaskHandle_t espnow_write_task_handle;

static motor_output_t g_motor_output_data;

void espnow_backend_output(float* wheel_rad) { *wheel_rad = g_motor_output_data.wheel_rad; }

static SemaphoreHandle_t g_send_done_sem = NULL;

//******************************** ESPNOW //********************************

static QueueHandle_t s_example_espnow_queue = NULL;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = {0, 0};

static void example_espnow_deinit(example_espnow_send_param_t* send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t* send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t* recv_cb = &evt.info.recv_cb;
    uint8_t* mac_addr = recv_info->src_addr;
    uint8_t* des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t* data, uint16_t data_len, uint8_t* state, uint16_t* seq, uint32_t* magic, uint8_t* payload, uint16_t* payload_len) {
    example_espnow_data_t* buf = (example_espnow_data_t*)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf, data_len);

    *payload_len = data_len - sizeof(example_espnow_data_t);
    memcpy(payload, buf->payload, *payload_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t* send_param, uint8_t* payload, uint16_t payload_len) {
    example_espnow_data_t* buf = (example_espnow_data_t*)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;

    memset(buf->payload, 0, send_param->len - sizeof(example_espnow_data_t));

    if (NULL != payload) {
        memcpy(buf->payload, payload, payload_len);
        buf->payload[payload_len] = '\0';
    }

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const*)buf, send_param->len);
}

void dump_send_cb(example_espnow_send_param_t* send_param, example_espnow_event_send_cb_t* send_cb) {
    ESP_LOGI(TAG,
             "Send to      "
             "MAC: " MACSTR
             ", state = %d"
             ", status: %d",
             MAC2STR(send_cb->mac_addr), send_param->state, send_cb->status);
}

void dump_receive_cb(example_espnow_event_recv_cb_t* recv_cb, uint16_t recv_seq, uint8_t recv_state) {
    ESP_LOGI(TAG,
             "Receive from "
             "MAC: " MACSTR
             ", state = %d"
             ", len: %d"
             ", %dth broadcast data",
             MAC2STR(recv_cb->mac_addr), recv_state, recv_cb->data_len, recv_seq);
}

void send_next_broadcast(example_espnow_send_param_t* send_param, example_espnow_event_send_cb_t* send_cb) {
    /* Delay a while before sending the next broadcast data. */
    if (send_param->delay > 0) {
        vTaskDelay(send_param->delay / portTICK_PERIOD_MS);
    }

    memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param, NULL, 0);

    /* Send the next data after the previous data is sent. */
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
}
static void example_espnow_task(void* pvParameter) {
    example_espnow_event_t evt;
    uint8_t recv_state = EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED_NOT;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t* send_param = (example_espnow_send_param_t*)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    static uint8_t confirm_count = BS_CONFIRMING_COUNT;
    static uint8_t peer_mac_addr[ESP_NOW_ETH_ALEN];
    memcpy(peer_mac_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB: {
                example_espnow_event_send_cb_t* send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                if (is_broadcast) {
                    dump_send_cb(send_param, send_cb);
                    switch (send_param->broadcast) {
                        case (BS_BROADCASTING): {
                            send_next_broadcast(send_param, send_cb);
                            break;
                        }
                        case (BS_CONFIRMING): {
                            if (confirm_count-- == 0) {
                                ESP_LOGI(TAG, "Start sending unicast data");
                                ESP_LOGI(TAG, "Peer to " MACSTR "", MAC2STR(peer_mac_addr));

                                /* Start sending unicast ESPNOW data. */
                                send_param->broadcast = BS_UNICAST;
                                memcpy(send_param->dest_mac, peer_mac_addr, ESP_NOW_ETH_ALEN);
                                xSemaphoreGive(g_send_done_sem);
                            } else {
                                ESP_LOGI(TAG, "Sending confim data: %d", confirm_count);
                                send_next_broadcast(send_param, send_cb);
                            }
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                } else {
                    xSemaphoreGive(g_send_done_sem);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB: {
                example_espnow_event_recv_cb_t* recv_cb = &evt.info.recv_cb;

                uint8_t payload[CONFIG_ESPNOW_SEND_LEN];
                uint16_t payload_len;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic, payload, &payload_len);
                free(recv_cb->data);

                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    dump_receive_cb(recv_cb, recv_seq, recv_state);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK(esp_now_add_peer(peer));
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED_NOT) {
                        send_param->state = EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED;
                    }

                    if (recv_state == EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED) {
                        if (send_param->broadcast == BS_BROADCASTING) {
                            send_param->broadcast = BS_CONFIRMING;
                            memcpy(peer_mac_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        }
                    }

                    if (recv_state == EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED_NOT) {
                        if (send_param->broadcast != BS_BROADCASTING) {
                            ESP_LOGI(TAG, "Restart sending broadcast data");

                            confirm_count = BS_CONFIRMING_COUNT;
                            s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_BROADCAST] = 0;

                            /* Start sending broadcast ESPNOW data. */
                            send_param->broadcast = BS_BROADCASTING;
                            send_param->state = EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED;
                            memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param, NULL, 0);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                        }
                    }

                } else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    // ESP_LOGI(TAG, "Receive %dth unicast data from: " MACSTR ", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    // ESP_LOGI(TAG, "payload: %s", payload);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    if (send_param->broadcast != BS_UNICAST) {
                        ESP_LOGI(TAG, "Receive unicast data without BS_UNICAST");
                        ESP_LOGI(TAG, "Peer to " MACSTR "", MAC2STR(recv_cb->mac_addr));

                        /* Start sending unicast ESPNOW data. */
                        send_param->broadcast = BS_UNICAST;
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        xSemaphoreGive(g_send_done_sem);
                    }

                    memcpy(&g_motor_output_data, payload, sizeof(motor_output_t));
                    xTaskNotify(*ffb_task_handle, 0, eSetBits);

                } else {
                    ESP_LOGI(TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static void espnow_write_task(void* pvParameter) {
    example_espnow_send_param_t* send_param = (example_espnow_send_param_t*)pvParameter;
    for (;;) {
        while (send_param->broadcast != BS_UNICAST) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(g_send_done_sem, portMAX_DELAY);

        ffb_output_t ffb_output_data = {};
        float constant_force;
        float damper;
        ffb_output(&constant_force, &damper);
        ffb_output_data.constant_force = constant_force;
        ffb_output_data.damper = damper;

        // Write data to the ESPNOW
        example_espnow_data_prepare(send_param, (uint8_t*)&ffb_output_data, sizeof(ffb_output_t));
        /* Send the next data after the previous data is sent. */
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
            example_espnow_deinit(send_param);
            vTaskDelete(NULL);
        }
    }
}

static esp_err_t example_espnow_init(void) {
    example_espnow_send_param_t* send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(example_espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(example_espnow_recv_cb));
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->broadcast = BS_BROADCASTING;
    send_param->state = EXAMPLE_ESPNOW_DATA_BROADCAST_RECEIVED_NOT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param, NULL, 0);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2560, send_param, 10, NULL);
    xTaskCreate(espnow_write_task, "espnow_write_task", TASK_STACK_SIZE, send_param, 10, &espnow_write_task_handle);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t* send_param) {
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}

void espnow_backend_init(void) {
    g_send_done_sem = xSemaphoreCreateBinary();
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    example_wifi_init();
    example_espnow_init();
}
