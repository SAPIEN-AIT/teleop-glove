/*
 * wifi.h
 *
 *  Created on: 14 mar 2026
 *      Author: Karol Wickel
 */

#include "wifi.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <cstring>

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRIES        5

static const char *TAG = "WiFiManager";

WiFiManager::WiFiManager(const char *ssid, const char *password)
    : m_ssid(ssid), m_password(password), m_event_group(nullptr) {}

void WiFiManager::eventHandler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    WiFiManager *self = static_cast<WiFiManager *>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (self->m_retry_count < MAX_RETRIES) {
            esp_wifi_connect();
            self->m_retry_count++;
            ESP_LOGI(TAG, "Retrying... (%d/%d)", self->m_retry_count, MAX_RETRIES);
        } else {
            xEventGroupSetBits(self->m_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        self->m_retry_count = 0;
        xEventGroupSetBits(self->m_event_group, WIFI_CONNECTED_BIT);
    }
}

void WiFiManager::connect()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    m_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &eventHandler, this, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &eventHandler, this, NULL);

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid,     m_ssid,     sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, m_password, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to %s...", m_ssid);
}

void WiFiManager::waitConnected()
{
    EventBits_t bits = xEventGroupWaitBits(m_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected.");
    } else {
        ESP_LOGE(TAG, "Connection failed.");
    }
}