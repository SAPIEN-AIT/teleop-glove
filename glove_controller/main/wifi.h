/*
 * wifi.h
 *
 *  Created on: 14 mar 2026
 *      Author: Karol Wickel
 */
#pragma once

#ifndef WIFI_H_
#define WIFI_H_

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "config.h"

class WiFiManager {
public:
    WiFiManager(const char *ssid, const char *password);
    void connect();
    void waitConnected();

private:
    const char *m_ssid;
    const char *m_password;
    EventGroupHandle_t m_event_group;

    static void eventHandler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data);
    int m_retry_count = 0;
};


#endif
