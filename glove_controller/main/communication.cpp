/*
 * communicaton.h
 *
 *  Created on: 14 mar 2026
 *      Author: Karol Wickel
 */

#include "communication.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "Telemetry";

static GloveConfig *s_config = nullptr;

explicit Telemetry::Telemetry(GloveConfig &config):_config(config), _mutex(nullptr){};

Telemetry::~ Telemetry()
{
}

void Telemetry::task(void *arg)
{
    static_cast<Telemetry *>(arg)->run();
}

void Telemetry::updatePacket(const data_packet_t &pkt){
    xSemaphoreTake(_mutex, portMAX_DELAY);
    data = pkt;
    xSemaphoreGive(_mutex);
}

void Telemetry::run(){
    _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in dest = { .sin_family = AF_INET };
    data_packet_t pkt_copy;

    int tos = 0xB8;
    setsockopt(_socket, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    struct timeval timeout = { .tv_sec = 0, .tv_usec = 500000 }; // 500ms
    setsockopt(_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    while (true) {
        dest.sin_port = htons(_config.udp_port);
        inet_pton(AF_INET, _config.udp_ip, &dest.sin_addr);

        xSemaphoreTake(_mutex, portMAX_DELAY);
        pkt_copy = data;
        xSemaphoreGive(_mutex);

        sendto(_socket, &pkt_copy, sizeof(pkt_copy), 0,
               reinterpret_cast<sockaddr *>(&dest), sizeof(dest));

        vTaskDelay(pdMS_TO_TICKS(1000 / _config.rate_hz));
    }

}

void Telemetry::start()
{
    _mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(task, "telemetry", 4096, this, 5, nullptr, 1);
    ESP_LOGI(TAG, "Telemetry started → %s:%d", _config.udp_ip, _config.udp_port);
}