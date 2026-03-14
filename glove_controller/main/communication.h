/*
 * communicaton.h
 *
 *  Created on: 14 mar 2026
 *      Author: Karol Wickel
 */

#pragma once

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "Quaternion.h"
#include "esp_types.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>

typedef struct data_packet_t{
    Quaternion joints[16];
    int64_t epoch_time;
}__attribute__((packed));



class  Telemetry
{
private:
    GloveConfig &_config;
    data_packet_t data = {};
    SemaphoreHandle_t _mutex;
    int _socket = -1;

public:
    explicit Telemetry(GloveConfig &config);
    void start();
    void updatePacket(const data_packet_t &pkt);
    void task(void* arg);
    void run();
};




#endif