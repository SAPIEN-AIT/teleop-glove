/*
 * wifi.h
 *
 *  Created on: 14 mar 2026
 *      Author: Karol Wickel
 */

#pragma once

#ifndef CONFIG_H_
#define CONFIG_H_
#include <cstdint>

struct GloveConfig {
    uint32_t rate_hz      = 100;
    float    filter_alpha = 0.9f;
    char     udp_ip[32]   = "192.168.1.100";
    uint16_t udp_port     = 5005;
};

#endif