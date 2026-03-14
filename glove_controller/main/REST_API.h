/*
 * MQTT.h
 *
 *  Created on: 4 mar 2026
 *      Author: edoardo
 */

#ifndef MAIN_REST_API_H_
#define MAIN_REST_API_H_

#include "esp_http_server.h"
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"


#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
static const char *TAG = "HTTP_CLIENT";

class REST_API {
private:
	
public:
	REST_API();
	virtual ~REST_API();
	
	
};

#endif /* MAIN_REST_API_H_ */