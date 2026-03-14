/*
 * REST_API.h
 *
 *  Created on: 4 mar 2026
 *      Author: Karol Wickel
 */

#ifndef MAIN_REST_API_H_
#define MAIN_REST_API_H_

#include "esp_http_server.h" 
#include "config.h"
#include "communication.h"

class REST_API {
private:
    httpd_handle_t  _server  = nullptr;
    httpd_config_t  _config;
    GloveConfig    *_cfg     = nullptr; 
	data_packet_t *sensor_data = nullptr;

    static esp_err_t root_handler              (httpd_req_t *req);
    static esp_err_t finger_data_get_uri_handler(httpd_req_t *req);
    static esp_err_t config_uri_data_get       (httpd_req_t *req);
    static esp_err_t config_uri_data_post      (httpd_req_t *req);

public:
    explicit REST_API(GloveConfig &config, data_packet_t &data);  // ← takes config by reference
    ~REST_API();
    void start();
    void stop();
};


#endif /* MAIN_REST_API_H_ */