/*
 * REST_API.cpp
 *
 *  Created on: 4 mar 2026
 *      Author: Karol Wickel
 */
#include "REST_API.h"
#include <string.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "config.h"
#include <algorithm>

static const char *TAG = "REST_API";

extern const char index_html_start[] asm("_binary_www_index_html_start");
extern const char index_html_end[]   asm("_binary_www_index_html_end");

REST_API::REST_API(GloveConfig &config, data_packet_t &data) 
			: _server(nullptr), _cfg(&config), sensor_data(&data) {
    _config = HTTPD_DEFAULT_CONFIG();
}

REST_API::~REST_API() {
    stop(); 
}

void REST_API::stop() {
    if (_server) {
        httpd_stop(_server);
        _server = nullptr;
    }
}

esp_err_t REST_API::root_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start,
		index_html_end - index_html_start);
		return ESP_OK;
}
	
esp_err_t REST_API::finger_data_get_uri_handler(httpd_req_t *req)
{
    REST_API    *self = static_cast<REST_API *>(req->user_ctx);
    data_packet_t *pkt = self->sensor_data;

    cJSON *root = cJSON_CreateObject();

    // ── helper lambda to build one finger object ──────────────────
    auto addFinger = [&](const char *name, int idx) {
        cJSON *finger = cJSON_CreateObject();

        // quaternion
        cJSON *quat = cJSON_CreateObject();
        cJSON_AddNumberToObject(quat, "w", pkt->joints[idx].getQ1());
        cJSON_AddNumberToObject(quat, "x", pkt->joints[idx].getQ2());
        cJSON_AddNumberToObject(quat, "y", pkt->joints[idx].getQ3());
        cJSON_AddNumberToObject(quat, "z", pkt->joints[idx].getQ4());
        cJSON_AddItemToObject(finger, "quaternion", quat);

        cJSON_AddItemToObject(root, name, finger);
    };

    addFinger("finger1", 0);
    addFinger("finger2", 1);
    addFinger("finger3", 2);
    addFinger("finger4", 3);
    addFinger("thumb",   4);

    cJSON_AddNumberToObject(root, "timestamp_ms", pkt->epoch_time);

    char *json = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}
// ── GET /api/v1/config ───────────────────────────────────────────
esp_err_t REST_API::config_uri_data_get(httpd_req_t *req)
{
	REST_API *self = static_cast<REST_API *>(req->user_ctx);
    GloveConfig *cfg = self->_cfg;  // access config via the instance
	
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "rate_hz",      cfg->rate_hz);
    cJSON_AddNumberToObject(root, "filter_alpha", cfg->filter_alpha);
    cJSON_AddStringToObject(root, "udp_ip",       cfg->udp_ip);
    cJSON_AddNumberToObject(root, "udp_port",     cfg->udp_port);
	
    char *json = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    free(json);
    cJSON_Delete(root);
    return ESP_OK;
}

// ── POST /api/v1/config ──────────────────────────────────────────
esp_err_t REST_API::config_uri_data_post(httpd_req_t *req)
{
	REST_API *self = static_cast<REST_API *>(req->user_ctx);
    GloveConfig *cfg = self->_cfg;
	
    char buf[256] = {0};
    int ret = httpd_req_recv(req, buf,
                             std::min((size_t)req->content_len,
							 (size_t)(sizeof(buf) - 1)));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
	
    cJSON *rate  = cJSON_GetObjectItem(root, "rate_hz");
    cJSON *alpha = cJSON_GetObjectItem(root, "filter_alpha");
    cJSON *ip    = cJSON_GetObjectItem(root, "udp_ip");
    cJSON *port  = cJSON_GetObjectItem(root, "udp_port");
	
    if (cJSON_IsNumber(rate))  cfg->rate_hz      = rate->valueint;
    if (cJSON_IsNumber(alpha)) cfg->filter_alpha  = (float)alpha->valuedouble;
    if (cJSON_IsString(ip))    strncpy(cfg->udp_ip, ip->valuestring, sizeof(cfg->udp_ip) - 1);
    if (cJSON_IsNumber(port))  cfg->udp_port      = (uint16_t)port->valueint;
	
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// ── start() ──────────────────────────────────────────────────────
void REST_API::start()  // BUG WAS: static void start(void* arg) — wrong signature
{
	if (httpd_start(&_server, &_config) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to start server");
        return;
    }
	
    // BUG WAS: all three httpd_register_uri_handler calls used &finger_uri
    // BUG WAS: config_uri declared twice (duplicate variable name)
    // BUG WAS: POST handler used HTTP_GET method
    // BUG WAS: user_ctx = self but handlers cast to GloveConfig* — now consistent
	
    httpd_uri_t root_uri = {
		.uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
        .user_ctx = this
    };
    httpd_register_uri_handler(_server, &root_uri);
	
    httpd_uri_t finger_uri = {
		.uri      = "/api/v1/finger/all",
        .method   = HTTP_GET,
        .handler  = finger_data_get_uri_handler,
        .user_ctx = this
    };
    httpd_register_uri_handler(_server, &finger_uri);
	
    httpd_uri_t config_get_uri = {     // BUG WAS: named config_uri (duplicate)
        .uri      = "/api/v1/config",
        .method   = HTTP_GET,
        .handler  = config_uri_data_get,
        .user_ctx = this
    };
    httpd_register_uri_handler(_server, &config_get_uri);
	
    httpd_uri_t config_post_uri = {    // BUG WAS: named config_uri (duplicate)
        .uri      = "/api/v1/config",
        .method   = HTTP_POST,         // BUG WAS: HTTP_GET
        .handler  = config_uri_data_post,
        .user_ctx = this
    };
    httpd_register_uri_handler(_server, &config_post_uri);
	
    ESP_LOGI(TAG, "REST API started on port %d", _config.server_port);
}

