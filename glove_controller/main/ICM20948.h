/*
 * ICM20948.h
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */
#pragma once

#ifndef MAIN_ICM20948_H_
#define MAIN_ICM20948_H_

#include "driver/i2c_master.h" 
#include "driver/i2c_types.h"
#include "hal/i2c_types.h"
#include "soc/clk_tree_defs.h"
#include <cstdint>
#include "i2c_registers.h"

static const i2c_port_num_t I2C_PORT = 0;
static const gpio_num_t SCL_PIN = static_cast<gpio_num_t>(7);
static const gpio_num_t SDA_PIN = static_cast<gpio_num_t>(8);

typedef enum {
    ICM_BANK_0 = 0,
    ICM_BANK_1 = 1,
    ICM_BANK_2 = 2,
    ICM_BANK_3 = 3,
} icm20948_bank_t;

typedef enum {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
} accel_range_t;

typedef enum {
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
} gyro_range_t;

struct icm20948_data_t {
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
};


class ICM20948 {
private:
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68,
    .scl_speed_hz = 100000,
    };

    i2c_master_bus_handle_t _bus;
    i2c_master_dev_handle_t _dev;
    uint8_t                 _i2c_addr;      // 0x68 or 0x69

    uint8_t                 _mux_channel;   

    icm20948_bank_t         _current_bank;

    static const char*      TAG;

    float _accel_res = 16384.0f;
    float _gyro_res  = 131.0f;

    float _gyro_bias[3]  = {0};
    float _accel_bias[3] = {0};

    uint32_t _timeout = 100;

    esp_err_t _set_bank(icm20948_bank_t bank);

public:
    ICM20948(i2c_master_dev_handle_t mux_dev,
             uint8_t                 mux_channel,
             uint8_t                 i2c_addr = 0x68);

    ~ICM20948();

    esp_err_t verify();       // wake, verify WHO_AM_I, configure

    esp_err_t write_register(icm20948_bank_t bank, uint8_t reg, uint8_t value);
    esp_err_t read_register (icm20948_bank_t bank, uint8_t reg,
                              uint8_t* buf, size_t len = 1);

    uint8_t read_register (icm20948_bank_t bank, uint8_t reg,
                            size_t len = 1);
                              
    void read_accel(float* buf);
    esp_err_t read_gyro (float* gx, float* gy, float* gz);
    esp_err_t read_temp (float* temp_c); 
    
    esp_err_t reset();
    esp_err_t sleep(bool enable);
    esp_err_t begin();
    esp_err_t set_accel_range(accel_range_t range);
    esp_err_t set_gyro_range(gyro_range_t range);
    esp_err_t calibrate_gyro(uint16_t samples);
    esp_err_t enable_data_ready_interrupt(bool enable);
    bool data_ready();
    esp_err_t read_all(icm20948_data_t* out);
    bool is_connected();
};

#endif /* MAIN_ICM20948_H_ */
