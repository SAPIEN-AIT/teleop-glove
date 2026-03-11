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

    esp_err_t _mux_select();
    esp_err_t _mux_deselect();
    esp_err_t _set_bank(icm20948_bank_t bank);

public:
    ICM20948(i2c_master_bus_handle_t bus_handle,
             i2c_master_dev_handle_t mux_dev,
             uint8_t                 mux_channel,
             uint8_t                 i2c_addr = 0x68);

    ~ICM20948();

    esp_err_t begin();       // wake, verify WHO_AM_I, configure

    esp_err_t write_register(icm20948_bank_t bank, uint8_t reg, uint8_t value);
    esp_err_t read_register (icm20948_bank_t bank, uint8_t reg,
                              uint8_t* buf, size_t len = 1);

    esp_err_t read_accel(float* ax, float* ay, float* az);
    esp_err_t read_gyro (float* gx, float* gy, float* gz);
    esp_err_t read_temp (float* temp_c);     
};

#endif /* MAIN_ICM20948_H_ */
