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


#define MUX_PIN_A0   GPIO_NUM_4
#define MUX_PIN_A1   GPIO_NUM_5
#define MUX_PIN_A2   GPIO_NUM_6


typedef enum {
    ICM_BANK_0 = 0,
    ICM_BANK_1 = 1,
    ICM_BANK_2 = 2,
    ICM_BANK_3 = 3,
} icm20948_bank_t;


class ICM20948 {
private:
    i2c_master_bus_handle_t _bus;
    i2c_master_dev_handle_t _dev;
    uint8_t                 _mux_channel;    
    uint8_t                 _i2c_addr; 

    static const char*      TAG;

    void _mux_select();   // drives A0/A1/A2 to select channel

public:
    ICM20948(i2c_master_bus_handle_t bus_handle,
             uint8_t                 mux_channel,
             uint8_t                 i2c_addr = 0x68);
    ~ICM20948();

    esp_err_t begin();
    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t read_register (uint8_t reg, uint8_t* buf, size_t len = 1);

    esp_err_t read_accel(float* ax, float* ay, float* az);
    esp_err_t read_gyro (float* gx, float* gy, float* gz);
};
#endif /* MAIN_ICM20948_H_ */
