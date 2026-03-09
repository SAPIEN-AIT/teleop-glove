/*
 * ICM20948.cpp
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */
 
 #include "ICM20948.h"
 
ICM20948::ICM20948(i2c_master_bus_handle_t bus_handle,
             i2c_master_dev_handle_t mux_dev,
             uint8_t                 mux_channel,
             uint8_t                 i2c_addr = 0x68)
{

}

ICM20948::~ICM20948(){

}

esp_err_t begin(){       // wake, verify WHO_AM_I, configure

}

esp_err_t write_register(icm20948_bank_t bank, uint8_t reg, uint8_t value){

}
esp_err_t read_register (icm20948_bank_t bank, uint8_t reg,
                              uint8_t* buf, size_t len = 1){

}

esp_err_t read_accel(float* ax, float* ay, float* az){

}

esp_err_t read_gyro (float* gx, float* gy, float* gz){

}

esp_err_t read_temp (float* temp_c){

}