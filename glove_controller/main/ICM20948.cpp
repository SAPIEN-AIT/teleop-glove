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
    ESP_ERROR_CHECK(i2c_new_master_bus(&(this->i2c_mst_config), &(this->_bus)));
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device((this->_bus), &dev_cfg, &(this->_dev)));

}

ICM20948::~ICM20948(){

}

esp_err_t ICM20948::verify(){       // wake, verify WHO_AM_I, configure
    uint8_t buf[1];
    uint8_t res = this->read_register(ICM_BANK_0, 0x0, buf);
    if((res == ESP_OK) &&(buf[0] == 0xAE))
        return ESP_OK;
}

esp_err_t write_register(icm20948_bank_t bank, uint8_t reg, uint8_t value){

}
esp_err_t ICM20948::read_register(icm20948_bank_t bank, uint8_t reg,
                              uint8_t* buf, size_t len = 1){

    this->_set_bank(bank);                            
    // Write the register address, then read back 1 byte
    esp_err_t ret = i2c_master_transmit_receive(
        this->_dev,         // i2c_master_dev_handle_t, obtained from i2c_master_bus_add_device()
        &reg, 1,       // tx: send the register address
        buf,  1,       // rx: receive 1 byte back
        100                 // timeout in ms
    );

    if (ret != ESP_OK) {
        printf("Wrong I2C read of the register %d, in the bank %d\n", reg, bank);
    }
}

esp_err_t ICM20948::read_accel(float* ax, float* ay, float* az){

}

esp_err_t ICM20948::read_gyro (float* gx, float* gy, float* gz){

}

esp_err_t ICM20948::read_temp (float* temp_c){

}

esp_err_t ICM20948::_set_bank(icm20948_bank_t bank){   
    uint8_t buf = 0;

    uint8_t write_buf[2] = {SELECT_BANK, bank};
    i2c_master_transmit((this->_dev), write_buf, sizeof(write_buf), -1);

}