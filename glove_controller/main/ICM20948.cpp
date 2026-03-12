/*
 * ICM20948.cpp
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */
 
#include "ICM20948.h"
#include "esp_check.h"

ICM20948::ICM20948(i2c_master_dev_handle_t mux_dev,
                   uint8_t                 mux_channel,
                   uint8_t                 i2c_addr = 0x68)
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&(this->i2c_mst_config), &(this->_bus)));
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device((this->_bus), &dev_cfg, &(this->_dev)));
    _mux_channel = mux_channel;
}

ICM20948::~ICM20948(){

}

esp_err_t ICM20948::verify(){       // wake, verify WHO_AM_I, configure
    uint8_t buf[1];
    uint8_t res = this->read_register(ICM_BANK_0, 0x0, buf);
    if((res == ESP_OK) &&(buf[0] == 0xAE))
        return ESP_OK;
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t ICM20948::write_register(icm20948_bank_t bank, uint8_t reg, uint8_t value){
    this->_set_bank(bank);
    
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit((this->_dev), write_buf, sizeof(write_buf), -1);
}
esp_err_t ICM20948::read_register(icm20948_bank_t bank, uint8_t reg,
                              uint8_t* buf, size_t len = 1){

    this->_set_bank(bank);                            
    // Write the register address, then read back 1 byte
    esp_err_t ret = i2c_master_transmit_receive(
        this->_dev,         // i2c_master_dev_handle_t, obtained from i2c_master_bus_add_device()
        &reg, 1,       // tx: send the register address
        buf,  len,       // rx: receive 1 byte back
        100                 // timeout in ms
    );

    if (ret != ESP_OK) {
        printf("Wrong I2C read of the register %d, in the bank %d\n", reg, bank);
    }
    return ret;
}
uint8_t ICM20948::read_register (icm20948_bank_t bank, uint8_t reg, size_t len = 1){
    uint8_t buf;
    
    this->_set_bank(bank);                            
    // Write the register address, then read back 1 byte
    esp_err_t ret = i2c_master_transmit_receive(
        this->_dev,         // i2c_master_dev_handle_t, obtained from i2c_master_bus_add_device()
        &reg, 1,       // tx: send the register address
        &buf,  1,       // rx: receive 1 byte back
        100                 // timeout in ms
    );

    if (ret != ESP_OK) {
        printf("Wrong I2C read of the register %d, in the bank %d\n", reg, bank);
    }

    return buf; 
}

void ICM20948::read_accel(float* buf){
    int16_t raw[3];

    raw[0] = (this->read_register(ICM_BANK_0, ACCEL_XOUT_H)) << 8 | (this->read_register(ICM_BANK_0, ACCEL_XOUT_L) );
    raw[1] = (this->read_register(ICM_BANK_0, ACCEL_YOUT_H)) << 8 | (this->read_register(ICM_BANK_0, ACCEL_YOUT_L) );
    raw[2] = (this->read_register(ICM_BANK_0, ACCEL_ZOUT_H)) << 8 | (this->read_register(ICM_BANK_0, ACCEL_ZOUT_L) );
    
    for(int i = 0; i<3; i++){
        buf[i] = (float)raw[i]/ACCEL_RES;
    }
}

esp_err_t ICM20948::read_gyro (float* gx, float* gy, float* gz){

}

esp_err_t ICM20948::read_temp (float* temp_c){

}

esp_err_t ICM20948::_set_bank(icm20948_bank_t bank){   
    uint8_t buf = 0;

    uint8_t write_buf[2] = {SELECT_BANK, bank};
    esp_err_t ret = i2c_master_transmit((this->_dev), write_buf, sizeof(write_buf), -1);
    return ret;
}

esp_err_t ICM20948::reset()
{
    esp_err_t ret = write_register(ICM_BANK_0, PWR_MGMT_1, 0x80);
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
}

esp_err_t ICM20948::sleep(bool enable)
{
    uint8_t val;

    read_register(ICM_BANK_0, PWR_MGMT_1, &val);

    if(enable)
        val |= 0x40;
    else
        val &= ~0x40;

    return write_register(ICM_BANK_0, PWR_MGMT_1, val);
}

esp_err_t ICM20948::begin()
{
    ESP_RETURN_ON_ERROR(reset(), TAG, "Reset failed");

    uint8_t who;
    ESP_RETURN_ON_ERROR(read_register(ICM_BANK_0, WHO_AM_I, &who), TAG, "WHO_AM_I read failed");

    if (who != 0xEA)
        return ESP_ERR_NOT_FOUND;

    ESP_RETURN_ON_ERROR(sleep(false), TAG, "Wake failed");

    set_accel_range(ACCEL_RANGE_2G);
    set_gyro_range(GYRO_RANGE_250DPS);
    enable_mag();   

    return ESP_OK;
}

esp_err_t ICM20948::set_accel_range(accel_range_t range)
{
    uint8_t reg;

    ESP_RETURN_ON_ERROR(read_register(ICM_BANK_2, ACCEL_CONFIG, &reg), TAG, "Read fail");

    reg &= ~0x06;
    reg |= (range << 1);

    ESP_RETURN_ON_ERROR(write_register(ICM_BANK_2, ACCEL_CONFIG, reg), TAG, "Write fail");

    switch(range)
    {
        case ACCEL_RANGE_2G:  _accel_res = 16384.0f; break;
        case ACCEL_RANGE_4G:  _accel_res = 8192.0f; break;
        case ACCEL_RANGE_8G:  _accel_res = 4096.0f; break;
        case ACCEL_RANGE_16G: _accel_res = 2048.0f; break;
    }

    return ESP_OK;
}

esp_err_t ICM20948::set_gyro_range(gyro_range_t range)
{
    uint8_t reg;

    ESP_RETURN_ON_ERROR(read_register(ICM_BANK_2, GYRO_CONFIG_1, &reg), TAG, "Read fail");

    reg &= ~0x06;
    reg |= (range << 1);

    ESP_RETURN_ON_ERROR(write_register(ICM_BANK_2, GYRO_CONFIG_1, reg), TAG, "Write fail");

    switch(range)
    {
        case GYRO_RANGE_250DPS:  _gyro_res = 131.0f; break;
        case GYRO_RANGE_500DPS:  _gyro_res = 65.5f; break;
        case GYRO_RANGE_1000DPS: _gyro_res = 32.8f; break;
        case GYRO_RANGE_2000DPS: _gyro_res = 16.4f; break;
    }

    return ESP_OK;
}

esp_err_t ICM20948::read_temp(float* temp_c)
{
    uint8_t buf[2];

    ESP_RETURN_ON_ERROR(read_register(ICM_BANK_0, TEMP_OUT_H, buf, 2), TAG, "Temp read fail");

    int16_t raw = (buf[0] << 8) | buf[1];

    *temp_c = ((float)raw / 333.87f) + 21.0f;

    return ESP_OK;
}

esp_err_t ICM20948::read_gyro(float* gx, float* gy, float* gz)
{
    uint8_t buf[6];

    ESP_RETURN_ON_ERROR(
        read_register(ICM_BANK_0, GYRO_XOUT_H, buf, 6),
        TAG,
        "gyro read failed"
    );

    int16_t raw[3];

    raw[0] = (buf[0] << 8) | buf[1];
    raw[1] = (buf[2] << 8) | buf[3];
    raw[2] = (buf[4] << 8) | buf[5];

    *gx = ((float)raw[0] / _gyro_res) - _gyro_bias[0];
    *gy = ((float)raw[1] / _gyro_res) - _gyro_bias[1];
    *gz = ((float)raw[2] / _gyro_res) - _gyro_bias[2];

    return ESP_OK;
}

esp_err_t ICM20948::calibrate_gyro(uint16_t samples)
{
    float gx, gy, gz;

    float sum[3] = {0};

    for(int i = 0; i < samples; i++)
    {
        read_gyro(&gx, &gy, &gz);

        sum[0] += gx;
        sum[1] += gy;
        sum[2] += gz;

    }

    _gyro_bias[0] = sum[0] / samples;
    _gyro_bias[1] = sum[1] / samples;
    _gyro_bias[2] = sum[2] / samples;

    return ESP_OK;
}

esp_err_t ICM20948::enable_data_ready_interrupt(bool enable)
{
    uint8_t val = enable ? 0x01 : 0x00;

    return write_register(ICM_BANK_0, 0x10, val);
}

bool ICM20948::data_ready()
{
    uint8_t status;

    read_register(ICM_BANK_0, INT_STATUS, &status);

    return status & 0x01;
}

esp_err_t ICM20948::read_all(icm20948_data_t* out)
{
    uint8_t buf[14];

    ESP_RETURN_ON_ERROR(
        read_register(ICM_BANK_0, ACCEL_XOUT_H, buf, 14),
        TAG,
        "burst read failed"
    );

    ESP_RETURN_ON_ERROR(
        read_mag(out->mx, out->my, out->mz),
        TAG,
        "magnetometer read failed"
    );

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];

    int16_t gx = (buf[6] << 8) | buf[7];
    int16_t gy = (buf[8] << 8) | buf[9];
    int16_t gz = (buf[10] << 8) | buf[11];

    int16_t t  = (buf[12] << 8) | buf[13];

    out->ax = (float)ax / _accel_res - _accel_bias[0];
    out->ay = (float)ay / _accel_res - _accel_bias[1];
    out->az = (float)az / _accel_res - _accel_bias[2];

    out->gx = (float)gx / _gyro_res - _gyro_bias[0];
    out->gy = (float)gy / _gyro_res - _gyro_bias[1];
    out->gz = (float)gz / _gyro_res - _gyro_bias[2];

    out->temp_c = ((float)t / 333.87f) + 21.0f;

    return ESP_OK;
}

bool ICM20948::is_connected()
{
    uint8_t id;

    if(read_register(ICM_BANK_0, WHO_AM_I, &id) != ESP_OK)
        return false;

    return id == 0xEA;
}

esp_err_t ICM20948::enable_mag()
{
    esp_err_t ret;

    // enable I2C master mode
    ret = write_register(ICM_BANK_0, 0x03, 0x20);
    if (ret != ESP_OK) return ret;

    // set I2C master clock
    ret = write_register(ICM_BANK_3, I2C_MST_CTRL, 0x07);
    if (ret != ESP_OK) return ret;

    // power up magnetometer
    ret = write_register(ICM_BANK_3, I2C_SLV0_ADDR, AK09916_ADDR);
    if (ret != ESP_OK) return ret;

    ret = write_register(ICM_BANK_3, I2C_SLV0_REG, AK09916_CNTL2);
    if (ret != ESP_OK) return ret;

    ret = write_register(ICM_BANK_3, I2C_SLV0_DO, 0x08); // continuous 100Hz
    if (ret != ESP_OK) return ret;

    ret = write_register(ICM_BANK_3, I2C_SLV0_CTRL, 0x81);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

esp_err_t ICM20948::read_mag(float* mx, float* my, float* mz)
{
    uint8_t buf[6];

    esp_err_t ret = read_register(ICM_BANK_0,
                                  EXT_SENS_DATA_00,
                                  buf,
                                  6);

    if (ret != ESP_OK)
        return ret;

    int16_t raw_x = (buf[1] << 8) | buf[0];
    int16_t raw_y = (buf[3] << 8) | buf[2];
    int16_t raw_z = (buf[5] << 8) | buf[4];

    const float MAG_RES = 0.15f; // µT/LSB

    *mx = raw_x * MAG_RES;
    *my = raw_y * MAG_RES;
    *mz = raw_z * MAG_RES;

    return ESP_OK;
}