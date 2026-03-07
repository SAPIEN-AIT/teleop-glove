/*
 * ICM20948.cpp
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */
 
 #include "ICM20948.h"
 
ICM20948::ICM20948(i2c_master_bus_handle_t bus_handle,
                   uint8_t                 mux_channel,
                   uint8_t                 i2c_addr)
    : _bus(bus_handle),
      _i2c_addr(i2c_addr),
      _mux_channel(mux_channel & 0x07),
      _current_bank(ICM_BANK_0)
{
    i2c_master_bus_config_t i2c_mst_config_1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config_1, &bus_handle));
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = _i2c_addr,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(_bus, &dev_cfg, &_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device (addr=0x%02X, mux_ch=%d): %s",
                 _i2c_addr, _mux_channel, esp_err_to_name(err));
    }
}

ICM20948::~ICM20948() {
    i2c_master_bus_rm_device(_dev);
}

esp_err_t begin(){
    unit8_t buf = 0;

    

}


