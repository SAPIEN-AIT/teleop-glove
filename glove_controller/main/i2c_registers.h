/*
 * i2c_registers.h
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */

    #ifndef MAIN_I2C_REGISTERS_H_
#define MAIN_I2C_REGISTERS_H_


#define ICM20948_WHO_AM_I       0xEA
#define ACCEL_RES               100.0f


// USER BANK 0
static const uint8_t WHO_AM_I      = 0x00;
const static uint8_t SETUP = 0x3;
static const uint8_t PWR_MGMT_1    = 0x06;
static const uint8_t INT_STATUS    = 0x19;
static const uint8_t TEMP_OUT_H    = 0x39;
static const uint8_t TEMP_OUT_L    = 0x3A;
static const uint8_t FIFO_EN_1     = 0x66;
static const uint8_t FIFO_EN_2     = 0x67;
static const uint8_t FIFO_RST      = 0x68;
static const uint8_t FIFO_COUNT_H  = 0x70;
static const uint8_t FIFO_R_W      = 0x72;
const static uint8_t PWR_MGT_2 = 0x6;
const static uint8_t ACCEL_XOUT_H = 0x2d;
const static uint8_t ACCEL_XOUT_L = 0x2e;
const static uint8_t ACCEL_YOUT_H = 0x2f;
const static uint8_t ACCEL_YOUT_L = 0x30;
const static uint8_t ACCEL_ZOUT_H = 0x31;
const static uint8_t ACCEL_ZOUT_L = 0x32;
const static uint8_t GYRO_XOUT_H = 0x33;
const static uint8_t GYRO_XOUT_L = 0x34;
const static uint8_t GYRO_YOUT_H = 0x35;
const static uint8_t GYRO_YOUT_L = 0x36;
const static uint8_t GYRO_ZOUT_H = 0x37;
const static uint8_t GYRO_ZOUT_L = 0x38;

// USER BANK 1
static const uint8_t GYRO_CONFIG_1  = 0x01;
static const uint8_t ACCEL_CONFIG   = 0x14;
static const uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
static const uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
static const uint8_t GYRO_SMPLRT_DIV    = 0x00;

const static uint8_t SELECT_BANK = 0x7f;

//USER BANK 2
static const uint8_t GYRO_CONFIG_1  = 0x01;
static const uint8_t ACCEL_CONFIG   = 0x14;
static const uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
static const uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
static const uint8_t GYRO_SMPLRT_DIV    = 0x00;

// USER BANK 0
static const uint8_t EXT_SENS_DATA_00 = 0x3B;

// USER BANK 3 (I2C master)
static const uint8_t I2C_MST_CTRL = 0x01;
static const uint8_t I2C_SLV0_ADDR = 0x03;
static const uint8_t I2C_SLV0_REG  = 0x04;
static const uint8_t I2C_SLV0_CTRL = 0x05;
static const uint8_t I2C_SLV0_DO   = 0x06;

// Magnetometer
static const uint8_t AK09916_ADDR  = 0x0C;
static const uint8_t AK09916_ST1   = 0x10;
static const uint8_t AK09916_DATA  = 0x11;
static const uint8_t AK09916_CNTL2 = 0x31;



#endif /* MAIN_I2C_REGISTERS_H_ */
