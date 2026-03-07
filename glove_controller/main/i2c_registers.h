/*
 * i2c_registers.h
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */

#ifndef MAIN_I2C_REGISTERS_H_
#define MAIN_I2C_REGISTERS_H_


#define ICM20948_WHO_AM_I       0xEA

//USER BANK 1
const static int SETUP = 0x3;
const static int PWR_MGT_2 = 0x6;
const static int ACCEL_XOUT_H = 0x2d;
const static int ACCEL_XOUT_L = 0x2e;
const static int ACCEL_YOUT_H = 0x2f;
const static int ACCEL_YOUT_L = 0x30;
const static int ACCEL_ZOUT_H = 0x31;
const static int ACCEL_ZOUT_L = 0x32;
const static int GYRO_XOUT_H = 0x33;
const static int GYRO_XOUT_L = 0x34;
const static int GYRO_YOUT_H = 0x35;
const static int GYRO_YOUT_L = 0x36;
const static int GYRO_ZOUT_H = 0x37;
const static int GYRO_ZOUT_L = 0x38;

const static int SELECT_BANK = 0x7f;

//USER BANK 2
const static int ACCEL_CONFIG = 0x14;





#endif /* MAIN_I2C_REGISTERS_H_ */
