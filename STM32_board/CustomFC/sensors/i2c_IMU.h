#ifndef I2C_IMU_H
#define I2C_IMU_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;


typedef struct{

    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temp;


} i2cIMUData;

uint8_t i2cImuInit();
uint8_t i2cImuRead(i2cIMUData *IMUData);

#endif //I2C_IMU_H