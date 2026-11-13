#include "i2c_IMU.h"
#include "stm32f4xx_hal_i2c.h"

#define imuAddress 0x68
#define whoAmIAddress 0x75
#define mgmtAddress 0x6B

#define accelRegister 0x1C
#define accelRange2g 0x00 //16384.0 LSB/g
#define accelRange4g 0x08 //8192.0 LSB/g
#define accelRange8g 0x10 //4096.0 LSB/g
#define accelRange16g 0x18 //2048.0 LSB/g

#define gyroRegister 0x1B
#define gyroRange250 0x00 //131.0 LSB/dg/s
#define gyroRange500 0x08 //65.5 LSB/dg/s
#define gyroRange1000 0x10 //32.8 LSB/dg/s
#define gyroRange2000 0x18 //16.4 LSB/dg/s


uint8_t i2cImuInit(){

    uint8_t val;
    uint8_t wake = 0;

    if(HAL_I2C_IsDeviceReady(&hi2c1, imuAddress<<1, 3, 100) != HAL_OK){
        return 0;
    }

    if(HAL_I2C_Mem_Read(&hi2c1, imuAddress<<1, whoAmIAddress, I2C_MEMADD_SIZE_8BIT, &val, 1, 100 ) != HAL_OK || val != 0x68){
        return 0;
    }

    if (HAL_I2C_Mem_Write(&hi2c1, imuAddress<<1, mgmtAddress, I2C_MEMADD_SIZE_8BIT, &wake, 1, 100)!=HAL_OK){
        return 0;
    }


    uint8_t accelConfig = accelRange16g;
    uint8_t gyroConfig = gyroRange2000;
    HAL_I2C_Mem_Write(&hi2c1, imuAddress<<1, accelRegister, I2C_MEMADD_SIZE_8BIT, &accelConfig, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, imuAddress<<1, gyroRegister, I2C_MEMADD_SIZE_8BIT, &gyroConfig,1 ,100);


    return 1;

}


uint8_t i2cImuRead(i2cIMUData *imuData){
    uint8_t rawData[14];
    float accelSF = 2048.0;
    float gyroSF = 16.4;

    //Read the raw data from the first byte: ACCEL_XOUTH = 0x3B
    if(HAL_I2C_Mem_Read(&hi2c1, imuAddress<<1, 0x3B, I2C_MEMADD_SIZE_8BIT, rawData, 14, 100)!= HAL_OK){
        return 0;
    };

    int16_t rawAccelX = (int16_t)(rawData[0]<<8|rawData[1]);
    int16_t rawAccelY = (int16_t)(rawData[2]<<8|rawData[3]);
    int16_t rawAccelZ = (int16_t)(rawData[4]<<8|rawData[5]);

    int16_t rawTemp = (int16_t)(rawData[6]<<8|rawData[7]);
    
    int16_t rawGyroX = (int16_t)(rawData[8]<<8|rawData[9]);
    int16_t rawGyroY = (int16_t)(rawData[10]<<8|rawData[11]);
    int16_t rawGyroZ = (int16_t)(rawData[12]<<8|rawData[13]);

    imuData->accelX = (float)rawAccelX/accelSF;
    imuData->accelY = (float)rawAccelY/accelSF;
    imuData->accelZ = (float)rawAccelZ/accelSF;
    imuData->temp   = ((float)rawTemp / 340.0f) + 36.53f;
    imuData->gyroX = (float)rawGyroX/gyroSF;
    imuData->gyroY = (float)rawGyroY/gyroSF;
    imuData->gyroZ = (float)rawGyroZ/gyroSF;

    return 1;
}

