#include "i2c_IMU.h"
#include "fcConfig.h"
#include <stdbool.h>
#include <stddef.h>

#define imuAddress MPU6050_ADDRESS
#define whoAmIAddress 0x75
#define mgmtAddress 0x6B
#define sampleRateDivRegister 0x19
#define configRegister 0x1A
#define dataStartRegister 0x3B  //ACCEL_XOUT_H, start of the 14 data bytes

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

#define clockPllGyroX 0x01      //gyro PLL clock, more stable than the internal one
#define dlpf98Hz 0x02           //on-chip low pass: ~98 Hz, ~3 ms delay
#define sampleRateDiv1kHz 0x00  //1 kHz output

#define accelSF 2048.0f //LSB/g for +-16 g
#define gyroSF 16.4f    //LSB/(deg/s) for +-2000 deg/s

#define gravity 9.81f
#define degToRad 0.017453292519943295f

#define calibrationMaxSpread 0.035f //[rad/s], about 2 deg/s - more than this and it moved

static float gyroBias[3] = {0.0f, 0.0f, 0.0f}; //[rad/s], sensor axes

static bool correctionOn = false;
static imuCorrection correction;
static quaternion trimQuat = {1.0f, 0.0f, 0.0f, 0.0f};   //board -> frame


//Raw sensor-axis accel [g], gyro [rad/s, bias not removed] and temperature
static uint8_t readRaw(float accel[3], float gyro[3], float *temp){
    uint8_t rawData[14];

    if(!i2cRead(imuAddress, dataStartRegister, rawData, 14)){
        return 0;
    }

    for(int i = 0; i < 3; i++){
        int16_t rawAccel = (int16_t)(rawData[2*i]<<8 | rawData[2*i + 1]);
        int16_t rawGyro = (int16_t)(rawData[8 + 2*i]<<8 | rawData[8 + 2*i + 1]);
        accel[i] = (float)rawAccel / accelSF;
        gyro[i] = ((float)rawGyro / gyroSF) * degToRad;
    }

    int16_t rawTemp = (int16_t)(rawData[6]<<8 | rawData[7]);
    *temp = ((float)rawTemp / 340.0f) + 36.53f;

    return 1;
}


static void sensorToBody(const float sensor[3], float *x, float *y, float *z){
    *x = IMU_AXIS_X_SIGN * sensor[IMU_AXIS_X_SOURCE];
    *y = IMU_AXIS_Y_SIGN * sensor[IMU_AXIS_Y_SOURCE];
    *z = IMU_AXIS_Z_SIGN * sensor[IMU_AXIS_Z_SOURCE];
}


uint8_t i2cImuInit(void){

    uint8_t val;

    if(!i2cRead(imuAddress, whoAmIAddress, &val, 1) || val != 0x68){
        return 0;
    }

    if(!i2cWrite(imuAddress, mgmtAddress, clockPllGyroX)) return 0;   //wake up
    HAL_Delay(10);

    if(!i2cWrite(imuAddress, configRegister, dlpf98Hz)) return 0;
    if(!i2cWrite(imuAddress, sampleRateDivRegister, sampleRateDiv1kHz)) return 0;
    if(!i2cWrite(imuAddress, accelRegister, accelRange16g)) return 0;
    if(!i2cWrite(imuAddress, gyroRegister, gyroRange2000)) return 0;

    //Read the ranges back to catch a write that silently failed
    if(!i2cRead(imuAddress, accelRegister, &val, 1) || val != accelRange16g) return 0;
    if(!i2cRead(imuAddress, gyroRegister, &val, 1) || val != gyroRange2000) return 0;

    return 1;
}


void i2cImuSetCorrection(const imuCorrection *corr){

    if(corr == NULL){
        correctionOn = false;
        return;
    }
    correction = *corr;
    trimQuat = quaternionFromEuler(corr->trimRoll, corr->trimPitch, 0.0f);
    correctionOn = true;
}


uint8_t i2cImuCalibrateGyro(uint16_t samples){

    if(samples == 0) return 0;

    float sum[3] = {0.0f, 0.0f, 0.0f};
    float minVal[3] = { 1e9f,  1e9f,  1e9f};
    float maxVal[3] = {-1e9f, -1e9f, -1e9f};
    float accel[3], gyro[3], temp;

    for(uint16_t n = 0; n < samples; n++){
        if(!readRaw(accel, gyro, &temp)){
            return 0;
        }
        for(int i = 0; i < 3; i++){
            sum[i] += gyro[i];
            if(gyro[i] < minVal[i]) minVal[i] = gyro[i];
            if(gyro[i] > maxVal[i]) maxVal[i] = gyro[i];
        }
        HAL_Delay(1);
    }

    for(int i = 0; i < 3; i++){
        if(maxVal[i] - minVal[i] > calibrationMaxSpread){
            return 0;   //it moved
        }
    }

    for(int i = 0; i < 3; i++){
        gyroBias[i] = sum[i] / (float)samples;
    }

    return 1;
}


uint8_t i2cImuRead(i2cIMUData *imuData){

    float accel[3], gyro[3], temp;

    if(!readRaw(accel, gyro, &temp)){
        i2cBusRecover();
        return 0;
    }

    for(int i = 0; i < 3; i++){
        gyro[i] -= gyroBias[i];
    }

    vector3 a, w;
    sensorToBody(accel, &a.x, &a.y, &a.z);     //[g]
    sensorToBody(gyro, &w.x, &w.y, &w.z);

    if(correctionOn){
        a.x = correction.accelScale.x * (a.x - correction.accelOffset.x);
        a.y = correction.accelScale.y * (a.y - correction.accelOffset.y);
        a.z = correction.accelScale.z * (a.z - correction.accelOffset.z);
        a = quaternionRotate(trimQuat, a);
        w = quaternionRotate(trimQuat, w);
    }

    imuData->accelX = a.x * gravity;
    imuData->accelY = a.y * gravity;
    imuData->accelZ = a.z * gravity;
    imuData->gyroX = w.x;
    imuData->gyroY = w.y;
    imuData->gyroZ = w.z;
    imuData->temp = temp;

    return 1;
}
