#ifndef I2C_IMU_H
#define I2C_IMU_H

#include "i2cBus.h"
#include "quaternion.h"

// IMU data in the body frame (FRD: x forward, y right, z down), same as the simulation.
// Level and still, accel reads about (0, 0, -9.81) since it measures the push of the ground (up).
typedef struct{
    float accelX, accelY, accelZ;   // [m/s^2]
    float gyroX, gyroY, gyroZ;      // [rad/s], bias removed
    float temp;                     // [degC]
} i2cIMUData;

// How the MPU6050 sits on the frame: body axis = sign * sensor axis (0 = x, 1 = y, 2 = z).
// Default: chip x arrow forward, components up. The chip's z points up, so y and z flip.
#define IMU_AXIS_X_SOURCE 0
#define IMU_AXIS_Y_SOURCE 1
#define IMU_AXIS_Z_SOURCE 2
#define IMU_AXIS_X_SIGN   ( 1.0f)
#define IMU_AXIS_Y_SIGN   (-1.0f)
#define IMU_AXIS_Z_SIGN   (-1.0f)

// Corrections from imuCalibration (body axes):
//   accel = scale * (raw - offset), then accel and gyro are both turned by the trim rotation,
//   which undoes the board sitting slightly crooked on the frame.
typedef struct{
    vector3 accelOffset;    // [g]
    vector3 accelScale;     // [-], 1 = no change
    float trimRoll;         // [rad] how the board is tilted relative to the frame
    float trimPitch;
} imuCorrection;

uint8_t i2cImuInit(void);                       // 1 = ok
void i2cImuSetCorrection(const imuCorrection *corr);   // NULL = raw readings (used while calibrating)
uint8_t i2cImuCalibrateGyro(uint16_t samples);  // keep the drone still. 0 = it moved or I2C failed
uint8_t i2cImuRead(i2cIMUData *imuData);        // 0 = I2C failed (bus gets recovered, data untouched)

#endif //I2C_IMU_H
