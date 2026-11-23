#ifndef ATTITUDEESTIMATOR_H
#define ATTITUDEESTIMATOR_H

/* Mahony filter, IMU only (gyro + accelerometer, no magnetometer).
 *
 * The gyro is integrated into the quaternion. The accelerometer gives the "up" direction, and the
 * angle between it and the up direction the quaternion predicts is fed back to the gyro (Kp, plus Ki
 * to learn leftover gyro bias). This stops roll and pitch drifting.
 *
 * Yaw has nothing to correct it, so it starts at 0 and drifts slowly. That's fine for angle mode:
 * the yaw stick only commands a rate and the heading target follows the estimate.
 *
 * Frames are the same as everywhere else: body FRD, world NED, quaternion body -> world. */

#include <stdbool.h>
#include "quaternion.h"

typedef struct{
    quaternion q;           // body -> world
    vector3 gyroBias;       // integral term [rad/s], learnt in flight
    bool initialised;
    bool accelUsed;         // last update trusted the accelerometer (debug)
} attitudeEstimator;

// Start from the accelerometer (drone still), so roll and pitch are right straight away.
// Returns false if the accel reading makes no sense (|a| far from 1 g).
bool attEstInit(attitudeEstimator *est, vector3 accel);

// gyro [rad/s] and accel [m/s^2] in the body frame, as from i2cImuRead()
void attEstUpdate(attitudeEstimator *est, vector3 gyro, vector3 accel, float dt);

// Roll, pitch (ZYX Euler) [rad], for printing
void attEstGetRollPitch(const attitudeEstimator *est, float *roll, float *pitch);

#endif // ATTITUDEESTIMATOR_H
