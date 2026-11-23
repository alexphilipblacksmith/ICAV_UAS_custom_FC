#include "attitudeEstimator.h"
#include "fcConfig.h"
#include <math.h>

#define gravity 9.80665f


static float clampf(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}


bool attEstInit(attitudeEstimator *est, vector3 accel){

    est->gyroBias = (vector3){0.0f, 0.0f, 0.0f};
    est->accelUsed = false;
    est->initialised = false;
    est->q = (quaternion){1.0f, 0.0f, 0.0f, 0.0f};

    float norm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if(norm < (1.0f - ATT_EST_ACCEL_GATE) * gravity || norm > (1.0f + ATT_EST_ACCEL_GATE) * gravity){
        return false;
    }

    // Level and still the accel reads (0, 0, -g). Nose down: x < 0. Right side down: y < 0.
    float roll = atan2f(-accel.y, -accel.z);
    float pitch = atan2f(accel.x, sqrtf(accel.y*accel.y + accel.z*accel.z));
    est->q = quaternionFromEuler(roll, pitch, 0.0f);
    est->initialised = true;
    return true;
}


void attEstUpdate(attitudeEstimator *est, vector3 gyro, vector3 accel, float dt){

    if(!est->initialised || dt <= 0.0f) return;

    quaternion q = est->q;
    vector3 w = gyro;

    // ---- Accelerometer correction ----
    // Only when it's mostly measuring gravity: hard manoeuvres and big bumps are skipped
    float norm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    est->accelUsed = norm > (1.0f - ATT_EST_ACCEL_GATE) * gravity && norm < (1.0f + ATT_EST_ACCEL_GATE) * gravity;

    if(est->accelUsed){
        float inv = 1.0f / norm;
        vector3 a = {accel.x * inv, accel.y * inv, accel.z * inv};

        // Where the quaternion thinks "up" (world -z) is, seen from the body: R^T * (0, 0, -1)
        vector3 v;
        v.x = -2.0f * (q.x*q.z - q.w*q.y);
        v.y = -2.0f * (q.y*q.z + q.w*q.x);
        v.z = -(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);

        // Error = measured up x predicted up. Turning by it lines the two up
        vector3 e;
        e.x = a.y*v.z - a.z*v.y;
        e.y = a.z*v.x - a.x*v.z;
        e.z = a.x*v.y - a.y*v.x;

        if(ATT_EST_KI > 0.0f){
            float lim = ATT_EST_BIAS_LIMIT;
            est->gyroBias.x = clampf(est->gyroBias.x + ATT_EST_KI * e.x * dt, -lim, lim);
            est->gyroBias.y = clampf(est->gyroBias.y + ATT_EST_KI * e.y * dt, -lim, lim);
            est->gyroBias.z = clampf(est->gyroBias.z + ATT_EST_KI * e.z * dt, -lim, lim);
        }

        w.x += ATT_EST_KP * e.x;
        w.y += ATT_EST_KP * e.y;
        w.z += ATT_EST_KP * e.z;
    }

    w.x += est->gyroBias.x;
    w.y += est->gyroBias.y;
    w.z += est->gyroBias.z;

    // ---- Integrate: q_dot = 0.5 * q * (0, w) ----
    float h = 0.5f * dt;
    quaternion dq;
    dq.w = (-q.x*w.x - q.y*w.y - q.z*w.z) * h;
    dq.x = ( q.w*w.x + q.y*w.z - q.z*w.y) * h;
    dq.y = ( q.w*w.y - q.x*w.z + q.z*w.x) * h;
    dq.z = ( q.w*w.z + q.x*w.y - q.y*w.x) * h;

    q.w += dq.w;
    q.x += dq.x;
    q.y += dq.y;
    q.z += dq.z;
    est->q = quaternionNormalise(q);

    // Something went badly wrong (NaN from a garbage reading): start again from the accelerometer
    if(isnan(est->q.w) || isnan(est->q.x) || isnan(est->q.y) || isnan(est->q.z)){
        est->initialised = false;
    }
}


void attEstGetRollPitch(const attitudeEstimator *est, float *roll, float *pitch){

    quaternion q = est->q;
    *roll = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y));
    *pitch = asinf(clampf(2.0f*(q.w*q.y - q.z*q.x), -1.0f, 1.0f));
}
