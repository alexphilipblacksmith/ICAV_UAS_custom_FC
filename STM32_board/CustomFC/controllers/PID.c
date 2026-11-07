#include "PID.h"

void PIDInit(PIDController *pid, float kp, float ki, float kd, float integralLimit, float outputLimit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integralLimit = integralLimit;
    pid->outputLimit = outputLimit;
    PIDReset(pid);
}

void PIDReset(PIDController *pid) {
    pid->integral = 0.0f;
    pid->previousMeasured = 0.0f;
    pid->firstRun = true;
}

float PIDUpdate(PIDController *pid, float setpoint, float measuredValue, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measuredValue;

    // Accumulate error and clamp to prevent integrator windup
    pid->integral += error * dt;
    if (pid->integralLimit > 0.0f) {
        pid->integral = clipVals(pid->integral, -pid->integralLimit, pid->integralLimit);
    }

    // Derivative on measurement: d(error)/dt = -d(measured)/dt (prevents setpoint step kicks)
    float derivative = 0.0f;
    if (pid->firstRun) {
        pid->previousMeasured = measuredValue;
        pid->firstRun = false;
    } else {
        derivative = -(measuredValue - pid->previousMeasured) / dt;
    }
    pid->previousMeasured = measuredValue;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    if (pid->outputLimit > 0.0f) {
        output = clipVals(output, -pid->outputLimit, pid->outputLimit);
    }


    return output;
}

float clipVals(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
