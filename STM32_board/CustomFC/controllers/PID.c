#include "PID.h"


//Declare functions
float clipVals(float value, float min, float max);


void init(PIDController *pid, float kp, float ki, float kd, float integralLimit, float outputLimit){
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integralLimit = integralLimit;
    pid->outputLimit = outputLimit;
    PIDReset(pid);

}

float update(PIDController *pid, float setpoint, float measuredValue, float dt){
        
    float error = setpoint - measuredValue;
    pid->integral += error * dt;

    if (pid->firstRun){
        pid->previousMeasured = measuredValue;
        pid->firstRun = false;
    }

    float derivative = -(measuredValue - pid->previousMeasured) / dt;
    
    pid->previousMeasured = measuredValue;
    if (pid->integralLimit > 0.0f){
        pid->integral = clipVals(pid->integral, -pid->integralLimit, pid->integralLimit);
    }

    if (pid->outputLimit > 0.0f){
        return clipVals(pid->kp * error + pid->ki * pid->integral + pid->kd * derivative, -pid->outputLimit, pid->outputLimit);
    }
    else {
        return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    }
}


void reset(PIDController *pid){

    pid->integral = 0.0f;
    pid->previousMeasured = 0.0f;
    pid->firstRun = true;
}


float clipVals(float value, float min, float max){
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
