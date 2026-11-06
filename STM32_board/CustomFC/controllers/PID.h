#ifndef PID_H
#define PID_H
#include <stdbool.h>


typedef struct {
    float kp, ki, kd;
    float previousMeasured;
    float integral;
    float integralLimit, outputLimit;
    bool firstRun;
} PIDController;

void init(PIDController *pid, float kp, float ki, float kd, float integralLimit, float outputLimit);
void reset(PIDController *pid);
float update(PIDController *pid, float setpoint, float measuredValue, float dt);
float clipVals(float value, float min, float max);

#endif 