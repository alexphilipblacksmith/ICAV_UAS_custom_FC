#ifndef PID_H
#define PID_H

#include <stdbool.h>

/* Single-axis PID controller with anti-windup and derivative-on-measurement */
typedef struct {
    float kp, ki, kd;
    float previousMeasured;
    float integral;
    float integralLimit;    /* Max |integral| term (anti-windup). Set <= 0 for no limit */
    float outputLimit;      /* Max |output| magnitude. Set <= 0 for no limit */
    bool firstRun;          /* Flag to suppress derivative kick on the first update */
} PIDController;

void PIDInit(PIDController *pid, float kp, float ki, float kd, float integralLimit, float outputLimit);
void PIDReset(PIDController *pid);
float PIDUpdate(PIDController *pid, float setpoint, float measuredValue, float dt);
float clipVals(float value, float min, float max);

#endif // PID_H
