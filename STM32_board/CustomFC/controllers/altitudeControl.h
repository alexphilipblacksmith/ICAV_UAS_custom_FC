#ifndef ALTITUDECONTROL_H
#define ALTITUDECONTROL_H

#include "PID.h"

/* Altitude PID controller (altitude positive up).
 * Output is a thrust correction [N] added to the baseline hover thrust. */
typedef struct {
    PIDController altPID;
} altitudeController;

void altCtrlInit(altitudeController *altCtrl, 
                 float kp,
                 float ki, 
                 float kd, 
                 float integralLimit, 
                 float outputLimit);

/* Updates altitude loop.
 * setpoint: target altitude [m] (positive up)
 * measuredValue: current altitude [m] (positive up)
 * Returns: thrust correction [N] */
float altCtrlUpdate(altitudeController *altCtrl, 
                     float setpoint, 
                     float measuredValue, 
                     float dt);

void altCtrlReset(altitudeController *altCtrl);

#endif // ALTITUDECONTROL_H
