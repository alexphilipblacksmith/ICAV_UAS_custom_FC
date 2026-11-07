#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include "PID.h"

/* Horizontal position controller for world X and Y axes.
 * Array convention for all 2-element arrays:
 *   [0] = X (North)
 *   [1] = Y (East)
 */
typedef struct {
    PIDController xPID;
    PIDController yPID;
} positionController;

void posCtrlInit(positionController *posCtrl, 
                 float kp[2], 
                 float ki[2], 
                 float kd[2], 
                 float integralLimit[2], 
                 float outputLimit[2]);

/* Updates horizontal position loops.
 * setpoint: target world position [x, y] [m]
 * measuredValue: current world position [x, y] [m]
 * outputs: computed desired world acceleration [ax, ay] [m/s^2] */
void posCtrlUpdate(positionController *posCtrl, 
                   float setpoint[2], 
                   float measuredValue[2], 
                   float dt, 
                   float outputs[2]);

void posCtrlReset(positionController *posCtrl);

#endif // POSITIONCONTROL_H
