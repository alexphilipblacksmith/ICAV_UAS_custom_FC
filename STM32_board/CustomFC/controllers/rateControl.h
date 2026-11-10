#ifndef RATECONTROL_H
#define RATECONTROL_H

#include "PID.h"

/* Body angular rate PID controller.
 * Array convention for all 3-element arrays:
 *   [0] = roll rate  (p, body x)
 *   [1] = pitch rate (q, body y)
 *   [2] = yaw rate (r, body z)
 */
typedef struct {
    PIDController rollRatePID;
    PIDController pitchRatePID;
    PIDController yawRatePID;
} rateController;

void rateCtrlInit(rateController *rateCtrl, 
                  float kp[3], 
                  float ki[3], 
                  float kd[3], 
                  float integralLimit[3], 
                  float outputLimit[3]);

/* Updates rate loops.
 * setpoint: desired body rates [p, q, r] [rad/s]
 * measuredValue: measured gyro rates [p, q, r] [rad/s]
 * outputs: commanded body moments [L, M, N] [N*m] */
void rateCtrlUpdate(rateController *rateCtrl, 
                    float setpoint[3], 
                    float measuredValue[3], 
                    float dt, 
                    float outputs[3]);

void rateCtrlReset(rateController *rateCtrl);

#endif // RATECONTROL_H

