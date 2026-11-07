#include "positionControl.h"

void posCtrlInit(positionController *posCtrl, float kp[2], float ki[2], float kd[2], float integralLimit[2], float outputLimit[2]) {

    // [0] = X (North), [1] = Y (East)
    PIDInit(&posCtrl->xPID, kp[0], ki[0], kd[0], integralLimit[0], outputLimit[0]);
    PIDInit(&posCtrl->yPID, kp[1], ki[1], kd[1], integralLimit[1], outputLimit[1]);


}

void posCtrlUpdate(positionController *posCtrl, float setpoint[2], float measuredValue[2], float dt, float outputs[2]) {

    // Output is desired acceleration in world coordinates [ax, ay] [m/s^2]
    outputs[0] = PIDUpdate(&posCtrl->xPID, setpoint[0], measuredValue[0], dt);
    outputs[1] = PIDUpdate(&posCtrl->yPID, setpoint[1], measuredValue[1], dt);

}

void posCtrlReset(positionController *posCtrl) {
    PIDReset(&posCtrl->xPID);
    PIDReset(&posCtrl->yPID);
}
