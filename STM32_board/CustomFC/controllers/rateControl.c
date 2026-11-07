#include "rateControl.h"

void rateCtrlInit(rateController *rateCtrl, float kp[3], float ki[3], float kd[3], float integralLimit[3], float outputLimit[3]) {

    // [0] = roll, [1] = pitch, [2] = yaw
    PIDInit(&rateCtrl->rollPID,  kp[0], ki[0], kd[0], integralLimit[0], outputLimit[0]);
    PIDInit(&rateCtrl->pitchPID, kp[1], ki[1], kd[1], integralLimit[1], outputLimit[1]);
    PIDInit(&rateCtrl->yawPID,   kp[2], ki[2], kd[2], integralLimit[2], outputLimit[2]);

}

void rateCtrlUpdate(rateController *rateCtrl, float setpoint[3], float measuredValue[3], float dt, float outputs[3]) {

    // Output moments: [0] = L (roll), [1] = M (pitch), [2] = N (yaw) [N*m]
    outputs[0] = PIDUpdate(&rateCtrl->rollPID,  setpoint[0], measuredValue[0], dt);
    outputs[1] = PIDUpdate(&rateCtrl->pitchPID, setpoint[1], measuredValue[1], dt);
    outputs[2] = PIDUpdate(&rateCtrl->yawPID,   setpoint[2], measuredValue[2], dt);

}

void rateCtrlReset(rateController *rateCtrl) {
    PIDReset(&rateCtrl->rollPID);
    PIDReset(&rateCtrl->pitchPID);
    PIDReset(&rateCtrl->yawPID);
}

