#include "altitudeControl.h"

void altCtrlInit(altitudeController *altCtrl, float kp, float ki, float kd, float integralLimit, float outputLimit){

    PIDInit(&altCtrl->altPID, kp, ki, kd, integralLimit, outputLimit);
    
}

float altCtrlUpdate(altitudeController *altCtrl, float setpoint, float measuredValue, float dt){

    return PIDUpdate(&altCtrl -> altPID, setpoint, measuredValue, dt);

}

void altCtrlReset(altitudeController *altCtrl){

    PIDReset(&altCtrl -> altPID);
}
