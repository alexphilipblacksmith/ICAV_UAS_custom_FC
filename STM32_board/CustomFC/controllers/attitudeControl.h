#ifndef ATTITUDECONTROL_H
#define ATTITUDECONTROL_H

#include<stdbool.h>

#include "quaternion.h"

typedef struct{
    vector3 kp;
    vector3 ki;
    vector3 kd;
    vector3 integralLimit;
    vector3 outputLimit;
    vector3 integral;
    vector3 previousError;
    bool firstRun;
}attitudeController;


//Function declarations
void attCtrlInit(attitudeController *attCtrl,
                 vector3 kp,
                 vector3 ki,
                 vector3 kd,
                 vector3 integralLimit, 
                 vector3 outputLimit);

void attCtrlUpdate(attitudeController *attCtrl,
                   quaternion targetQuat, 
                   quaternion measuredQuat,
                   float dt, 
                   float outputs[3]);


void attCtrlReset(attitudeController *attCtrl);

#endif //ATTITUDECONTROL_H