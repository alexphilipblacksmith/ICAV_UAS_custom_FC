#ifndef ATTITUDECONTROL_H
#define ATTITUDECONTROL_H

#include<stdbool.h>

typedef struct{
    float kp[3];
    float ki[3];
    float kd[3];
    float integralLimit[3];
    float outputLimit[3];
    bool firstRun;
}attitudeController;


//Define a structure with individual quaternion components
typedef struct{
    float w, x, y, z;
}quaternion;

//Define a structure with individual vector components
typedef struct{
    float x, y, z;
}vector3


void attCtrlInit(attitudeController *attCtrl,
                 float kp[3],
                 float ki[3],
                 float kd[3],
                 float integralLimit[3], 
                 float outputLimit[3]);

void attCtrlUpdate(attitudeController *attCtrl,
                   float targetQuat[4], 
                   float measuredQuat[4], 
                   float outputs[3]);

void attCtrlReset(attitudeController *attCtrl);


#endif