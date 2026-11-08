#ifndef ATTITUDECONTROL_H
#define ATTITUDECONTROL_H

#include<stdbool.h>

//Define a structure with individual quaternion components
typedef struct{
    float w, x, y, z;
}quaternion;

//Define a structure with individual vector components
typedef struct{
    float x, y, z;
}vector3;

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

//Quaternion operation declarations
quaternion quaternionConjugate(quaternion quat);
quaternion quaternionNormalise(quaternion quat);
quaternion quaternionMultiply(quaternion quat1, quaternion quat2);




#endif //ATTITUDECONTROL_H