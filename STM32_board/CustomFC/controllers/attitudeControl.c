#include "attitudeControl.h"
#include "PID.h"
#include <math.h>


void attCtrlInit(attitudeController *attCtrl, vector3 kp, vector3 ki, vector3 kd, vector3 integralLimit, vector3 outputLimit){

   attCtrl->kp = kp;
   attCtrl->ki = ki;
   attCtrl->kd = kd;
   attCtrl->integralLimit = integralLimit;
   attCtrl->outputLimit = outputLimit;
   attCtrlReset(attCtrl);

}

void attCtrlUpdate(attitudeController *attCtrl, quaternion targetQuat, quaternion measuredQuat, float dt, float outputs[3]){

    if (dt <= 0.0f) return;

    //For a unit quaternion the inverse is just the conjugate
    measuredQuat = quaternionNormalise(measuredQuat);
    quaternion invMeasuredQuat = quaternionConjugate(measuredQuat);

    //Error rotation in the body frame
    quaternion quatErr = quaternionMultiply(invMeasuredQuat, targetQuat);
    //Take the shortest way round
    if (quatErr.w <0){
        quatErr = (quaternion){-quatErr.w, -quatErr.x, -quatErr.y, -quatErr.z};
    }

    //Error as axis * angle (same as as_rotvec() in the sim). Small angles: ~2 * (x, y, z)
    float sinHalf = sqrtf(quatErr.x*quatErr.x + quatErr.y*quatErr.y + quatErr.z*quatErr.z);
    float scale = 2.0f;
    if (sinHalf > 1e-6f){
        scale = 2.0f * atan2f(sinHalf, quatErr.w) / sinHalf;
    }
    vector3 errVec = (vector3){scale*quatErr.x, scale*quatErr.y, scale*quatErr.z};

    attCtrl->integral.x += errVec.x * dt;
    attCtrl->integral.y += errVec.y *dt;
    attCtrl->integral.z += errVec.z *dt;
    if (attCtrl->integralLimit.x > 0.0f){
        attCtrl->integral.x = clipVals(attCtrl->integral.x, -attCtrl->integralLimit.x, attCtrl->integralLimit.x);
    } 
    if (attCtrl->integralLimit.y > 0.0f){
        attCtrl->integral.y = clipVals(attCtrl->integral.y, -attCtrl->integralLimit.y, attCtrl->integralLimit.y);
    }
    if (attCtrl->integralLimit.z > 0.0f){
        attCtrl->integral.z = clipVals(attCtrl->integral.z, -attCtrl->integralLimit.z, attCtrl->integralLimit.z);
    }

    vector3 derivative;

    if(attCtrl->firstRun){
        derivative = (vector3){0.0f, 0.0f, 0.0f};
        attCtrl->firstRun = false;
    }else{
        //Use invDt, because 1 division and 3 multiplications is faster than 3 divisions
        float invDt = 1.0f/dt;
        derivative.x = (errVec.x -attCtrl->previousError.x) * invDt;
        derivative.y = (errVec.y - attCtrl->previousError.y) * invDt;
        derivative.z = (errVec.z - attCtrl->previousError.z) * invDt;
    }
    attCtrl->previousError = errVec;



    outputs[0] = attCtrl->kp.x*errVec.x + attCtrl->ki.x*attCtrl->integral.x + attCtrl->kd.x*derivative.x;
    outputs[1] = attCtrl->kp.y*errVec.y + attCtrl->ki.y*attCtrl->integral.y + attCtrl->kd.y*derivative.y;
    outputs[2] = attCtrl->kp.z*errVec.z + attCtrl->ki.z*attCtrl->integral.z + attCtrl->kd.z*derivative.z;

    if(attCtrl->outputLimit.x > 0.0f){
        outputs[0] = clipVals(outputs[0],-attCtrl->outputLimit.x, attCtrl->outputLimit.x);
        
    }
    if(attCtrl->outputLimit.y > 0.0f){
        outputs[1] = clipVals(outputs[1], -attCtrl->outputLimit.y, attCtrl->outputLimit.y);
    }
    if(attCtrl->outputLimit.z > 0.0f){
        outputs[2] = clipVals(outputs[2], -attCtrl->outputLimit.z, attCtrl->outputLimit.z);
    }

}

void attCtrlReset(attitudeController *attCtrl){
    attCtrl->integral = (vector3){0.0f, 0.0f, 0.0f};
    attCtrl->firstRun = true;
    attCtrl->previousError = (vector3){0.0f, 0.0f, 0.0f};

}
