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

    //Ensure the measuredQuat is normalised and find its inverse by using the fact
    //that the inverse of a quaternion who's norm is 1, is the conjugate of that
    //quaternion
    measuredQuat = quaternionNormalise(measuredQuat);
    quaternion invMeasuredQuat = quaternionConjugate(measuredQuat);

    //Quaternion error = hamilton product of invMeasuredQuat with targetQuat
    quaternion quatErr = quaternionMultiply(invMeasuredQuat, targetQuat);
    //Ensure quaternion takes the shortest way round:
    if (quatErr.w <0){
        quatErr = (quaternion){-quatErr.w, -quatErr.x, -quatErr.y, -quatErr.z};
    }
    //Create error vector using small angle approximation: err = 2 * quat(x,y,z)
    vector3 errVec = (vector3){2*quatErr.x, 2*quatErr.y, 2*quatErr.z};

    
    

    
    attCtrl->integral.x += errVec.x * dt;
    attCtrl->integral.y += errVec.y *dt;
    attCtrl->integral.z += errVec.z *dt;
    if (attCtrl->integralLimit.x != 0){
        attCtrl->integral.x = clipVals(attCtrl->integral.x, -attCtrl->integralLimit.x, attCtrl->integralLimit.x);
    } 
    if (attCtrl->integralLimit.y != 0){
        attCtrl->integral.y = clipVals(attCtrl->integral.y, -attCtrl->integralLimit.y, attCtrl->integralLimit.y);
    }
    if (attCtrl->integralLimit.z != 0){
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

    if(attCtrl->outputLimit.x != 0){
        outputs[0] = clipVals(outputs[0],-attCtrl->outputLimit.x, attCtrl->outputLimit.x);
        
    }
    if(attCtrl->outputLimit.y != 0){
        outputs[1] = clipVals(outputs[1], -attCtrl->outputLimit.y, attCtrl->outputLimit.y);
    }
    if(attCtrl->outputLimit.z != 0){
        outputs[2] = clipVals(outputs[2], -attCtrl->outputLimit.z, attCtrl->outputLimit.z);
    }

}

void attCtrlReset(attitudeController *attCtrl){
    attCtrl->integral = (vector3){0.0f, 0.0f, 0.0f};
    attCtrl->firstRun = true;
    attCtrl->previousError = (vector3){0.0f, 0.0f, 0.0f};

}




//Quaternion operations:
quaternion quaternionConjugate(quaternion quat){

    quaternion res;

    res.w = quat.w;
    res.x = -quat.x;
    res.y = -quat.y;
    res.z = -quat.z;

    return res;
}

quaternion quaternionNormalise(quaternion quat){
    
    float norm;
    quaternion res;

    norm = sqrtf((quat.w*quat.w) + (quat.x*quat.x) + (quat.y*quat.y) + (quat.z*quat.z));

    if(norm > 1e-6f){
        //Use invNorm, because 1 division and 4 multiplications is faster than 4 divisions
        float invNorm = 1.0f/norm;
        res.w = (quat.w) * invNorm;
        res.x = (quat.x) * invNorm;
        res.y = (quat.y) * invNorm;
        res.z = (quat.z) * invNorm;
    }else{
        res.w = 1.0f;
        res.x = 0.0f;
        res.y = 0.0f;
        res.z = 0.0f;
    }

    return res;
}


quaternion quaternionMultiply(quaternion quat1, quaternion quat2){

    //Function finds the hamilton product of quat1 with quat2
    //This is not the same as the hamilton product of quat2 with quat1

    quaternion res;

    res.w = quat1.w*quat2.w - quat1.x*quat2.x - quat1.y*quat2.y - quat1.z*quat2.z;
    res.x = quat1.w*quat2.x + quat1.x*quat2.w + quat1.y*quat2.z - quat1.z*quat2.y;
    res.y = quat1.w*quat2.y - quat1.x*quat2.z + quat1.y*quat2.w + quat1.z*quat2.x;
    res.z = quat1.w*quat2.z + quat1.x*quat2.y - quat1.y*quat2.x + quat1.z*quat2.w;

    return res;


}
