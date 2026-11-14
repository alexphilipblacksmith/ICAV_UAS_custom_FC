#include "quaternion.h"
#include <math.h>


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
        float invNorm = 1.0f/norm;
        res.w = (quat.w) * invNorm;
        res.x = (quat.x) * invNorm;
        res.y = (quat.y) * invNorm;
        res.z = (quat.z) * invNorm;
    }else{
        res = (quaternion){1.0f, 0.0f, 0.0f, 0.0f};
    }

    return res;
}


quaternion quaternionMultiply(quaternion quat1, quaternion quat2){

    quaternion res;

    res.w = quat1.w*quat2.w - quat1.x*quat2.x - quat1.y*quat2.y - quat1.z*quat2.z;
    res.x = quat1.w*quat2.x + quat1.x*quat2.w + quat1.y*quat2.z - quat1.z*quat2.y;
    res.y = quat1.w*quat2.y - quat1.x*quat2.z + quat1.y*quat2.w + quat1.z*quat2.x;
    res.z = quat1.w*quat2.z + quat1.x*quat2.y - quat1.y*quat2.x + quat1.z*quat2.w;

    return res;
}


vector3 quaternionRotate(quaternion quat, vector3 vec){

    // Rotation matrix of q times v (same result as q * v * q^-1)
    float ww = quat.w*quat.w, xx = quat.x*quat.x, yy = quat.y*quat.y, zz = quat.z*quat.z;
    float wx = quat.w*quat.x, wy = quat.w*quat.y, wz = quat.w*quat.z;
    float xy = quat.x*quat.y, xz = quat.x*quat.z, yz = quat.y*quat.z;

    vector3 res;
    res.x = (ww + xx - yy - zz)*vec.x + 2.0f*(xy - wz)*vec.y + 2.0f*(xz + wy)*vec.z;
    res.y = 2.0f*(xy + wz)*vec.x + (ww - xx + yy - zz)*vec.y + 2.0f*(yz - wx)*vec.z;
    res.z = 2.0f*(xz - wy)*vec.x + 2.0f*(yz + wx)*vec.y + (ww - xx - yy + zz)*vec.z;

    return res;
}


quaternion quaternionFromEuler(float roll, float pitch, float yaw){

    float cr = cosf(0.5f*roll),  sr = sinf(0.5f*roll);
    float cp = cosf(0.5f*pitch), sp = sinf(0.5f*pitch);
    float cy = cosf(0.5f*yaw),   sy = sinf(0.5f*yaw);

    quaternion res;
    res.w = cr*cp*cy + sr*sp*sy;
    res.x = sr*cp*cy - cr*sp*sy;
    res.y = cr*sp*cy + sr*cp*sy;
    res.z = cr*cp*sy - sr*sp*cy;

    return res;
}


float quaternionYaw(quaternion quat){
    return atan2f(2.0f*(quat.w*quat.z + quat.x*quat.y), 1.0f - 2.0f*(quat.y*quat.y + quat.z*quat.z));
}
