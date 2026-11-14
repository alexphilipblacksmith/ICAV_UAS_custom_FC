#ifndef QUATERNION_H
#define QUATERNION_H

// Quaternions are [w, x, y, z] and rotate body vectors into the world frame (NED),
// same as the simulation's state vector.

typedef struct{
    float w, x, y, z;
}quaternion;

typedef struct{
    float x, y, z;
}vector3;

quaternion quaternionConjugate(quaternion quat);
quaternion quaternionNormalise(quaternion quat);
quaternion quaternionMultiply(quaternion quat1, quaternion quat2);   // quat1 * quat2 (order matters)
vector3 quaternionRotate(quaternion quat, vector3 vec);             // body -> world

// Yaw-pitch-roll (ZYX) angles [rad], same as scipy's from_euler('ZYX', [yaw, pitch, roll]) in the sim
quaternion quaternionFromEuler(float roll, float pitch, float yaw);
float quaternionYaw(quaternion quat);

#endif //QUATERNION_H
