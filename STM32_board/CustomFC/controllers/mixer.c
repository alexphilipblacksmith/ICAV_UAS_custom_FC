#include "mixer.h"

void mixerPlus(float thrust, const float moments[3], float motorCmd[4]){

    static const float motorDirs[4] = MIXER_MOTOR_DIRS;
    float roll = moments[0], pitch = moments[1], yaw = moments[2];

    // Roll: left motor up, right down. Pitch: front up, back down.
    // Yaw: a motor pushes the body the opposite way to its spin, so the CCW pair speeds up for +yaw
    float mix[4];
    mix[0] =  pitch - motorDirs[0]*yaw;
    mix[1] = -roll  - motorDirs[1]*yaw;
    mix[2] = -pitch - motorDirs[2]*yaw;
    mix[3] =  roll  - motorDirs[3]*yaw;

    float minMix = mix[0], maxMix = mix[0];
    for(int i = 1; i < 4; i++){
        if(mix[i] < minMix) minMix = mix[i];
        if(mix[i] > maxMix) maxMix = mix[i];
    }

    // Moments alone need more than the whole motor range: shrink them
    float spread = maxMix - minMix;
    if(spread > 1.0f){
        float scale = 1.0f / spread;
        for(int i = 0; i < 4; i++) mix[i] *= scale;
        minMix *= scale;
        maxMix *= scale;
    }

    // Slide the thrust so every motor fits in 0..1
    if(thrust + maxMix > 1.0f) thrust = 1.0f - maxMix;
    if(thrust + minMix < 0.0f) thrust = -minMix;

    for(int i = 0; i < 4; i++){
        float cmd = thrust + mix[i];
        if(cmd < 0.0f) cmd = 0.0f;   // rounding
        if(cmd > 1.0f) cmd = 1.0f;
        motorCmd[i] = cmd;
    }
}
