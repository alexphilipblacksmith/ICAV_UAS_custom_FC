#include "flightControl.h"
#include "fcConfig.h"
#include "attitudeControl.h"
#include "rateControl.h"
#include "mixer.h"
#include "motorOutput.h"
#include <math.h>

#define degToRad 0.017453292519943295f
#define pi 3.14159265358979f

static attitudeController attCtrl;
static rateController rateCtrl;

static fcStatus status;
static float heading = 0.0f;            // heading target [rad]
static float failsafeThrust = 0.0f;
static float failsafeTime = 0.0f;
static float crashTime = 0.0f;
static bool armSwitchSeenOff = false;   // stops it arming if the switch is already on at power-up


static float clampf(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}


static float wrapPi(float angle){
    while(angle > pi) angle -= 2.0f * pi;
    while(angle < -pi) angle += 2.0f * pi;
    return angle;
}


// Two straight pieces so mid stick = hover, like the sim's ManualFlightMode
static float throttleCurve(float stick){
    if(stick < 0.5f){
        return 2.0f * stick * HOVER_THROTTLE;
    }
    return HOVER_THROTTLE + 2.0f * (stick - 0.5f) * (1.0f - HOVER_THROTTLE);
}


static void resetControllers(void){
    attCtrlReset(&attCtrl);
    rateCtrlReset(&rateCtrl);
}


static void disarm(const char *reason){
    static const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    status.armState = FC_DISARMED;
    status.disarmReason = reason;
    motorWrite(zero, false);
}


void flightControlInit(void){

    attCtrlInit(&attCtrl,
                (vector3){ATT_KP_ROLL, ATT_KP_PITCH, ATT_KP_YAW},
                (vector3){0.0f, 0.0f, 0.0f},
                (vector3){0.0f, 0.0f, 0.0f},
                (vector3){0.0f, 0.0f, 0.0f},
                (vector3){MAX_RATE_ROLL, MAX_RATE_PITCH, MAX_RATE_YAW});

    float kp[3] = {RATE_KP_ROLL, RATE_KP_PITCH, RATE_KP_YAW};
    float ki[3] = {RATE_KI_ROLL, RATE_KI_PITCH, RATE_KI_YAW};
    float kd[3] = {RATE_KD_ROLL, RATE_KD_PITCH, RATE_KD_YAW};
    float iLimit[3] = {RATE_I_LIMIT, RATE_I_LIMIT, RATE_I_LIMIT};
    float outLimit[3] = {RATE_OUT_LIMIT, RATE_OUT_LIMIT, RATE_OUT_LIMIT};
    rateCtrlInit(&rateCtrl, kp, ki, kd, iLimit, outLimit);

    status = (fcStatus){0};
    disarm("startup");
}


void flightControlUpdate(const vehicleState *state, const radioCommand *cmd, bool linkUp, float dt){

    quaternion q = state->attitude;
    float cosTilt = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);     // body z vs world z
    float tiltDeg = acosf(clampf(cosTilt, -1.0f, 1.0f)) / degToRad;
    float yaw = quaternionYaw(q);

    // The switch only counts as "off" while we can actually hear the radio
    if(linkUp && !cmd->armSwitch) armSwitchSeenOff = true;

    // ---- Arm / disarm / failsafe ----
    switch(status.armState){

        case FC_DISARMED:
            if(linkUp && cmd->armSwitch && armSwitchSeenOff && state->imuHealthy && state->attitudeValid
               && cmd->throttle < ARM_MAX_THROTTLE && tiltDeg < ARM_MAX_TILT_DEG){
                status.armState = FC_ARMED;
                armSwitchSeenOff = false;
                crashTime = 0.0f;
                heading = yaw;
                resetControllers();
            }
            break;

        case FC_ARMED:
            if(!linkUp){
                status.armState = FC_FAILSAFE;
                failsafeTime = 0.0f;
                failsafeThrust = status.thrust;
            }else if(!cmd->armSwitch){
                disarm("switch");
            }
            break;

        case FC_FAILSAFE:
            failsafeTime += dt;
            if(linkUp){
                if(cmd->armSwitch) status.armState = FC_ARMED;   // link back, carry on
                else disarm("switch");
            }else if(failsafeTime > FAILSAFE_HOLD_S){
                disarm("failsafe");
            }
            break;
    }

    if(status.armState != FC_DISARMED){
        if(!state->imuHealthy){
            disarm("imu");
        }else if(!state->attitudeValid){
            disarm("attitude");
        }else if(tiltDeg > CRASH_TILT_DEG){
            crashTime += dt;
            if(crashTime > CRASH_TIME_S) disarm("crash");
        }else{
            crashTime = 0.0f;
        }
    }

    if(status.armState == FC_DISARMED){
        // Keep everything fresh for the next arm
        resetControllers();
        heading = yaw;
        status.thrust = 0.0f;
        for(int i = 0; i < 4; i++) status.motorCmd[i] = 0.0f;
        return;
    }

    // ---- Setpoint ----
    float targetRoll, targetPitch, yawRate, thrust;

    if(status.armState == FC_FAILSAFE){
        targetRoll = 0.0f;
        targetPitch = 0.0f;
        yawRate = 0.0f;
        thrust = failsafeThrust;
    }else{
        float maxTilt = MAX_TILT_DEG * degToRad;
        targetRoll = cmd->roll * maxTilt;
        targetPitch = -cmd->pitch * maxTilt;   // stick forward = nose down
        yawRate = cmd->yaw * MAX_RATE_YAW;
        thrust = throttleCurve(cmd->throttle);

        // Throttle right down: probably on the ground, so stop the integrators winding up
        if(cmd->throttle < LOW_THROTTLE){
            resetControllers();
            heading = yaw;
        }
    }

    heading = wrapPi(heading + yawRate * dt);
    status.thrust = thrust;

    // ---- Cascade: attitude -> rates -> moments -> motors ----
    quaternion target = quaternionFromEuler(targetRoll, targetPitch, heading);

    float rates[3];
    attCtrlUpdate(&attCtrl, target, q, dt, rates);
    rates[2] += yawRate;    // feedforward, so yaw doesn't lag behind the stick
    rates[0] = clampf(rates[0], -MAX_RATE_ROLL, MAX_RATE_ROLL);
    rates[1] = clampf(rates[1], -MAX_RATE_PITCH, MAX_RATE_PITCH);
    rates[2] = clampf(rates[2], -MAX_RATE_YAW, MAX_RATE_YAW);

    float measured[3] = {state->gyro.x, state->gyro.y, state->gyro.z};
    float moments[3];
    rateCtrlUpdate(&rateCtrl, rates, measured, dt, moments);

    mixerPlus(thrust, moments, status.motorCmd);
    motorWrite(status.motorCmd, true);
}


const fcStatus *flightControlStatus(void){
    return &status;
}
