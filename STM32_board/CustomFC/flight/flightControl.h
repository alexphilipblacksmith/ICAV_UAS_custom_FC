#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

/* Arming, failsafe and the control cascade (C version of flightModes.py + flightController.py):
 *   sticks -> target attitude -> attitude loop -> rate loop -> mixer -> motors
 *
 * Angle mode only: sticks = tilt angle, like ManualFlightMode in the sim.
 *
 * Arming: arm switch flicked on (it must have been seen off first), throttle down, roughly level,
 *         IMU ok, and an attitude estimate available (attitudeValid).
 * Disarming: arm switch off, IMU or attitude lost, crash (tilted past CRASH_TILT_DEG), or link lost too long.
 * Link lost while flying: level out, hold the last throttle for FAILSAFE_HOLD_S, then cut the motors. */

#include <stdbool.h>
#include "quaternion.h"
#include "rcInput.h"

typedef struct{
    quaternion attitude;    // body -> world, from an attitude estimator
    bool attitudeValid;     // false = no estimate, won't arm
    vector3 gyro;           // body rates [rad/s]
    bool imuHealthy;
} vehicleState;

typedef enum{
    FC_DISARMED,
    FC_ARMED,
    FC_FAILSAFE,            // armed, link lost
} fcArmState;

typedef struct{
    fcArmState armState;
    float thrust;               // collective 0..1
    float motorCmd[4];
    const char *disarmReason;   // why we last disarmed
} fcStatus;

void flightControlInit(void);

// Every loop. Writes the motors.
void flightControlUpdate(const vehicleState *state, const radioCommand *cmd, bool linkUp, float dt);

const fcStatus *flightControlStatus(void);

#endif // FLIGHTCONTROL_H
