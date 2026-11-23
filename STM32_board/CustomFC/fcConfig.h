#ifndef FCCONFIG_H
#define FCCONFIG_H

/* All the tuning numbers in one place (like config.py in the simulation).
 * Gains are first guesses - tune them with the drone tied down. */

/* ---- Loop ---- */
#define LOOP_RATE_HZ        500         // main control loop
#define DEBUG_RATE_DIVIDER  50          // status print every 50 loops (10 Hz)

/* ---- I2C addresses (7-bit) ---- */
#define MPU6050_ADDRESS     0x68

/* ---- Limits (same as the simulation) ---- */
#define MAX_TILT_DEG        20.0f       // max roll/pitch from the sticks
#define MAX_RATE_ROLL       3.0f        // [rad/s]
#define MAX_RATE_PITCH      3.0f
#define MAX_RATE_YAW        2.0f

/* ---- Attitude estimator (Mahony, gyro + accel) ---- */
#define ATT_EST_KP          1.0f        // accel correction [1/s]. ~1 s time constant averages out vibration
#define ATT_EST_KI          0.02f       // gyro bias learning. 0 turns it off
#define ATT_EST_BIAS_LIMIT  0.05f       // max learnt bias [rad/s] (~3 deg/s)
#define ATT_EST_ACCEL_GATE  0.15f       // only use the accel when |a| is within 15% of 1 g
#define ATT_EST_INIT_SAMPLES 200        // accel samples averaged for the starting attitude

/* ---- Attitude loop: angle error [rad] -> rate [rad/s] ---- */
#define ATT_KP_ROLL         6.0f
#define ATT_KP_PITCH        6.0f
#define ATT_KP_YAW          3.0f

/* ---- Rate loop: rate error [rad/s] -> normalised moment (see mixer.h) ---- */
#define RATE_KP_ROLL        0.08f
#define RATE_KI_ROLL        0.10f
#define RATE_KD_ROLL        0.001f
#define RATE_KP_PITCH       0.08f
#define RATE_KI_PITCH       0.10f
#define RATE_KD_PITCH       0.001f
#define RATE_KP_YAW         0.15f
#define RATE_KI_YAW         0.10f
#define RATE_KD_YAW         0.0f
#define RATE_I_LIMIT        0.5f        // integral clamp
#define RATE_OUT_LIMIT      0.5f        // max moment command

/* ---- Throttle ---- */
#define HOVER_THROTTLE      0.5f        // thrust at mid stick. Raise/lower once you know the hover point
#define LOW_THROTTLE        0.05f       // below this we assume we're on the ground (integrators reset)

/* ---- Safety ---- */
#define ARM_MAX_THROTTLE    0.05f       // throttle must be down to arm
#define ARM_MAX_TILT_DEG    25.0f       // must be roughly level to arm
#define CRASH_TILT_DEG      70.0f       // disarm if tilted past this...
#define CRASH_TIME_S        0.5f        // ...for this long
#define FAILSAFE_HOLD_S     1.0f        // after link loss: level and hold throttle this long, then cut motors
#define IMU_FAIL_LIMIT      50          // disarm after this many failed IMU reads in a row (0.1 s)

#endif // FCCONFIG_H
