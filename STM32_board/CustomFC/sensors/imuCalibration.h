#ifndef IMUCALIBRATION_H
#define IMUCALIBRATION_H

/* Accelerometer calibration, saved in flash so it survives power-off.
 *
 * Two steps, both started from the sticks at power-up (see main.c):
 *
 * 1. Accel calibration (ellipsoid fit): pick the drone up and hold it still in lots of different
 *    orientations. Every time it's been still for ~0.5 s in a new direction, a sample is taken.
 *    It needs each of the six faces pointing roughly down at least once (any order, any angle
 *    in between helps too). Fixes the sensor's own offset and scale errors.
 *
 * 2. Level trim: drone on a flat, level surface. Fixes the board sitting slightly crooked on the
 *    frame, which step 1 can't see. Run it after every accel calibration.
 *
 * Both print progress on the debug UART and end in a loop: power cycle afterwards.
 *
 * Storage: the last flash sector (sector 7, 0x08060000, 128 KB). The linker script keeps code out
 * of it. A full chip erase wipes it, and the boot message then says "not calibrated". */

#include <stdbool.h>

typedef struct{
    bool accelCalibrated;
    bool levelTrimmed;
} imuCalStatus;

// At boot, after i2cImuInit: loads the saved calibration and hands it to the IMU driver
imuCalStatus imuCalLoad(void);

void imuCalRunAccel(void);   // never returns
void imuCalRunLevel(void);   // never returns

#endif // IMUCALIBRATION_H
