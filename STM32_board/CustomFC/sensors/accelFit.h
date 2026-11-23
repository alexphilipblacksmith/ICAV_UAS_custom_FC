#ifndef ACCELFIT_H
#define ACCELFIT_H

/* Accelerometer calibration maths (no hardware here, so it can be tested on a PC).
 *
 * Held still, the accel should read exactly 1 g whichever way it points, so readings from many
 * orientations should lie on a sphere of radius 1 g. Offsets move the centre and scale errors
 * stretch each axis, giving an ellipsoid. This finds the offset and scale per axis that put the
 * samples back on the unit sphere:
 *
 *     corrected = scale * (raw - offset)      (per axis, everything in g)
 *
 * Least squares with Levenberg-Marquardt. The orientations don't matter, but each axis must have
 * been seen pointing roughly up AND down, otherwise offset and scale can't be told apart. */

#include <stdbool.h>
#include "quaternion.h"

// offset [g], scale [-], rmsError = how far the corrected samples are from 1 g on average [g]
bool accelFitEllipsoid(const vector3 *samples, int count, vector3 *offset, vector3 *scale, float *rmsError);

#endif // ACCELFIT_H
