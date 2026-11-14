#ifndef MIXER_H
#define MIXER_H

/* Plus frame, same layout as the simulation (quadrotor.py). Spin seen from above:
 *
 *                 1 (front)  CW
 *                 |
 *    CCW 4 (left) + 2 (right) CCW
 *                 |
 *                 3 (back)   CW
 *
 * Everything is normalised because we don't know the real motor constants yet:
 *   thrust 0..1, moments roughly -1..1 (+roll = right side down, +pitch = nose up, +yaw = nose right),
 *   motor commands 0..1 ([0] = motor 1 ... [3] = motor 4). */

#define MIXER_MOTOR_DIRS {1.0f, -1.0f, 1.0f, -1.0f}   // +1 = CW (motorDirs in the sim)

// If a motor would go past 0 or 1, keeps the moments and gives up thrust first (like desaturatedMixer in the sim)
void mixerPlus(float thrust, const float moments[3], float motorCmd[4]);

#endif // MIXER_H
