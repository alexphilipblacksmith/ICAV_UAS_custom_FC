#ifndef MOTOROUTPUT_H
#define MOTOROUTPUT_H

/* PWM to the external ESCs on TIM1 (1 us steps, 400 Hz):
 *   motor 1 (front) PE9    motor 2 (right) PE11
 *   motor 3 (back)  PE13   motor 4 (left)  PE14
 * Wire each ESC's signal to its pin and its signal ground to the board's GND.
 * Don't connect an ESC's BEC 5V to the board if the board already has power from elsewhere.
 * If an ESC only takes 50 Hz servo PWM, set the TIM1 period to 19999 in CubeMX. */

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define MOTOR_COUNT 4

#define MOTOR_STOP_US 1000   // disarmed
#define MOTOR_IDLE_US 1070   // armed, zero throttle: props spin slowly
#define MOTOR_MAX_US  2000

void motorInit(TIM_HandleTypeDef *htim);                         // starts PWM with the motors stopped
void motorWrite(const float motorCmd[MOTOR_COUNT], bool armed);  // 0..1 -> idle..max when armed, stop otherwise
void motorWriteAllUs(uint16_t pulseUs);                          // same raw pulse on every ESC (ESC calibration only)
void motorEmergencyStop(void);                                   // safe from fault handlers, even before motorInit
void motorGetPulses(uint16_t pulsesUs[MOTOR_COUNT]);

#endif // MOTOROUTPUT_H
