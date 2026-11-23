#ifndef PWMTEST_H
#define PWMTEST_H

/* PWM output test, PROPS OFF (better: ESCs unplugged, scope or logic analyser on the pins).
 *
 * All four motor channels (PE9, PE11, PE13, PE14) get the same pulse width, stepping
 * 1000, 1100, ... 2000 us and holding each step for PWM_TEST_HOLD_MS. Then it goes back to
 * 1000 us and repeats. Each step is printed on the debug UART with the pulse widths read back
 * from the timer.
 *
 * Turned on with RUN_PWM_TEST in fcConfig.h: main.c then builds ONLY this test - no IMU,
 * radio, estimator or flight control code runs. */

void pwmTestInit(void);     // after motorInit and debugUartInit
void pwmTestUpdate(void);   // call from the main loop, as often as you like (doesn't block)

#endif // PWMTEST_H
