#include "pwmTest.h"
#include "fcConfig.h"
#include "motorOutput.h"
#include <stdio.h>

#define ledBlinkMs 250

static uint16_t currentUs;
static uint32_t stepStartMs;
static uint32_t lastBlinkMs;
static uint32_t cycle;


static void setStep(uint16_t us){

    currentUs = us;
    stepStartMs = HAL_GetTick();
    motorWriteAllUs(us);

    if(us == PWM_TEST_MIN_US){
        cycle++;
        printf("-- cycle %lu --\r\n", (unsigned long)cycle);
    }

    // Read back what the timer is actually set to
    uint16_t pulses[MOTOR_COUNT];
    motorGetPulses(pulses);
    printf("set %4u us   PE9 %4u  PE11 %4u  PE13 %4u  PE14 %4u\r\n", (unsigned)us,
           (unsigned)pulses[0], (unsigned)pulses[1], (unsigned)pulses[2], (unsigned)pulses[3]);
}


void pwmTestInit(void){

    printf("\r\n=== PWM TEST === (RUN_PWM_TEST in fcConfig.h, flight code not built)\r\n"
           "PROPS OFF. All channels step %d..%d us in %d us steps, %d ms each, then repeat.\r\n",
           PWM_TEST_MIN_US, PWM_TEST_MAX_US, PWM_TEST_STEP_US, PWM_TEST_HOLD_MS);
    printf("Channels: PE9 = motor 1 (front), PE11 = motor 2 (right), PE13 = motor 3 (back), PE14 = motor 4 (left)\r\n");

    cycle = 0;
    lastBlinkMs = HAL_GetTick();
    setStep(PWM_TEST_MIN_US);
}


void pwmTestUpdate(void){

    uint32_t now = HAL_GetTick();

    // LED blinks so you can see it's alive
    if(now - lastBlinkMs >= ledBlinkMs){
        lastBlinkMs = now;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    }

    if(now - stepStartMs < PWM_TEST_HOLD_MS) return;

    // Next step, or back to the start after the top one
    uint32_t next = (uint32_t)currentUs + PWM_TEST_STEP_US;
    setStep(next > PWM_TEST_MAX_US ? PWM_TEST_MIN_US : (uint16_t)next);
}
