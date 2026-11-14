#include "motorOutput.h"

static TIM_HandleTypeDef *motorTim = NULL;
static uint16_t lastPulses[MOTOR_COUNT] = {MOTOR_STOP_US, MOTOR_STOP_US, MOTOR_STOP_US, MOTOR_STOP_US};

static const uint32_t motorChannels[MOTOR_COUNT] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};


static void setPulse(int motor, uint16_t pulse){
    // Compare registers are preloaded, so the change lands at the start of the next period
    __HAL_TIM_SET_COMPARE(motorTim, motorChannels[motor], pulse);
    lastPulses[motor] = pulse;
}


void motorInit(TIM_HandleTypeDef *htim){

    motorTim = htim;

    for(int i = 0; i < MOTOR_COUNT; i++){
        setPulse(i, MOTOR_STOP_US);
    }
    for(int i = 0; i < MOTOR_COUNT; i++){
        HAL_TIM_PWM_Start(motorTim, motorChannels[i]);   // also turns on TIM1's main output (MOE)
    }
}


void motorWrite(const float motorCmd[MOTOR_COUNT], bool armed){

    if(motorTim == NULL) return;

    for(int i = 0; i < MOTOR_COUNT; i++){
        uint16_t pulse = MOTOR_STOP_US;

        if(armed){
            float cmd = motorCmd[i];
            if(!(cmd >= 0.0f)) cmd = 0.0f;   // also catches NaN
            if(cmd > 1.0f) cmd = 1.0f;
            pulse = (uint16_t)(MOTOR_IDLE_US + cmd * (float)(MOTOR_MAX_US - MOTOR_IDLE_US) + 0.5f);
        }

        setPulse(i, pulse);
    }
}


void motorWriteAllUs(uint16_t pulseUs){

    if(motorTim == NULL) return;
    if(pulseUs < MOTOR_STOP_US) pulseUs = MOTOR_STOP_US;
    if(pulseUs > MOTOR_MAX_US) pulseUs = MOTOR_MAX_US;

    for(int i = 0; i < MOTOR_COUNT; i++){
        setPulse(i, pulseUs);
    }
}


void motorEmergencyStop(void){
    // Straight to the registers: works with interrupts off and the HAL in any state
    TIM1->CCR1 = MOTOR_STOP_US;
    TIM1->CCR2 = MOTOR_STOP_US;
    TIM1->CCR3 = MOTOR_STOP_US;
    TIM1->CCR4 = MOTOR_STOP_US;
    for(int i = 0; i < MOTOR_COUNT; i++) lastPulses[i] = MOTOR_STOP_US;
}


void motorGetPulses(uint16_t pulsesUs[MOTOR_COUNT]){
    for(int i = 0; i < MOTOR_COUNT; i++) pulsesUs[i] = lastPulses[i];
}
