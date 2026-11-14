#ifndef RCINPUT_H
#define RCINPUT_H

/* ELRS receiver on USART2 (420000 baud): receiver TX -> PA3, receiver RX -> PA2.
 * DMA fills a ring buffer in the background; call rcInputUpdate() at least every ~20 ms.
 * Keep the USART2 interrupt OFF in CubeMX (the HAL error handler would stop the DMA).
 *
 * Radio (Pocket): AETR channel order, arm on CH5.
 * Receiver: failsafe mode "No Pulses" (ELRS default) so a lost link stops the frames. */

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "crsf.h"

// 0-based channel numbers (CH1 = 0)
#define RC_CHANNEL_ROLL     0
#define RC_CHANNEL_PITCH    1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW      3
#define RC_CHANNEL_ARM      4

// Flip to -1 if a stick goes the wrong way. Right, forward and up must all be positive
#define RC_SIGN_ROLL     ( 1.0f)
#define RC_SIGN_PITCH    ( 1.0f)
#define RC_SIGN_YAW      ( 1.0f)
#define RC_SIGN_THROTTLE ( 1.0f)

#define RC_STICK_DEADBAND   0.05f
#define RC_SWITCH_ON_US     1700    // a switch counts as on above this
#define RC_FRAME_TIMEOUT_MS 100     // no frames for this long = link lost

// Same as RadioCommand in the sim, plus the arm switch
typedef struct{
    float roll;       // -1..1, + = right
    float pitch;      // -1..1, + = forward
    float yaw;        // -1..1, + = right
    float throttle;   // 0..1
    bool armSwitch;
} radioCommand;

void rcInputInit(UART_HandleTypeDef *huart);
void rcInputUpdate(void);
bool rcInputLinkUp(void);

// Latest sticks. If the link is down: sticks centred, throttle 0, arm off, returns false
bool rcInputGetCommand(radioCommand *cmd);

uint16_t rcInputGetChannel(uint8_t channel);   // raw 172..1811
const crsfParser *rcInputGetParser(void);      // link stats and counters

#endif // RCINPUT_H
