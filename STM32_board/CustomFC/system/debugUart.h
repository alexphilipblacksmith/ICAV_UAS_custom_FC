#ifndef DEBUGUART_H
#define DEBUGUART_H

// printf over USART1 (PA9 TX, 115200) using DMA, so printing doesn't block the control loop.

#include <stdbool.h>
#include <stdint.h>

void debugUartInit(void);    // after MX_USART1_UART_Init
bool debugUartIdle(void);    // true when the last message has gone out

// Waits (up to ~50 ms) for the previous message, then starts sending this one.
// In the control loop, only print when debugUartIdle() so it never waits.
int debugUartWrite(const char *data, int len);

#endif // DEBUGUART_H
