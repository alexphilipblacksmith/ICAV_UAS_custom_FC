#ifndef TIMING_H
#define TIMING_H

// Microsecond timing from the CPU cycle counter, and the independent watchdog.

#include "stm32f4xx_hal.h"

void timingInit(void);              // starts the cycle counter
uint32_t micros(void);              // wraps every ~25 s, so only use differences

void watchdogStart(uint16_t timeoutMs);   // can't be stopped once started. Paused while the debugger halts the CPU
void watchdogKick(void);

#endif // TIMING_H
