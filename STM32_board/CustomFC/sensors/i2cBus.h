#ifndef I2CBUS_H
#define I2CBUS_H

// I2C1 bus (PB8 SCL, PB9 SDA, 400 kHz). The MPU6050 is on it; more sensors can share it.

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define I2C_TIMEOUT_MS 2    // short, so a stuck bus can't stall the control loop

// Return 1 on success, 0 on failure. Addresses are 7-bit.
uint8_t i2cRead(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t i2cWrite(uint8_t address, uint8_t reg, uint8_t value);

// Clocks SCL until a stuck device lets go of SDA, then restarts I2C1.
void i2cBusRecover(void);

#endif // I2CBUS_H
