#include "i2cBus.h"


uint8_t i2cRead(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len){
    return HAL_I2C_Mem_Read(&hi2c1, address << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT_MS) == HAL_OK;
}


uint8_t i2cWrite(uint8_t address, uint8_t reg, uint8_t value){
    return HAL_I2C_Mem_Write(&hi2c1, address << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}


static void shortDelay(void){
    for(volatile uint32_t i = 0; i < 200; i++){}   // a few us
}


void i2cBusRecover(void){

    // Hand the pins back to GPIO and reset the peripheral (also clears a stuck BUSY flag)
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    shortDelay();

    // Up to 9 clocks until SDA is released
    for(int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; i++){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        shortDelay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        shortDelay();
    }

    // STOP condition: SDA goes high while SCL is high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    shortDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    shortDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    shortDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    shortDelay();

    // Init settings are still in hi2c1, and MspInit puts the pins back in I2C mode
    HAL_I2C_Init(&hi2c1);
}
