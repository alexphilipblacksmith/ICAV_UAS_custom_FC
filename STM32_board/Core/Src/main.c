/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fcConfig.h"
#include "i2c_IMU.h"
#include "rcInput.h"
#include "motorOutput.h"
#include "flightControl.h"
#include "attitudeEstimator.h"
#include "imuCalibration.h"
#include "timing.h"
#include "debugUart.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len){
  (void)file;
  return debugUartWrite(ptr, len);
}

// Something we can't fly without failed at start-up: say so and blink the LED fast forever.
// The motors stay stopped.
static void startupFault(const char *what){
  printf("%s failed - not flying\r\n", what);
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    HAL_Delay(100);
  }
}

// ESC calibration, PROPS OFF:
// 1. Battery unplugged, board on USB. 2. Arm switch off, throttle stick at the top, radio on.
// 3. Reset the board: it prints "ESC calibration". 4. Plug in the battery, wait for the ESC beeps.
// 5. Throttle to the bottom, wait for the confirm beeps. 6. Unplug everything.
// The throttle stick goes straight to every ESC until power-off.
static void escCalibrationCheck(void){
  radioCommand cmd = {0};
  uint32_t start = HAL_GetTick();
  bool linked = false;

  while (HAL_GetTick() - start < 3000 && !linked)   // give the receiver time to link up
  {
    rcInputUpdate();
    linked = rcInputGetCommand(&cmd);
  }
  if (!linked || cmd.armSwitch || cmd.throttle < 0.9f) return;

  printf("ESC calibration: throttle goes straight to the ESCs. Power off to exit\r\n");
  while (1)
  {
    rcInputUpdate();
    if (rcInputGetCommand(&cmd)) motorWriteAllUs((uint16_t)(MOTOR_STOP_US + cmd.throttle * (MOTOR_MAX_US - MOTOR_STOP_US)));
    else motorWriteAllUs(MOTOR_STOP_US);
  }
}

// Accelerometer calibration, picked with the sticks at power-up (arm switch off, throttle down):
//   roll stick held full LEFT  -> accel calibration (pick the drone up, hold it still in many orientations)
//   roll stick held full RIGHT -> level trim (drone on a level surface)
// Runs after the gyro calibration, so keep the drone still until it starts. Power cycle afterwards.
static void imuCalibrationCheck(void){
  radioCommand cmd = {0};
  uint32_t start = HAL_GetTick();
  bool linked = false;
  while (HAL_GetTick() - start < 1000 && !linked)
  {
    rcInputUpdate();
    linked = rcInputGetCommand(&cmd);
  }
  if (!linked || cmd.armSwitch || cmd.throttle > 0.1f) return;

  if (cmd.roll < -0.8f) imuCalRunAccel();
  else if (cmd.roll > 0.8f) imuCalRunLevel();
}

static const char *armStateName(fcArmState s){
  switch (s)
  {
    case FC_ARMED: return "ARMED";
    case FC_FAILSAFE: return "FAILSAFE";
    default: return "DISARMED";
  }
}

// One status line, 10 Hz. Integers only (nano printf has no floats).
// Accel in mm/s^2, gyro in mrad/s. Level and still: accel ~ (0, 0, -9810).
// Tilt nose down: accel x < 0. Right side down: accel y < 0.
// Roll right, nose up, yaw right (clockwise from above): each gyro axis goes positive.
// att = estimated roll, pitch in degrees (right side down / nose up positive).
static void printStatus(const fcStatus *fc, const i2cIMUData *imu, const attitudeEstimator *est, bool linkUp, uint32_t maxLoopUs){
  float roll = 0.0f, pitch = 0.0f;
  attEstGetRollPitch(est, &roll, &pitch);
  uint16_t pulses[4];
  motorGetPulses(pulses);
  const crsfParser *rc = rcInputGetParser();

  char line[240];
  int n = snprintf(line, sizeof(line),
    "%s(%s)  att %4d %4d  acc %6d %6d %6d  gyro %5d %5d %5d  thr %3d%%  m %4u %4u %4u %4u  rc %s LQ %3u  loop %4lu us\r\n",
    armStateName(fc->armState), fc->disarmReason,
    (int)(roll * 57.2958f), (int)(pitch * 57.2958f),
    (int)(imu->accelX * 1000.0f), (int)(imu->accelY * 1000.0f), (int)(imu->accelZ * 1000.0f),
    (int)(imu->gyroX * 1000.0f), (int)(imu->gyroY * 1000.0f), (int)(imu->gyroZ * 1000.0f),
    (int)(fc->thrust * 100.0f),
    (unsigned)pulses[0], (unsigned)pulses[1], (unsigned)pulses[2], (unsigned)pulses[3],
    linkUp ? "UP" : "LOST", (unsigned)rc->linkStats.uplinkLinkQuality, (unsigned long)maxLoopUs);
  if (n > 0) debugUartWrite(line, n < (int)sizeof(line) ? n : (int)sizeof(line) - 1);
}
// Starting attitude from the averaged accelerometer. Drone still, like the gyro calibration.
static bool attitudeInitFromAccel(attitudeEstimator *est){
  i2cIMUData imu;
  vector3 sum = {0.0f, 0.0f, 0.0f};
  int good = 0;
  for (int i = 0; i < ATT_EST_INIT_SAMPLES; i++)
  {
    rcInputUpdate();
    if (i2cImuRead(&imu))
    {
      sum.x += imu.accelX; sum.y += imu.accelY; sum.z += imu.accelZ;
      good++;
    }
    HAL_Delay(2);
  }
  if (good < ATT_EST_INIT_SAMPLES / 2) return false;
  float inv = 1.0f / (float)good;
  return attEstInit(est, (vector3){sum.x * inv, sum.y * inv, sum.z * inv});
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Motors stopped before anything else, so the ESCs see a valid stop signal
  motorInit(&htim1);
  debugUartInit();
  setvbuf(stdout, NULL, _IOLBF, 128);   // printf sends a line at a time
  timingInit();
  printf("\r\nICAV FC starting\r\n");

  if (!i2cImuInit()) startupFault("MPU6050");
  imuCalStatus imuCal = imuCalLoad();
  printf("Accel calibration: %s, level trim: %s\r\n",
         imuCal.accelCalibrated ? "yes" : "NOT DONE", imuCal.levelTrimmed ? "yes" : "NOT DONE");

  rcInputInit(&huart2);
  escCalibrationCheck();

  // Gyro bias: keep the drone still
  while (!i2cImuCalibrateGyro(1000))
  {
    printf("Gyro calibration failed (moved?), retrying\r\n");
    HAL_Delay(500);
  }

  imuCalibrationCheck();

  // Attitude estimate (Mahony): roll and pitch from the accelerometer to start, yaw = 0
  static attitudeEstimator attEst;
  while (!attitudeInitFromAccel(&attEst))
  {
    printf("Attitude init failed (moving or IMU error), retrying\r\n");
    HAL_Delay(500);
  }

  flightControlInit();
  printf("Ready. Angle mode: arm switch on with throttle down to arm\r\n");

  i2cIMUData imu = {0};
  const uint32_t periodUs = 1000000u / LOOP_RATE_HZ;
  uint32_t lastLoopUs = micros();
  uint32_t nextLoopUs = lastLoopUs + periodUs;
  uint32_t loopCount = 0, maxLoopUs = 0, ledTick = 0;
  int imuFailures = 0;

  watchdogStart(100);   // if the loop ever hangs for 100 ms the board resets (motors stop on reset)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ---- Wait for the next slot (500 Hz) ----
    while ((int32_t)(micros() - nextLoopUs) < 0) {}
    uint32_t nowUs = micros();
    float dt = (float)(nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;
    nextLoopUs += periodUs;
    if ((int32_t)(nowUs - nextLoopUs) > (int32_t)periodUs) nextLoopUs = nowUs + periodUs;   // way behind: don't try to catch up
    loopCount++;

    // ---- Sensors ----
    rcInputUpdate();

    if (i2cImuRead(&imu))
    {
      imuFailures = 0;
      vector3 gyro = {imu.gyroX, imu.gyroY, imu.gyroZ};
      vector3 accel = {imu.accelX, imu.accelY, imu.accelZ};
      if (attEst.initialised) attEstUpdate(&attEst, gyro, accel, dt);
      else attEstInit(&attEst, accel);   // lost it (NaN): restart from the accel. Disarms meanwhile
    }
    else imuFailures++;

    // ---- Control ----
    radioCommand cmd;
    bool linkUp = rcInputGetCommand(&cmd);

    vehicleState state;
    state.attitude = attEst.q;
    state.attitudeValid = attEst.initialised;
    state.gyro = (vector3){imu.gyroX, imu.gyroY, imu.gyroZ};
    state.imuHealthy = imuFailures < IMU_FAIL_LIMIT;
    flightControlUpdate(&state, &cmd, linkUp, dt);

    // ---- Housekeeping ----
    const fcStatus *fc = flightControlStatus();
    uint32_t loopUs = micros() - nowUs;
    if (loopUs > maxLoopUs) maxLoopUs = loopUs;

    // LED (active low): solid when armed, slow blink when disarmed
    if (fc->armState != FC_DISARMED) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    else if (++ledTick >= LOOP_RATE_HZ / 2) { ledTick = 0; HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); }

    if (loopCount % DEBUG_RATE_DIVIDER == 7 && debugUartIdle())
    {
      printStatus(fc, &imu, &attEst, linkUp, maxLoopUs);
      maxLoopUs = 0;
    }

    watchdogKick();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 420000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* 1 us ticks (168 MHz / 168), 2500 us period = 400 Hz. Starts at 1000 us = motors stopped */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = GPIO_PIN_6; // PA6, PA7 = onboard LEDs
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      // Push-pull output
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Motors off first - TIM1 keeps sending the stop pulse even with interrupts off */
  motorEmergencyStop();
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
