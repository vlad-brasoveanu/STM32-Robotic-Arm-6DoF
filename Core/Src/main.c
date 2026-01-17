/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include "kinematics.h"
#include <string.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for StartTask_Monit */
osThreadId_t StartTask_MonitHandle;
const osThreadAttr_t StartTask_Monit_attributes = {
  .name = "StartTask_Monit",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for StartTask_Servo */
osThreadId_t StartTask_ServoHandle;
const osThreadAttr_t StartTask_Servo_attributes = {
  .name = "StartTask_Servo",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for StartTask_Logic */
osThreadId_t StartTask_LogicHandle;
const osThreadAttr_t StartTask_Logic_attributes = {
  .name = "StartTask_Logic",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for StartTask_HMI */
osThreadId_t StartTask_HMIHandle;
const osThreadAttr_t StartTask_HMI_attributes = {
  .name = "StartTask_HMI",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xMutex1 */
osMutexId_t xMutex1Handle;
const osMutexAttr_t xMutex1_attributes = {
  .name = "xMutex1"
};
/* USER CODE BEGIN PV */

#define BTN_SELECT_PORT GPIOC
#define BTN_SELECT_PIN  GPIO_PIN_0

#define BTN_PLUS_PORT   GPIOC
#define BTN_PLUS_PIN    GPIO_PIN_1

#define BTN_MINUS_PORT  GPIOC
#define BTN_MINUS_PIN   GPIO_PIN_2

#define BTN_MODE_PORT   GPIOC
#define BTN_MODE_PIN    GPIO_PIN_3

#define BTN_RECORD_PORT GPIOA
#define BTN_RECORD_PIN  GPIO_PIN_1

#define BTN_PLAY_PORT   GPIOA
#define BTN_PLAY_PIN    GPIO_PIN_4

#define BTN_HOME_PORT   GPIOB
#define BTN_HOME_PIN    GPIO_PIN_7

#define BTN_ESTOP_PORT  GPIOB
#define BTN_ESTOP_PIN   GPIO_PIN_0

#define RELAY_PORT      GPIOB
#define RELAY_PIN       GPIO_PIN_12

#define MAX_POINTS 20

#define HOME_M1   90.0f
#define HOME_M2   90.0f
#define HOME_M3   90.0f
#define HOME_M4   90.0f
#define HOME_M5   90.0f
#define HOME_GRIP 5.0f


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartTask_MonitorFcn(void *argument);
void StartTask_ServoFcn(void *argument);
void StartTask_LogicFcn(void *argument);
void StartTask_HMIFcn(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of xMutex1 */
  xMutex1Handle = osMutexNew(&xMutex1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StartTask_Monit */
  StartTask_MonitHandle = osThreadNew(StartTask_MonitorFcn, NULL, &StartTask_Monit_attributes);

  /* creation of StartTask_Servo */
  StartTask_ServoHandle = osThreadNew(StartTask_ServoFcn, NULL, &StartTask_Servo_attributes);

  /* creation of StartTask_Logic */
  StartTask_LogicHandle = osThreadNew(StartTask_LogicFcn, NULL, &StartTask_Logic_attributes);

  /* creation of StartTask_HMI */
  StartTask_HMIHandle = osThreadNew(StartTask_HMIFcn, NULL, &StartTask_HMI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_CTRL_GPIO_Port, RELAY_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT_BTN_Pin PLUS_BTN_Pin MINUS_BTN_Pin MODE_BTN_Pin */
  GPIO_InitStruct.Pin = SELECT_BTN_Pin|PLUS_BTN_Pin|MINUS_BTN_Pin|MODE_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RECORD_BTN_Pin PLAY_BTN_Pin */
  GPIO_InitStruct.Pin = RECORD_BTN_Pin|PLAY_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E_STOP_BTN_Pin HOME_BTN_Pin */
  GPIO_InitStruct.Pin = E_STOP_BTN_Pin|HOME_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_CTRL_Pin */
  GPIO_InitStruct.Pin = RELAY_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_CTRL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
typedef enum {
    PLAY_RETRACE, // Dus-Intors prin toate punctele
    PLAY_FAST     // Dus prin puncte, Intors direct
} PlaybackType;

volatile PlaybackType selectedPlaybackMode = PLAY_RETRACE;

typedef struct {
    float m1, m2, m3, m4, m5, grip;
} RobotPose;

RobotPose savedPoints[MAX_POINTS];
volatile int pointCount = 0;

typedef enum {
    MODE_MANUAL,
    MODE_PLAY
} RobotMode;

volatile RobotMode currentMode = MODE_MANUAL;
volatile uint8_t isPlaying = 0;
volatile uint8_t isEmergency = 0;
volatile uint8_t selectedMotor = 1;

volatile float angM1 = HOME_M1;
volatile float angM2 = HOME_M2;
volatile float angM3 = HOME_M3;
volatile float angM4 = HOME_M4;
volatile float angM5 = HOME_M5;
volatile float angGrip = HOME_GRIP;

char *motorNames[] = {"", "BASE (M1)", "SHLDR(M2)", "ELBOW(M3)", "ROLL (M4)", "PITCH(M5)", "GRIP (M6)"};

uint8_t MoveSynchronized(RobotPose target, int speedDelay) {

    if (isEmergency) return 1;

    float diff1 = target.m1 - angM1;
    float diff2 = target.m2 - angM2;
    float diff3 = target.m3 - angM3;
    float diff4 = target.m4 - angM4;
    float diff5 = target.m5 - angM5;
    float diffG = target.grip - angGrip;

    float maxDiff = 0;
    if(fabsf(diff1) > maxDiff) maxDiff = fabsf(diff1);
    if(fabsf(diff2) > maxDiff) maxDiff = fabsf(diff2);
    if(fabsf(diff3) > maxDiff) maxDiff = fabsf(diff3);

    int steps = (int)(maxDiff * 2.0f);
    if (steps < 10) steps = 10;

    float start1 = angM1, start2 = angM2, start3 = angM3;
    float start4 = angM4, start5 = angM5, startG = angGrip;

    for (int s = 1; s <= steps; s++) {

        if (isEmergency) return 1;
        if (isPlaying) {
             if (HAL_GPIO_ReadPin(BTN_PLAY_PORT, BTN_PLAY_PIN) == GPIO_PIN_RESET ||
                 HAL_GPIO_ReadPin(BTN_HOME_PORT, BTN_HOME_PIN) == GPIO_PIN_RESET ||
                 HAL_GPIO_ReadPin(BTN_MODE_PORT, BTN_MODE_PIN) == GPIO_PIN_RESET) {
                 return 1;
             }
        }

        float t = (float)s / (float)steps;

        osMutexAcquire(xMutex1Handle, osWaitForever);

        angM1 = start1 + t * diff1;
        angM2 = start2 + t * diff2;
        angM3 = start3 + t * diff3;
        angM4 = start4 + t * diff4;
        angM5 = start5 + t * diff5;
        angGrip = startG + t * diffG;

        osMutexRelease(xMutex1Handle);

        osDelay(speedDelay);
    }
    return 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask_MonitorFcn */
/**
  * @brief  Function implementing the StartTask_Monit thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_MonitorFcn */
void StartTask_MonitorFcn(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);

	  uint32_t adcZero = 0;
	  for(int i=0; i<50; i++) {
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, 10);
	      adcZero += HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Stop(&hadc1);
	      osDelay(2);
	  }
	  adcZero /= 50;

	  uint32_t adcLimit = adcZero + 900;

	  uint8_t overCurrentCount = 0;
	  /* Infinite loop */
	  for(;;)
	  {
	      // E-STOP FIZIC
	      if (HAL_GPIO_ReadPin(BTN_ESTOP_PORT, BTN_ESTOP_PIN) == GPIO_PIN_RESET) {
	          if (!isEmergency) {
	              isEmergency = 1;
	              HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
	          }
	      }

	      // MONITORIZARE CURENT
	      else if (!isEmergency) {

	          HAL_ADC_Start(&hadc1);
	          if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
	              uint32_t rawVal = HAL_ADC_GetValue(&hadc1);

	              if (rawVal > adcLimit) {
	                  overCurrentCount++;
	                  if (overCurrentCount > 10) {
	                      isEmergency = 1;
	                      HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
	                  }
	              } else {
	                  overCurrentCount = 0;
	              }
	          }
	      }
	      // C. Revenire
	      else {
	           // Robotul ramane in E-Stop pana la Reset
	      }

	      osDelay(20);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_ServoFcn */
/**
* @brief Function implementing the StartTask_Servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ServoFcn */
void StartTask_ServoFcn(void *argument)
{
  /* USER CODE BEGIN StartTask_ServoFcn */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_MOE_ENABLE(&htim1);
  /* Infinite loop */
  for(;;)
  {
	  if (!isEmergency) {
	          // CONVERSIE: GRADE (0-180) -> PULS (500-2500)
	          // Formula: Puls = 500 + (Unghi * 11.11)

		  	  osMutexAcquire(xMutex1Handle, osWaitForever);

	          uint32_t p1 = (uint32_t)(500 + angM1 * 11.11f);
	          uint32_t p2 = (uint32_t)(500 + angM2 * 11.11f);
	          uint32_t p3 = (uint32_t)(500 + angM3 * 11.11f);
	          uint32_t p4 = (uint32_t)(500 + angM4 * 11.11f);
	          uint32_t p5 = (uint32_t)(500 + angM5 * 11.11f);
	          uint32_t pG = (uint32_t)(500 + angGrip * 11.11f);

	          osMutexRelease(xMutex1Handle);

	          // Scriem in timere
	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, p1);
	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, p2);
	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, p3);
	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, p4);
	          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p5);
	          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pG);
	      }
	      osDelay(20);
	    }
  /* USER CODE END StartTask_ServoFcn */
}

/* USER CODE BEGIN Header_StartTask_LogicFcn */
/**
* @brief Function implementing the StartTask_Logic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_LogicFcn */
void StartTask_LogicFcn(void *argument)
{
  /* USER CODE BEGIN StartTask_LogicFcn */
  uint32_t lastToggle = 0;
  uint32_t lastMove = 0;
  /* Infinite loop */
  for(;;)
  {
      if (!isEmergency) {

        // A. MOD MANUAL
        if (currentMode == MODE_MANUAL) {

            // 1. SELECTARE MOTOR SI MODE
        	if (HAL_GPIO_ReadPin(BTN_MODE_PORT, BTN_MODE_PIN) == GPIO_PIN_RESET) {

        	                uint32_t pressStart = HAL_GetTick();
        	                uint8_t homeExecuted = 0; // Flag sa stim daca am facut Home

        	                // Cat timp tii apasat, numaram timpul
        	                while(HAL_GPIO_ReadPin(BTN_MODE_PORT, BTN_MODE_PIN) == GPIO_PIN_RESET) {

        	                    // Daca au trecut 3 secunde (3000 ms)
        	                    if (HAL_GetTick() - pressStart > 3000) {

        	                        // --- ACTIUNEA LUNGA: GO HOME ---
        	                        LCD_SetCursor(0, 0);
        	                        LCD_Print("GOING HOME...   ");

        	                        // Definim pozitia de casa
        	                        RobotPose homePose;
        	                        homePose.m1 = HOME_M1; homePose.m2 = HOME_M2; homePose.m3 = HOME_M3;
        	                        homePose.m4 = HOME_M4; homePose.m5 = HOME_M5; homePose.grip = HOME_GRIP;

        	                        // Executam miscarea
        	                        MoveSynchronized(homePose, 10);

        	                        // Confirmare
        	                        LCD_SetCursor(0, 0);
        	                        LCD_Print("HOME DONE       ");
        	                        osDelay(1000);

        	                        homeExecuted = 1; // Marcam ca am executat Home
        	                        break; // Iesim fortat din bucla de asteptare
        	                    }
        	                    osDelay(10);
        	                }

        	                // --- ACTIUNEA SCURTA: TOGGLE MODE ---
        	                // Executam schimbarea modului DOAR DACA NU am facut Home deja
        	                if (homeExecuted == 0) {
        	                    // Debounce simplu (sa nu fie zgomot)
        	                    if (HAL_GetTick() - pressStart > 50) {
        	                        selectedPlaybackMode = (selectedPlaybackMode == PLAY_RETRACE) ? PLAY_FAST : PLAY_RETRACE;
        	                    }
        	                }

        	                // Asigurare: Asteptam sa iei mana complet de pe buton inainte sa continuam
        	                while(HAL_GPIO_ReadPin(BTN_MODE_PORT, BTN_MODE_PIN) == GPIO_PIN_RESET) osDelay(10);
        	            }
            if (HAL_GPIO_ReadPin(BTN_SELECT_PORT, BTN_SELECT_PIN) == GPIO_PIN_RESET) {
                if (HAL_GetTick() - lastToggle > 300) {
                    selectedMotor++;
                    if (selectedMotor > 6) selectedMotor = 1;
                    lastToggle = HAL_GetTick();
                }
            }

            // 2. MISCARE MOTOR
            float dir = 0.0f;
            if (HAL_GPIO_ReadPin(BTN_PLUS_PORT, BTN_PLUS_PIN) == GPIO_PIN_RESET) dir = 1.0f;
            if (HAL_GPIO_ReadPin(BTN_MINUS_PORT, BTN_MINUS_PIN) == GPIO_PIN_RESET) dir = -1.0f;

            if (dir != 0.0f && (HAL_GetTick() - lastMove > 10)) {
                float step = 1.5f;

                osMutexAcquire(xMutex1Handle, osWaitForever);

                switch(selectedMotor) {
                    case 1: angM1 += dir * step; break;
                    case 2: angM2 += dir * step; break;
                    case 3: angM3 += dir * step; break;
                    case 4: angM4 += dir * step; break;
                    case 5: angM5 += dir * step; break;
                    case 6: angGrip += dir * 2.5f; break;
                }
                // Limite
                if(angM1 < 45.0f) angM1 = 45.0f;
                if(angM1 > 160.0f) angM1 = 160.0f;
                if(angM2 < 10.0f) angM2 = 10.0f;
                if(angM2 > 170.0f) angM2 = 170.0f;
                if(angM3 < 10.0f) angM3 = 10.0f;
                if(angM3 > 170.0f) angM3 = 170.0f;
                if(angM4 < 80.0f) angM4 = 80.0f;
                if(angM4 > 170.0f) angM4 = 170.0f;
                if(angM5 < 10.0f) angM5 = 10.0f;
                if(angM5 > 170.0f) angM5 = 170.0f;
                if(angGrip < 0.0f) angGrip = 0.0f;
                if(angGrip > 90.0f) angGrip = 90.0f;

                osMutexRelease(xMutex1Handle);

                lastMove = HAL_GetTick();
            }

            // 3. RECORD
            if (HAL_GPIO_ReadPin(BTN_RECORD_PORT, BTN_RECORD_PIN) == GPIO_PIN_RESET) {
                uint32_t start = HAL_GetTick();
                osDelay(50);

                while(HAL_GPIO_ReadPin(BTN_RECORD_PORT, BTN_RECORD_PIN) == GPIO_PIN_RESET) {
                    if(HAL_GetTick() - start > 3000) break;
                    osDelay(10);
                }

                if (HAL_GetTick() - start > 2000) {
                    pointCount = 0;
                    LCD_SetCursor(0,0); LCD_Print("POINTS CLEARED  "); osDelay(1000);
                } else {
                    if (pointCount < MAX_POINTS) {
                        savedPoints[pointCount].m1 = angM1;
                        savedPoints[pointCount].m2 = angM2;
                        savedPoints[pointCount].m3 = angM3;
                        savedPoints[pointCount].m4 = angM4;
                        savedPoints[pointCount].m5 = angM5;
                        savedPoints[pointCount].grip = angGrip;
                        pointCount++;
                    }
                }
            }

            // 4. GO HOME
            if (HAL_GPIO_ReadPin(BTN_HOME_PORT, BTN_HOME_PIN) == GPIO_PIN_RESET) {
                LCD_SetCursor(0, 0);
                LCD_Print("GOING HOME...   ");

                RobotPose homePose;
                homePose.m1 = HOME_M1; homePose.m2 = HOME_M2; homePose.m3 = HOME_M3;
                homePose.m4 = HOME_M4; homePose.m5 = HOME_M5; homePose.grip = HOME_GRIP;

                MoveSynchronized(homePose, 10);

                uint32_t wStart = HAL_GetTick();
                while(HAL_GPIO_ReadPin(BTN_HOME_PORT, BTN_HOME_PIN) == GPIO_PIN_RESET) {
                     if(HAL_GetTick() - wStart > 2000) break;
                     osDelay(10);
                }

                LCD_SetCursor(0, 0);
                LCD_Print("HOME DONE       ");
                osDelay(500);
            }

            // 5. START PLAY
            if (pointCount > 1 && HAL_GPIO_ReadPin(BTN_PLAY_PORT, BTN_PLAY_PIN) == GPIO_PIN_RESET) {
                 uint32_t wStart = HAL_GetTick();
                 while(HAL_GPIO_ReadPin(BTN_PLAY_PORT, BTN_PLAY_PIN) == GPIO_PIN_RESET) {
                     if(HAL_GetTick() - wStart > 2000) break;
                     osDelay(10);
                 }
                 currentMode = MODE_PLAY;
                 isPlaying = 1;
            }
        }

        // B. MOD PLAY
        else if (isPlaying) {
            // DUS
            for(int i=0; i<pointCount; i++) {
                if (MoveSynchronized(savedPoints[i], 10) != 0) {
                    isPlaying = 0; currentMode = MODE_MANUAL; osDelay(500); break;
                }
            }
            // INTORS
            if (isPlaying) {
                if (selectedPlaybackMode == PLAY_RETRACE) {
                    for(int i=pointCount-2; i>=0; i--) {
                        if (MoveSynchronized(savedPoints[i], 10) != 0) {
                            isPlaying = 0; currentMode = MODE_MANUAL; osDelay(500); break;
                        }
                    }
                } else {
                    // FAST MODE
                    if (MoveSynchronized(savedPoints[0], 10) != 0) {
                        isPlaying = 0; currentMode = MODE_MANUAL; osDelay(500);
                    }
                }
            }
        }
    }
    osDelay(10);
  }
  /* USER CODE END StartTask_LogicFcn */
}

/* USER CODE BEGIN Header_StartTask_HMIFcn */
/**
* @brief Function implementing the StartTask_HMI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_HMIFcn */
void StartTask_HMIFcn(void *argument)
{
  /* USER CODE BEGIN StartTask_HMIFcn */
	osDelay(1000);
	LCD_Init();
	LCD_Print("READY");
	osDelay(1000);
	LCD_Clear();
	char buf[32];
	uint32_t lastRef = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_GetTick() - lastRef > 10000) {
	  	        LCD_SendCommand(0x28);
	  	        LCD_SendCommand(0x0C);
	  	        LCD_SendCommand(0x06);
	  	        lastRef = HAL_GetTick();
	  	    }

	  	    if (isEmergency) {
	  	        LCD_SetCursor(0,0); LCD_Print("!! E-STOP !!   ");
	  	    } else {
	  	        LCD_SetCursor(0,0);
	  	        if (isPlaying) {
	  	             LCD_Print("RUNNING...      ");
	  	        } else {
	  	            char *modeStr = (selectedPlaybackMode == PLAY_RETRACE) ? "RET" : "FST";
	  	            sprintf(buf, "P:%d MODE:%s    ", pointCount, modeStr);
	  	            LCD_Print(buf);
	  	        }

	  	        LCD_SetCursor(1,0);
	  	        float val = 0;
	  	        switch(selectedMotor) {
	  	            case 1: val = angM1; break;
	  	            case 2: val = angM2; break;
	  	            case 3: val = angM3; break;
	  	            case 4: val = angM4; break;
	  	            case 5: val = angM5; break;
	  	            case 6: val = angGrip; break;
	  	        }

	  	        sprintf(buf, "M%d: %d deg      ", selectedMotor, (int)val);
	  	        LCD_Print(buf);
	  	    }
	  	    osDelay(150);
	  	  }
  /* USER CODE END StartTask_HMIFcn */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
