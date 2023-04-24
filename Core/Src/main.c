/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "Lookup_Tables.h"
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
ADC_HandleTypeDef hadc1; //ADC1 handle structure, used to configure and control the ADC1 peripheral, and to initialize it
DMA_HandleTypeDef hdma_adc1; //DMA handle structure, used to configure and control the DMA channel used to transfer data from the ADC1 peripheral to memory

CAN_HandleTypeDef hcan; //CAN handle structure, used to configure and control the CAN peripheral, and to initialize it

TIM_HandleTypeDef htim2; //TIM2 handle structure, used to configure and control the TIM2 peripheral, and to initialize it
TIM_HandleTypeDef htim3; //TIM3 handle structure, used to configure and control the TIM3 peripheral, and to initialize it

osThreadId Read_SensorsHandle;//Thread handler, used to initialize the thread responsible for reading the sensors
osThreadId Read_LookupHandle; //Thread handler, used to initialize the thread responsible for reading the lookup tables
osThreadId Fire_IgnitionHandle;  //Thread handler, used to initialize the thread responsible for firing the ignition
osThreadId Fire_InjectionHandle; //Thread handler, used to initialize the thread responsible for firing the injection
osThreadId Send_CANHandle; //Thread handler, used to initialize the thread responsible for sending the CAN messages
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Read_Sensors_F(void const * argument); //function to read sensors, used by the Thread Read_Sensors
void Read_lookup_F(void const * argument); //function to read lookup tables, used by the Thread Read_Lookup
void Fire_Ignition_F(void const * argument); //function to fire the ignition, used by the Thread Fire_Ignition - not used in yet, firing is handled in timers
void Fire_Injection_F(void const * argument); //function to fire the injection, used by the Thread Fire_Injection - not used in yet, firing is handled in timers
void Send_CAN_F(void const * argument);//function to send can messages, used by the Thread Send_CAN, not yet implemented

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Data[5]; //variable to store the data read from the sensors (IAT, CLT, TPS, MAP, O2)
float IGN=0;//variable to store the ignition advance, read from lookup tables with the help of threads
float INJ=0;//variable to store the injection advance, read from lookup tables with the help of threads
//float VE=0
uint32_t current_time=0; //variable to store the current time
uint32_t elapsed_time=0; //variable to store the time elapsed since beginning of cycle
uint32_t last_time=0; //variable to store the time of the last interrupt on TIM2 (720 degrees of crankshaft)
uint32_t captured_value=0; //variable to store the number of cycles since beginning of program
bool fired = false;//variable to store if the ignition has been fired

uint32_t time_since_cycle = 0;//variable to store the time since the beginning of the cycle
uint16_t times_overflown = 0;//variable to store the number of times the timer has overflown
uint16_t cycles_to_IGN = 0;//variable to store the number of cycles until the ignition is fired
uint32_t IGN_Time = 0;//variable to store the time of the ignition
float RPM_val=0;//variable to store the RPM value

//defines for the sensors, Data is read through DMA from ADC1
#define IAT_val Data[0]
#define CLT_val Data[1]
#define TPS_val Data[2]
#define MAP_val Data[3]
#define O2_val  Data[4]

//TODO: make shorter, consider adding a flag and doing everything in main while loop
//function called every time the timer TIM2 overflows
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM2) { //if the timer is TIM2
	current_time=HAL_GetTick();//get the current time
	elapsed_time=current_time-last_time;//calculate the time elapsed since the last interrupt
	captured_value++;//increment the number of cycles since the beginning of the program, only for debuggin purposes
	RPM_val = (1 / (float)elapsed_time)*2*60*1000;//calculate the RPM value TODO: optimize, very crude
	last_time=current_time;//set the last time to the current time
  }
	fired = false;//set the fired flag to false
	time_since_cycle = 0;//set the time since the beginning of the cycle to 0
	times_overflown = 0;//set the number of times the timer 3 has overflown to 0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); //set the pin B6 to high, used to start the injection and will be set to low by the output compare register functionnality
	TIM3->CCR2=INJ*1000;//set the output compare register to the value of the injection timing using the value we got from the lookup table
	TIM3->CNT=0;//reset the counter to 0, so that the timer can start counting from 0deg of crankshaft
  /*
    explanation:
     TIM3 counts microseconds, so we need to multiply the value of injection timing (calculated in thread) by 1000 to get microseconds
     after the TIM3 gets to CCR2, it pulls the pin B6 low, which ends the injection
      TIM3->CNT=0; resets the counter to 0, so that the timer can start counting from 0deg of crankshaft
  */
	}

if(htim->Instance == TIM3)//if the timer is TIM3
{
	times_overflown++; 
	cycles_to_IGN--;//decrement the number of times the TIM3 overflows until the ignition is fired
	if(!cycles_to_IGN)//if the number of cycles until the ignition is fired is 0
		TIM3->CCR1 = IGN_time-times_overflown*65535;//set the output compare register to the value of the ignition timing using the value we got from the lookup table
}
}

//function called every time the timer TIM3 reaches the value of the output compare register
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && !fired)//check if the timer is TIM3, if the channel is 1 and if the ignition has not been fired yet
	{
		TIM3->CCR1 += 2000;//add 2000 microseconds to the output compare register, so that the ignition is turned off 2ms after it's turned on
		fired = true;//set the fired flag to true
    // the PIN is toggled by hardware
	}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, Data,5);//start the ADC1 in DMA mode, reading 5 values
  HAL_TIM_Base_Start_IT(&htim2);//start the timer TIM2 in interrupt mode
  /* USER CODE END 2 */

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
  /* definition and creation of Read_Sensors */
  osThreadDef(Read_Sensors, Read_Sensors_F, osPriorityNormal, 0, 128);
  Read_SensorsHandle = osThreadCreate(osThread(Read_Sensors), NULL);

  /* definition and creation of Read_Lookup */
  osThreadDef(Read_Lookup, Read_lookup_F, osPriorityNormal, 0, 128);
  Read_LookupHandle = osThreadCreate(osThread(Read_Lookup), NULL);

  /* definition and creation of Fire_Ignition */
  osThreadDef(Fire_Ignition, Fire_Ignition_F, osPriorityNormal, 0, 128);
  Fire_IgnitionHandle = osThreadCreate(osThread(Fire_Ignition), NULL);

  /* definition and creation of Fire_Injection */
  osThreadDef(Fire_Injection, Fire_Injection_F, osPriorityNormal, 0, 128);
  Fire_InjectionHandle = osThreadCreate(osThread(Fire_Injection), NULL);

  /* definition and creation of Send_CAN */
  osThreadDef(Send_CAN, Send_CAN_F, osPriorityLow, 0, 128);
  Send_CANHandle = osThreadCreate(osThread(Send_CAN), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 35;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : CAMSHAFT_Pin */
  GPIO_InitStruct.Pin = CAMSHAFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAMSHAFT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief this function interpolates it's in the name
 */
float Interpolate_2D(float T[N][N], const float *Var, const float *RPM, float x, float y){
    if (x < 300.0 || x > 7000.0 || y < 20.0 || y > 200.0) {
        return -1;
    }
    int i, j;
    for (i = 0; i < N && RPM[i] <= x; i++);
    for (j = 0; j < N && Var[j] <= y; j++);

    if (i == 0 || j == 0 || i == N || j == N) {
        return -1;
    }

    const float x1 = RPM[i-1], x2 = RPM[i];
    const float y1 = Var[j-1], y2 = Var[j];
    const float f11 = T[j-1][i-1], f12 = T[j-1][i], f21 = T[j][i-1], f22 = T[j][i];

    const float factor_1 = (y2 - y) / (y2 - y1);
    const float factor_2 = (y - y1) / (y2 - y1);
    const float one = (x2 - x) / (x2 - x1);
    const float two = (x - x1) / (x2 - x1);
    const float three = (x2 - x) / (x2 - x1);
    const float four = (x - x1) / (x2 - x1);

    return factor_1 * (f11 * one + f21 * two) + factor_2 * (f12 * three + f22 * four);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Read_Sensors_F */
/**
  * @brief  Function implementing the Read_Sensors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Read_Sensors_F */
void Read_Sensors_F(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Read_lookup_F */
/**
* @brief Function implementing the Read_Lookup thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_lookup_F */
void Read_lookup_F(void const * argument)
{
  /* USER CODE BEGIN Read_lookup_F */
  /* Infinite loop */
  for(;;)
  {
	 IGN= Interpolate_2D(Ignition_Timing,TBP,RPM,RPM_val,TPS_val);
	 INJ=Interpolate_2D(Injection_Timing,TBP,RPM,RPM_val,TPS_val);
	 IGN_Time = (360.0-IGN)/(6.0*RPM)*1000000; //calulate the time in us from the angle we got from the table
	 //VE=Interpolate_2D(VE_Table,MAP,RPM,RPM_val,MAP_val);
	 cycles_to_IGN = (uint32_t) IGN_Time/65535; //how many overflows to make the IGN pulse (since TIM3 only counts to 65535 and the IGN pulse is longer than that)
  /* USER CODE END Read_lookup_F */
  }
}
/* USER CODE BEGIN Header_Fire_Ignition_F */
/**
* @brief Function implementing the Fire_Ignition thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Fire_Ignition_F */
void Fire_Ignition_F(void const * argument)
{
  /* USER CODE BEGIN Fire_Ignition_F */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Fire_Ignition_F */
}

/* USER CODE BEGIN Header_Fire_Injection_F */
/**
* @brief Function implementing the Fire_Injection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Fire_Injection_F */
void Fire_Injection_F(void const * argument)
{
  /* USER CODE BEGIN Fire_Injection_F */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Fire_Injection_F */
}

/* USER CODE BEGIN Header_Send_CAN_F */
/**
* @brief Function implementing the Send_CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_CAN_F */
void Send_CAN_F(void const * argument)
{
  /* USER CODE BEGIN Send_CAN_F */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Send_CAN_F */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line){}
#endif /* USE_FULL_ASSERT */
