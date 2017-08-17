/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "../../CAN_ID.h"
#include "nodeConf.h"
#include "serial.h"
#include "can.h"
#include "can2.h"
// #include "Can_Processor.h"
#include "nodeMiscHelpers.h"
// #include "thermistor.h"
// #include "ts_lib.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId Motor_CtrlHandle;
osThreadId Can_ProcessorHandle;
osThreadId HousekeepingHandle;
osThreadId Can2_ProcessorHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osMessageQId MotCanTxQHandle;
osMessageQId motCanRxQHandle;
osTimerId WWDGTmrHandle;
osTimerId HBTmrHandle;
osMutexId swMtxHandle;
osMutexId uartMtxHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint16_t adcBuffer[8];
//TODO flaming can dumpster
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_WWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN2_Init(void);
void doMotCtrl(void const * argument);
void doProcessCan(void const * argument);
void doHousekeeping(void const * argument);
void doProcessorMotCan(void const * argument);
void TmrKickDog(void const * argument);
void TmrSendHB(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef * hcan);
void HAL_GPIO_EXTI_Callback(uint16_t pinNum);
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg);

void EM_Init();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	selfStatusWord = INIT;
	#define DISABLE_RT
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_WWDG_Init();
  MX_ADC1_Init();
  MX_CAN2_Init();

  /* USER CODE BEGIN 2 */

	Serial2_begin();

	bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
	// bxCan_addMaskedFilterStd(p2pOffset,0xFF0,0);

//	bxCan2_begin(&hcan2, &motCanRxQHandle, &MotCanTxQHandle);
	// bxCan2_addMaskedFilterStd(0,0,0);
	// bxCan2_addMaskedFilterExt(0,0,0);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of swMtx */
  osMutexDef(swMtx);
  swMtxHandle = osMutexCreate(osMutex(swMtx));

  /* definition and creation of uartMtx */
  osMutexDef(uartMtx);
  uartMtxHandle = osMutexCreate(osMutex(uartMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of WWDGTmr */
  osTimerDef(WWDGTmr, TmrKickDog);
  WWDGTmrHandle = osTimerCreate(osTimer(WWDGTmr), osTimerPeriodic, NULL);

  /* definition and creation of HBTmr */
  osTimerDef(HBTmr, TmrSendHB);
  HBTmrHandle = osTimerCreate(osTimer(HBTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(WWDGTmrHandle, WD_Interval);
  // osTimerStart(HBTmrHandle, HB_Interval);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Motor_Ctrl */
  osThreadDef(Motor_Ctrl, doMotCtrl, osPriorityHigh, 0, 512);
  Motor_CtrlHandle = osThreadCreate(osThread(Motor_Ctrl), NULL);

  /* definition and creation of Can_Processor */
  osThreadDef(Can_Processor, doProcessCan, osPriorityAboveNormal, 0, 512);
  Can_ProcessorHandle = osThreadCreate(osThread(Can_Processor), NULL);

  /* definition and creation of Housekeeping */
  osThreadDef(Housekeeping, doHousekeeping, osPriorityBelowNormal, 0, 512);
  HousekeepingHandle = osThreadCreate(osThread(Housekeeping), NULL);

  /* definition and creation of Can2_Processor */
  osThreadDef(Can2_Processor, doProcessorMotCan, osPriorityNormal, 0, 512);
  Can2_ProcessorHandle = osThreadCreate(osThread(Can2_Processor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxQ */
  osMessageQDef(mainCanTxQ, 32, Can_frame_t);
  mainCanTxQHandle = osMessageCreate(osMessageQ(mainCanTxQ), NULL);

  /* definition and creation of mainCanRxQ */
  osMessageQDef(mainCanRxQ, 32, Can_frame_t);
  mainCanRxQHandle = osMessageCreate(osMessageQ(mainCanRxQ), NULL);

  /* definition and creation of MotCanTxQ */
  osMessageQDef(MotCanTxQ, 32, Can_frame_t);
  MotCanTxQHandle = osMessageCreate(osMessageQ(MotCanTxQ), NULL);

  /* definition and creation of motCanRxQ */
  osMessageQDef(motCanRxQ, 32, Can_frame_t);
  motCanRxQHandle = osMessageCreate(osMessageQ(motCanRxQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_13TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = ENABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_13TQ;
  hcan2.Init.BS2 = CAN_BS2_2TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOT_ON_Pin */
  GPIO_InitStruct.Pin = MOT_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MOT_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC4 PC5 
                           PC6 PC7 PC8 PC9 
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA6 PA7 
                           PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB13 PB3 PB4 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_DIR_Pin DRV_MODE_Pin */
  GPIO_InitStruct.Pin = DRV_DIR_Pin|DRV_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if (hcan == &hcan1)
		CAN1_TxCpltCallback(hcan);
	if (hcan == &hcan2)
		CAN1_TxCpltCallback(hcan);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if (hcan == &hcan1)
		CAN1_RxCpltCallback(hcan);
	if (hcan == &hcan2)
		CAN1_RxCpltCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	if (hcan == &hcan1)
		CAN1_ErrorCallback(hcan);
	if (hcan == &hcan2)
		CAN1_ErrorCallback(hcan);
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg){
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_ADC_Stop_DMA(hadc);
	HAL_ADC_Start_DMA(hadc, (uint32_t*)adcBuffer, 2); //dw about ptr types. NEVER dma more than sequenced!
}

/* USER CODE END 4 */

/* doMotCtrl function */
void doMotCtrl(void const * argument)
{

  /* USER CODE BEGIN 5 */
	#define MOT_BASE_ID		0x10
	#define MOT_DRV_CMD_ID	MOT_BASE_ID + 0x01
	#define MOT_PWR_CMD_ID	MOT_BASE_ID + 0x02
	#define MOT_RST_CMD_ID	MOT_BASE_ID + 0x03
	#define POS_100			80
	#define POS_0			20
	#define MAX_RPM			700

	Can_frame_t newFrame;
	static uint16_t accel;
	static uint16_t regen;

	uint32_t val_100 = 0xfff;
	uint32_t map_100 = val_100*POS_100/100;
	uint32_t map_0 = val_100*POS_0/100;
	uint32_t map_range = map_100-map_0;
	uint8_t lastMotOn = 0;
	uint8_t lastDrvDir = 0;
	uint8_t lastDrvMode = 0;
	uint8_t motOn, drvDir, drvMode;

	newFrame.dlc = 8;
	newFrame.isExt = 0;
	newFrame.isRemote = 0;
	float* frameData = (float*)newFrame.Data;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 2);

	for(;;){
		osDelay(50);
		motOn = HAL_GPIO_ReadPin(MOT_ON_GPIO_Port, MOT_ON_Pin);
		//process motOn state
		if(!lastMotOn && motOn || lastMotOn && !motOn){
			newFrame.id = MOT_RST_CMD_ID;
			bxCan_sendFrame(&newFrame);
		}else if(motOn){
			newFrame.id = MOT_DRV_CMD_ID;
			drvDir = HAL_GPIO_ReadPin(DRV_DIR_GPIO_Port, DRV_DIR_Pin);
			drvMode = HAL_GPIO_ReadPin(DRV_MODE_GPIO_Port, DRV_MODE_Pin);
			accel = adcBuffer[0];
			regen = adcBuffer[1];
			float temp;
			if(accel< map_0){
				temp = 0.0f;
			}else if(accel<= map_100){
				temp = (float)(accel - map_0)/(float)map_range;
			}else{
				temp = 1.0f;
			}
			if(drvMode){ //speed drive
				frameData[0] = 1.0f;
                                temp*=MAX_RPM;
                                for(int i=0; i<4; i++){
                                  ((uint32_t*)frameData)[1] &= ~(0xff<<(8*i));
                                  ((uint32_t*)frameData)[1] |= *((uint32_t*)&temp)&(0xff<<(8*i));
                                }
//				frameData[1] = temp * MAX_RPM;
			}else{ //current drive
                          for(int i=0; i<4; i++){
                            ((uint32_t*)frameData)[0] &= ~(0xff<<(8*i));
                            ((uint32_t*)frameData)[0] |= *((uint32_t*)&temp)&(0xff<<(8*i));
                          }
//				frameData[0] = temp;
				frameData[1] = 20000.0f;
			}
			if(drvDir){
                          for(int i=0; i<4; i++){
                            *((uint32_t*)&temp) &= ~(0xff<<(8*i));
                            *((uint32_t*)&temp) |= ((uint32_t*)frameData)[1]&(0xff<<(8*i));
                          }
                          temp *=  -1;
                          for(int i=0; i<4; i++){
                            ((uint32_t*)frameData)[1] &= ~(0xff<<(8*i));
                            ((uint32_t*)frameData)[1] |= *((uint32_t*)&temp)&(0xff<<(8*i));
                          }
                        }
			lastDrvDir = drvDir;
			lastDrvMode = drvMode;
			bxCan_sendFrame(&newFrame);
		}
		lastMotOn = motOn;
	}
  /* USER CODE END 5 */ 
}

/* doProcessCan function */
void doProcessCan(void const * argument)
{
  /* USER CODE BEGIN doProcessCan */
  // uint32_t val_100 = 0xfff;
  // uint32_t val_1 = val_100 / 100;
  // uint32_t map_100 = val_1*80;
  // uint32_t map_0 = val_1*20;
  // uint32_t map_range = map_100-map_0;
  // int32_t map_val = 0;
  // int32_t accelerator1;
  //
  // an_frame_t newFrame;
  // ewFrame.id = 0x04880120;
  // ewFrame.isExt = 1;
  // ewFrame.isRemote = 0;
  //       newFrame.dlc = 8;
  //       newFrame.Data[0] = 0x01;
  //       newFrame.Data[1] = 0x00;
  //       newFrame.Data[2] = 0x04;
  //       newFrame.Data[3] = 0x40;
  //       newFrame.Data[4] = 0x00;
  //       newFrame.Data[5] = 0x00;
  //       newFrame.Data[6] = 0x14;
  // ewFrame.Data[7] = 0x14;
  //
  // for(int i=0; i<(sizeof(ecuFrames)/sizeof(Can_frame_t)); i++){
  //  	bxCan_sendFrame(&(ecuFrames[i]));
  // }
  /* Infinite loop */
  for(;;)
  {
	osDelay(10000);
	// accelerator1 = accelerator;
	// if(accelerator1>map_100) accelerator1 = map_100;
	// if((accelerator1<=map_100) && (accelerator1>map_0)){
	// 	newFrame.Data[2] = 0x00;
	// 	newFrame.Data[3] = 0x44;
	// 	map_val = (accelerator1 - map_0) * 0xff / map_range;
	// 	newFrame.Data[1] = map_val&0xff;
	// }else if((accelerator1 > map_0/4) && (accelerator1 <= map_0)){
	// 	newFrame.Data[1] = 0x00;
	// 	newFrame.Data[2] = 0x04;
	// 	newFrame.Data[3] = 0x44;
	// }else if(accelerator1 <= map_0/4){
	// 	newFrame.Data[1] = 0x00;
	// 	newFrame.Data[2] = 0x04;
	// 	newFrame.Data[3] = 0x40;
	// }
    //     bxCan_sendFrame(&newFrame);
  }
  /* USER CODE END doProcessCan */
}

/* doHousekeeping function */
void doHousekeeping(void const * argument)
{
  /* USER CODE BEGIN doHousekeeping */
	static int bamboozle;
	bamboozle = 0;
	static int bamboozle2;
	bamboozle2 = 0;
	/* Infinite loop */
	for(;;){
		if(hcan1.State == HAL_CAN_STATE_READY || hcan1.State == HAL_CAN_STATE_BUSY_TX || \
		hcan1.State == HAL_CAN_STATE_TIMEOUT || hcan1.State == HAL_CAN_STATE_ERROR){
			bamboozle++;
		}else{
			bamboozle = 0;
		}
		if(bamboozle > 8){
			HAL_CAN_Receive_IT(&hcan1, 0);
		}
		if(bamboozle > 12){
			NVIC_SystemReset();
		}

		if(hcan2.State == HAL_CAN_STATE_READY || hcan2.State == HAL_CAN_STATE_BUSY_TX || \
		hcan2.State == HAL_CAN_STATE_TIMEOUT || hcan2.State == HAL_CAN_STATE_ERROR){
			bamboozle2++;
		}else{
			bamboozle2 = 0;
		}
		if(bamboozle2 > 8){
			HAL_CAN_Receive_IT(&hcan2, 0);
		}
		if(bamboozle2 > 12){
			NVIC_SystemReset();
		}

		osDelay(17);    //nice prime number
	}
  /* USER CODE END doHousekeeping */
}

/* doProcessorMotCan function */
void doProcessorMotCan(void const * argument)
{
  /* USER CODE BEGIN doProcessorMotCan */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END doProcessorMotCan */
}

/* TmrKickDog function */
void TmrKickDog(void const * argument)
{
  /* USER CODE BEGIN TmrKickDog */
  taskENTER_CRITICAL();
  HAL_WWDG_Refresh(&hwwdg);
  taskEXIT_CRITICAL();
  /* USER CODE END TmrKickDog */
}

/* TmrSendHB function */
void TmrSendHB(void const * argument)
{
  /* USER CODE BEGIN TmrSendHB */
  static Can_frame_t newFrame;

//	newFrame.isExt = 0;
//	newFrame.isRemote = 0;
  // ^ is initialized as 0

  if(getSelfState() == ACTIVE){
	  // Assemble new heartbeat frame
	  newFrame.id = selfNodeID + swOffset;
	  newFrame.dlc = CAN_HB_DLC;
	  for(int i=0; i<4; i++){
		  newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	  }
	  bxCan_sendFrame(&newFrame);
	  #ifdef DEBUG
		  static uint8_t hbmsg[] = "Heartbeat issued\n";
		  Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
	  #endif
  }
  else if (getSelfState() == INIT){
	  // Assemble new addition request (firmware version) frame
	  newFrame.id = selfNodeID + fwOffset;
	  newFrame.dlc = CAN_FW_DLC;
	  for(int i=0; i<4; i++){
		  newFrame.Data[3-i] = (firmwareString >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	  }
	  bxCan_sendFrame(&newFrame);
	  #ifdef DEBUG
		  static uint8_t hbmsg[] = "Init handshake issued\n";
		  Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
	  #endif
  }
  // No heartbeats sent in other states
  /* USER CODE END TmrSendHB */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
