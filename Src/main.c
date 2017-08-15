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
// #include "can2.h"
// #include "Can_Processor.h"
// #include "mcp3909.h"
#include "nodeMiscHelpers.h"
// #include "thermistor.h"
#include "ts_lib.h"
// #include "psb0cal.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId PPTPollHandle;
osThreadId Can_ProcessorHandle;
osThreadId TempTaskHandle;
osThreadId HousekeepingHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osTimerId WWDGTmrHandle;
osTimerId HBTmrHandle;
osMutexId swMtxHandle;
osMutexId uartMtxHandle;
osSemaphoreId mcp3909_DRHandle;
osSemaphoreId mcp3909_RXHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t init_Done = 0;

/*
typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t Data[8];
  uint8_t isExt; //1 or 0
  uint8_t isRemote;
  int filterNum;
}Can_frame_t;
*/


Can_frame_t ecuFrames[] = {
	//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":2,"id":"04089520","data":["01","01"]},
	{0x04089520, 2, {0x01,0x01,00,00,00,00,00,00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":3,"id":"04048225","data":["01","01","00"]},
	{0x04048225, 3, {0x01,0x01,0x00,00,00,00,00,00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","06","50","53","38","30","30","30"]},
	{0x04080120, 8, {0x01,0x06,0x50,0x53,0x38,0x30,0x30,0x30}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":2,"id":"04089520","data":["01","01"]},
	{0x04089520, 2, {0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":3,"id":"04048225","data":["01","01","05"]},
	{0x04048225, 3, {0x01,0x01,0x05,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","00","00","01","00","00","02"]},
	{0x04080120, 8, {0x01,0x02,0x00,0x00,0x01,0x00,0x00,0x02}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","04","00","40","20","03","00"]},
	{0x04080120, 8, {0x01,0x02,0x04,0x00,0x040,0x20,0x03,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","08","00","1C","46","64","64"]},
	{0x04080120, 8, {0x01,0x02,0x08,0x00,0x1C,0x46,0x64,0x64}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","0C","00","64","64","64","02"]},
	{0x04080120, 8, {0x01,0x02,0x0C,0x00,0x64,0x64,0x64,0x02}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","10","00","2F","20","14","01"]},
	{0x04080120, 8, {0x01, 0x02, 0x10, 0x00, 0x2F, 0x20, 0x14, 0x01}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","14","00","0D","08","11","FF"]},
	{0x04080120, 8, {0x01,0x02,0x14,0x00,0x0D,0x08,0x11,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","18","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x18,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","1C","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x1C,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","20","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x20,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","24","00","1E","00","50","00"]},
	{0x04080120, 8, {0x01,0x02,0x24,0x00,0x1E,0x00,0x50,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","28","00","32","00","19","00"]},
	{0x04080120, 8, {0x01,0x02,0x28,0x00,0x32,0x00,0x19,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","2C","00","0A","00","0F","00"]},
	{0x04080120, 8, {0x01,0x02,0x2C,0x00,0x0A,0x00,0x0F,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","30","00","0F","00","50","00"]},
	{0x04080120, 8, {0x01,0x02,0x30,0x00,0x0F,0x00,0x50,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","34","00","41","00","64","00"]},
	{0x04080120, 8, {0x01,0x02,0x34,0x00,0x41,0x00,0x64,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","38","00","3C","00","64","00"]},
	{0x04080120, 8, {0x01,0x02,0x38,0x00,0x3C,0x00,0x64,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","3C","00","A0","00","64","00"]},
	{0x04080120, 8, {0x01,0x02,0x3C,0x00,0xA0,0x00,0x64,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","40","00","FF","FF","16","01"]},
	{0x04080120, 8, {0x01,0x02,0x40,0x00,0xFF,0xFF,0x16,0x01}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","44","00","04","01","50","00"]},
	{0x04080120, 8, {0x01,0x02,0x44,0x00,0x04,0x01,0x50,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","48","00","16","01","50","00"]},
	{0x04080120, 8, {0x01,0x02,0x48,0x00,0x16,0x01,0x50,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","4C","00","00","00","1E","00"]},
	{0x04080120, 8, {0x01,0x02,0x4C,0x00,0x00,0x00,0x1E,0x00}, 1, 0, 0},

//{"timestamp":"01:53:00","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","50","00","0F","00","54","01"]},
	{0x04080120, 8, {0x01,0x02,0x50,0x00,0x0F,0x00,0x54,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","54","00","58","02","5A","00"]},
	{0x04080120, 8, {0x01,0x02,0x54,0x00,0x58,0x02,0x5A,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","58","00","2A","01","02","00"]},
	{0x04080120, 8, {0x01,0x02,0x58,0x00,0x2A,0x01,0x02,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","5C","00","50","00","16","01"]},
	{0x04080120, 8, {0x01,0x02,0x5C,0x00,0x50,0x00,0x16,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","60","00","90","01","90","01"]},
	{0x04080120, 8, {0x01,0x02,0x60,0x00,0x90,0x01,0x90,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","64","00","10","00","AF","00"]},
	{0x04080120, 8, {0x01,0x02,0x64,0x00,0x10,0x00,0xAF,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","68","00","20","03","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x68,0x00,0x20,0x03,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","6C","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x6C,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","70","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x70,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","74","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x74,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","78","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x78,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","7C","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x7C,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","80","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x80,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","84","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x84,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","88","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x88,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","8C","00","FF","FF","FF","00"]},
	{0x04080120, 8, {0x01,0x02,0x8C,0x00,0xFF,0xFF,0xFF,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","90","00","10","01","21","01"]},
	{0x04080120, 8, {0x01,0x02,0x90,0x00,0x10,0x01,0x21,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","94","00","32","01","43","01"]},
	{0x04080120, 8, {0x01,0x02,0x94,0x00,0x32,0x01,0x43,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","98","00","54","01","65","01"]},
	{0x04080120, 8, {0x01,0x02,0x98,0x00,0x54,0x01,0x65,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","9C","00","76","01","87","01"]},
	{0x04080120, 8, {0x01,0x02,0x9C,0x00,0x76,0x01,0x87,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","A0","00","98","01","A9","01"]},
	{0x04080120, 8, {0x01,0x02,0xA0,0x00,0x98,0x01,0xA9,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","A4","00","BA","01","CB","01"]},
	{0x04080120, 8, {0x01,0x02,0xA4,0x00,0xBA,0x01,0xCB,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","A8","00","DC","01","ED","01"]},
	{0x04080120, 8, {0x01,0x02,0xA8,0x00,0xDC,0x01,0xED,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","AC","00","FF","01","02","03"]},
	{0x04080120, 8, {0x01,0x02,0xAC,0x00,0xFF,0x01,0x02,0x03}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","B0","00","D5","02","A4","02"]},
	{0x04080120, 8, {0x01,0x02,0xB0,0x00,0xD5,0x02,0xA4,0x02}, 1, 0, 0},
//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","B4","00","74","02","44","02"]},
	{0x04080120, 8, {0x01,0x02,0xB4,0x00,0x74,0x02,0x44,0x02}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","B8","00","13","02","E3","01"]},
	{0x04080120, 8, {0x01,0x02,0xB8,0x00,0x13,0x02,0xE3,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","BC","00","B3","01","82","01"]},
	{0x04080120, 8, {0x01,0x02,0xBC,0x00,0xB3,0x01,0x82,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","C0","00","52","01","22","01"]},
	{0x04080120, 8, {0x01,0x02,0xC0,0x00,0x52,0x01,0x22,0x01}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","C4","00","F1","00","C1","00"]},
	{0x04080120, 8, {0x01,0x02,0xC4,0x00,0xF1,0x00,0xC1,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","C8","00","91","00","61","00"]},
	{0x04080120, 8, {0x01,0x02,0xC8,0x00,0x91,0x00,0x61,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","CC","00","30","00","9B","00"]},
	{0x04080120, 8, {0x01,0x02,0xCC,0x00,0x30,0x00,0x9B,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","D0","00","A3","00","A8","00"]},
	{0x04080120, 8, {0x01,0x02,0xD0,0x00,0xA3,0x00,0xA8,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","D4","00","AF","00","B5","00"]},
	{0x04080120, 8, {0x01,0x02,0xD4,0x00,0xAF,0x00,0xB5,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","D8","00","BC","00","C1","00"]},
	{0x04080120, 8, {0x01,0x02,0xD8,0x00,0xBC,0x00,0xC1,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","DC","00","C9","00","CE","00"]},
	{0x04080120, 8, {0x01,0x02,0xDC,0x00,0xC9,0x00,0xCE,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","E0","00","D3","00","D8","00"]},
	{0x04080120, 8, {0x01,0x02,0xE0,0x00,0xD3,0x00,0xD8,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","E4","00","D8","00","D8","00"]},
	{0x04080120, 8, {0x01,0x02,0xE4,0x00,0xD8,0x00,0xD8,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","E8","00","D8","00","D8","00"]},
	{0x04080120, 8, {0x01,0x02,0xE8,0x00,0xD8,0x00,0xD8,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","EC","00","D8","00","68","89"]},
	{0x04080120, 8, {0x01,0x02,0xEC,0x00,0xD8,0x00,0x68,0x69}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","F0","00","09","00","03","00"]},
{0x04080120, 8, {0x01,0x02,0xF0,0x00,0x09,0x00,0x03,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","F4","00","00","00","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0xF4,0x00,0x00,0x00,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","F8","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0xF8,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","FC","00","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0xFC,0x00,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","18","01","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x18,0x01,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","48","01","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x48,0x01,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","90","01","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0x90,0x01,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04080120","data":["01","02","C0","01","FF","FF","FF","FF"]},
	{0x04080120, 8, {0x01,0x02,0xC0,0x01,0xFF,0xFF,0xFF,0xFF}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":2,"id":"04080120","data":["01","03"]},
	{0x04080120, 2, {0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":2,"id":"04089520","data":["01","01"]},
	{0x04089520, 2, {0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":3,"id":"04048225","data":["01","01","02"]},
	{0x04048225, 3, {0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":2,"id":"04080120","data":["01","04"]},
	{0x0408120, 2, {0x01,0x04,0x00,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:01","type":"frame","ide":true,"rtr":false,"dlc":2,"id":"04089520","data":["01","01"]},
	{0x04089520, 2, {0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00}, 1, 0, 0},

//{"timestamp":"01:53:02","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["00","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x00,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},

//{"timestamp":"01:53:02","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["02","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x02,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},

//{"timestamp":"01:53:02","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["02","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x02,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},

//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]}
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},

//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},
//{"timestamp":"01:53:04","type":"frame","ide":true,"rtr":false,"dlc":8,"id":"04880120","data":["01","00","04","40","00","00","14","14"]},
	{0x04880120, 8, {0x01,0x00,0x04,0x40,0x00,0x00,0x14,0x14}, 1, 0, 0},


};




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
void doPPTPoll(void const * argument);
void doProcessCan(void const * argument);
void doTempTask(void const * argument);
void doHousekeeping(void const * argument);
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

  /* USER CODE BEGIN 2 */
	__HAL_GPIO_EXTI_CLEAR_IT(DR1_Pin);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	__HAL_GPIO_EXTI_CLEAR_IT(DR1_Pin);

	init_Done = 1;

	Serial2_begin();

	bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
	// bxCan_addMaskedFilterStd(p2pOffset,0xFF0,0);
	//
	// bxCan2_begin(&hcan2, &can2RxQHandle, &can2TxQHandle);
	// bxCan2_addMaskedFilterStd(0,0,0);
	// bxCan2_addMaskedFilterExt(0,0,0);

	#ifndef DISABLE_TMT
	    Temp_begin(&hadc1);
	#endif

	#ifndef DISABLE_RT
      EM_Init();
      HAL_WWDG_Refresh(&hwwdg);
    #endif
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

  /* Create the semaphores(s) */
  /* definition and creation of mcp3909_DR */
  osSemaphoreDef(mcp3909_DR);
  mcp3909_DRHandle = osSemaphoreCreate(osSemaphore(mcp3909_DR), 1);

  /* definition and creation of mcp3909_RX */
  osSemaphoreDef(mcp3909_RX);
  mcp3909_RXHandle = osSemaphoreCreate(osSemaphore(mcp3909_RX), 1);

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
  /* definition and creation of PPTPoll */
  osThreadDef(PPTPoll, doPPTPoll, osPriorityNormal, 0, 512);
  PPTPollHandle = osThreadCreate(osThread(PPTPoll), NULL);

  /* definition and creation of Can_Processor */
  osThreadDef(Can_Processor, doProcessCan, osPriorityAboveNormal, 0, 512);
  Can_ProcessorHandle = osThreadCreate(osThread(Can_Processor), NULL);

  /* definition and creation of TempTask */
  osThreadDef(TempTask, doTempTask, osPriorityRealtime, 0, 512);
  TempTaskHandle = osThreadCreate(osThread(TempTask), NULL);

  /* definition and creation of Housekeeping */
  osThreadDef(Housekeeping, doHousekeeping, osPriorityBelowNormal, 0, 512);
  HousekeepingHandle = osThreadCreate(osThread(Housekeeping), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxQ */
  osMessageQDef(mainCanTxQ, 256, Can_frame_t);
  mainCanTxQHandle = osMessageCreate(osMessageQ(mainCanTxQ), NULL);

  /* definition and creation of mainCanRxQ */
  osMessageQDef(mainCanRxQ, 64, Can_frame_t);
  mainCanRxQHandle = osMessageCreate(osMessageQ(mainCanRxQ), NULL);

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
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 3;
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
     PC3   ------> SPI2_MOSI
     PA9   ------> SPI2_SCK
     PB5   ------> CAN2_RX
     PB6   ------> CAN2_TX
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
  HAL_GPIO_WritePin(MCP1_CS_GPIO_Port, MCP1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FAN_Pin|MCP2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN1_Pin|S2_Pin|S1_Pin|S3_Pin
                          |S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC5
                           PC6 PC7 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP1_CS_Pin */
  GPIO_InitStruct.Pin = MCP1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCP1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin EN2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_Pin */
  GPIO_InitStruct.Pin = FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN1_Pin S2_Pin S1_Pin S3_Pin
                           S0_Pin */
  GPIO_InitStruct.Pin = EN1_Pin|S2_Pin|S1_Pin|S3_Pin
                          |S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB3 PB4
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DR2_Pin DR1_Pin */
  GPIO_InitStruct.Pin = DR2_Pin|DR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP2_CS_Pin */
  GPIO_InitStruct.Pin = MCP2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCP2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if (hcan == &hcan1)
		CAN1_TxCpltCallback(hcan);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if (hcan == &hcan1)
		CAN1_RxCpltCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	if (hcan == &hcan1)
		CAN1_ErrorCallback(hcan);
}

void HAL_GPIO_EXTI_Callback(uint16_t pinNum){
	if(init_Done){
		if(pinNum == DR1_Pin){
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			xSemaphoreGiveFromISR(mcp3909_DRHandle, NULL);
		}
	}
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg){
  int i;
  for(i = 0; i < 100; i++);
}


/* USER CODE END 4 */

/* doPPTPoll function */
void doPPTPoll(void const * argument)
{

  /* USER CODE BEGIN 5 */
  static Can_frame_t newFrame;
  /* Infinite loop */
	for(;;){
		osDelay(1000);
	}
  /* USER CODE END 5 */
}

/* doProcessCan function */
void doProcessCan(void const * argument)
{
  /* USER CODE BEGIN doProcessCan */
  uint32_t val_100 = 0xfff;
  uint32_t val_1 = val_100 / 100;
  uint32_t map_100 = val_1*80;
  uint32_t map_0 = val_1*20;
  uint32_t map_range = map_100-map_0;
	uint32_t map_val = 0;
	uint32_t accelerator1;

	Can_frame_t newFrame;
	newFrame.id = 0x04880120;
	newFrame.isExt = 1;
	newFrame.isRemote = 0;
        newFrame.dlc = 8;
        newFrame.Data[0] = 0x01;
        newFrame.Data[1] = 0x00;
        newFrame.Data[2] = 0x04;
        newFrame.Data[3] = 0x40;
        newFrame.Data[4] = 0x00;
        newFrame.Data[5] = 0x00;
        newFrame.Data[6] = 0x14;
	newFrame.Data[7] = 0x14;

  for(int i=0; i<(sizeof(ecuFrames)/sizeof(Can_frame_t)); i++){
	  	bxCan_sendFrame(&(ecuFrames[i]));
  }
  /* Infinite loop */
  for(;;)
  {
	osDelay(35);
	accelerator1 = accelerator;
	if(accelerator1>map_100) accelerator1 = map_100;
	if((accelerator1<=map_100) && (accelerator1>map_0)){
		newFrame.Data[2] = 0x00;
		newFrame.Data[3] = 0x44;
		map_val = (accelerator1 - map_0) * 0xff / map_range;
		newFrame.Data[1] = map_val&0xff;
	}else if((accelerator1 > map_0/4) && (accelerator1 <= map_0)){
		newFrame.Data[1] = 0x00;
		newFrame.Data[2] = 0x04;
		newFrame.Data[3] = 0x44;
	}else if(accelerator1 <= map_0/4){
		newFrame.Data[1] = 0x00;
		newFrame.Data[2] = 0x04;
		newFrame.Data[3] = 0x40;
	}
        bxCan_sendFrame(&newFrame);
  }
  /* USER CODE END doProcessCan */
}

/* doTempTask function */
void doTempTask(void const * argument)
{
  /* USER CODE BEGIN doTempTask */
  // #ifndef DISABLE_TMT
  //
  // #ifndef DISABLE_CAN
  // 	static Can_frame_t newFrame;
  // 	newFrame.dlc = 8;
  // 	newFrame.isRemote = 0;
  // 	newFrame.isExt = 0;
  // #else
  // 	osDelay(10);
  // #endif
  //
  //   /* Infinite loop */
  //   for(;;)
  //   {
  // #ifndef DISABLE_CAN
  // 	  if((selfStatusWord & 0x07) == ACTIVE){
  // #endif
  //         int32_t microCelcius;
  // 		  for(int i=0; 2*i<TEMP_CHANNELS; i++){
  // 			  for(int j=0; j<2; j++){
  // 		  microCelcius = getMicroCelcius(2*i+j);
  // 				  resetReading(2*i+j);
  // #ifndef DISABLE_CAN
  // 				  *(int32_t*)(&(newFrame.Data[j*4])) = microCelcius;
  // #endif
  // 			  }
  // #ifndef DISABLE_CAN
  // 			  newFrame.id = adcTempOffset + i;
  // 			  bxCan_sendFrame(&newFrame);
  // #endif
  // 		  }
  // 		  osDelay(TMT_Interval);
  // #ifndef DISABLE_CAN
  // 	  }else{
  // 		  osDelay(1);
  // 	  }
  // #endif
  //   }
  //
  // #else
  //   for(;;){
  // 	  osDelay(1000);
  //   }
  // #endif
  for(;;){
	  osDelay(100);
  }
  /* USER CODE END doTempTask */
}

/* doHousekeeping function */
void doHousekeeping(void const * argument)
{
  /* USER CODE BEGIN doHousekeeping */
	static int bamboozle;
	bamboozle = 0;
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

		osDelay(17);    //nice prime number
	}
  /* USER CODE END doHousekeeping */
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
