/*************************************************************************************
*	@file			BSP.h
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************/
#ifndef  __BSP_H
#define  __BSP_H

/* Includes ------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "can.h"
#include "spi.h"
#include "adc.h"
#include "communication.h "
#include "Motor_USE_TIM.h"
#include "Motor_USE_CAN.h"
#include "minipc.h"
#include "Power_restriction.h"
#include "atom_imu.h"
#include "decode.h"
#include "SystemState.h"
/* Exported macro ------------------------------------------------------------*/

/* Exported types --------------------------------------------------------*/
extern volatile unsigned long long FreeRTOSRunTimeTicks;
/* Exported constants------------------------------------------------------------*/

/* Internal functions ------------------------------------------------------- */
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart);
void JY61_Frame(void);
void ConfigureTimerForRunTimeStats(void);
/* Exported functions ------------------------------------------------------- */

void BSP_Init(void);

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/







