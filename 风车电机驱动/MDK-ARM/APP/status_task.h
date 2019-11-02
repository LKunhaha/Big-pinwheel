/*************************************************************************************
*	@file			status_task.h
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************/
#ifndef __status_h
#define __status_h
/* Includes ------------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "SystemState.h"
#include "Power_restriction.h"
/* Exported macro ------------------------------------------------------------*/

/* Exported types --------------------------------------------------------*/

/* Exported constants------------------------------------------------------------*/

/* Internal functions ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Check_Task(void const * argument);
void Status_Task(void const * argument);
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/