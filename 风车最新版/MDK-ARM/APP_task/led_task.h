/*************************************************************************************
*	@file			led_tesk.h
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************/
#ifndef __LED_TASK_H
#define __LED_TASK_H

/* Includes ------------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
/* Exported macro ------------------------------------------------------------*/

/* Exported types --------------------------------------------------------*/

/* Exported constants------------------------------------------------------------*/

/* Internal functions ------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */
void Led_Task(void const * argument);
void Check_Task(void const * argument);






#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

