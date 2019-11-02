/*************************************************************************************
*	@file			bsp.h
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************/
#ifndef test_task_h
#define test_task_h

/* Includes ------------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "SystemState.h"
#include "Power_restriction.h"
/* Exported macro ------------------------------------------------------------*/
#define printf_sendware 0   //示波器打印
#define printf_speed    0   //弹丸速度打印 
/* Exported types --------------------------------------------------------*/

/* Exported constants------------------------------------------------------------*/

/* Internal functions ------------------------------------------------------- */

void testTask(void const * argument);
/* Exported functions ------------------------------------------------------- */
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/