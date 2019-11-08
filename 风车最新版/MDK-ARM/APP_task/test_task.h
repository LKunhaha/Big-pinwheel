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
#include "BSP.h"
/* Exported macro ------------------------------------------------------------*/
#define printf_sendware 1   //示波器打印
#define printf_speed    1   //弹丸速度打印 
#define printf_power    0   //弹丸速度打印 
/* Exported types --------------------------------------------------------*/
typedef struct 
{
	
 uint8_t ID;
 uint8_t state;
 uint8_t flag;       //判断是否亮过灯
 uint8_t on_off;     //判断此时灯条的状态
}LED;
/* Exported constants------------------------------------------------------------*/

/* Internal functions ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/