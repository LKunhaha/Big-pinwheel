/* 包含头文件----------------------------------------------------------------*/
#include "led_task.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define LED_PERIOD  600
#define Check_PERIOD  100
/* 外部变量声明--------------------------------------------------------------*/
//Power_Heat * power_heat;
/* 外部函数原型声明-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/
/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/


#if BoardNew
void Led_Task(void const * argument)
{

	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		
		if(SystemState.OutLine_Flag!=0)
		{
				if((SystemState.OutLine_Flag&0x01))
				{
						HAL_GPIO_TogglePin(GPIOG,LED1_Pin);
						osDelayUntil(&xLastWakeTime,200);
				}else  HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_RESET);
				
				
				if((SystemState.OutLine_Flag&0x02))
				{
						HAL_GPIO_TogglePin(GPIOG,LED2_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_RESET);
				
				
				if((SystemState.OutLine_Flag&0x04))
				{
						HAL_GPIO_TogglePin(GPIOG,LED3_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_RESET);
				
				if((SystemState.OutLine_Flag&0x08))
				{
						HAL_GPIO_TogglePin(GPIOG,LED4_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED4_Pin,GPIO_PIN_RESET);
				
				if((SystemState.OutLine_Flag&0x10))
				{
						HAL_GPIO_TogglePin(GPIOG,LED5_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED5_Pin,GPIO_PIN_RESET);		
				
				if((SystemState.OutLine_Flag&0x20))
				{
						HAL_GPIO_TogglePin(GPIOG,LED6_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED6_Pin,GPIO_PIN_RESET);


				if((SystemState.OutLine_Flag&0x40))
				{
						HAL_GPIO_TogglePin(GPIOG,LED7_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED7_Pin,GPIO_PIN_RESET);

				if((SystemState.OutLine_Flag&0x80))
				{
						HAL_GPIO_TogglePin(GPIOG,LED8_Pin);
						osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED8_Pin,GPIO_PIN_RESET);
		
				if((SystemState.OutLine_Flag&0x100))
				{
						HAL_GPIO_TogglePin(GPIOG,LED1_Pin);
						osDelayUntil(&xLastWakeTime,200);
					  HAL_GPIO_TogglePin(GPIOG,LED1_Pin);
					  osDelayUntil(&xLastWakeTime,200);
				} else  HAL_GPIO_WritePin(GPIOG,LED8_Pin,GPIO_PIN_RESET);
				
		osDelayUntil(&xLastWakeTime,LED_PERIOD);
		
	 }
 }

}
#endif





