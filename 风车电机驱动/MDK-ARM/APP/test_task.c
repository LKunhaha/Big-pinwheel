/*************************************************************************************
*	@file			test_task.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "test_task.h"

/* External variables --------------------------------------------------------------*/
/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/

/*测速模块*/
#define GunLength 0.05
#define MicroTime 0.000005


extern Current_GET  current_get;

uint32_t Micro_Tick; //单位0.005ms
uint32_t Photoelectric_gate1 = 0,Photoelectric_gate2 = 0;
uint16_t gate1_counter = 0,gate2_counter = 0;
 float Golf_speed = 0;
int16_t Golf_counter = 0;
/*测速down*/

void testTask(void const * argument)
{

	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();		
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
    #if printf_speed
    
			Golf_speed = (float)(GunLength / MicroTime / (Photoelectric_gate1 - Photoelectric_gate2));

			if(counter_last != gate1_counter)
			{  
				printf("Golf_speed = %4f\n",Golf_speed);
			}
			counter_last = gate1_counter;
			
		printf("ADC_current= %4f\n",current_get.CurrentCalculat);     //电流采集数据打印
			
    #endif
  #if printf_sendware
		  int16_t  *ptr = &angle; //初始化指针
      int16_t  *p1  = &speed_set;
			angle	= (pit_get.total_angle);
			/*用虚拟示波器，发送数据*/
			vcan_sendware((uint8_t *)ptr,sizeof(angle));
		
//		printf("  pit=%d \n\t",pit_get.total_angle);
//	  printf("  yaw=%d \n\t",yaw_get.angle);
		
//		HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin); //Red
	#endif
		osDelayUntil(&xLastWakeTime,50);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/