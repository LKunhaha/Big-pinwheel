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
#define Check_PERIOD  100
/* Private function prototypes ---------------------------------------------------*/

/*测速模块*/
#define GunLength 0.05
#define MicroTime 0.000005

uint32_t Micro_Tick; //单位0.005ms
uint32_t Photoelectric_gate1 = 0,Photoelectric_gate2 = 0;
uint16_t gate1_counter = 0,gate2_counter = 0;
 float Golf_speed = 0;
int16_t Golf_counter = 0;
/*测速down*/

extern uint32_t rrand_count;
extern uint8_t FC_Flag;
uint8_t	rand_count=1;
uint8_t LED_rand;
uint8_t aaaa;
uint8_t tx_date[8];
uint32_t reset_count = 0;

void Vibration_Read( void);
void rand_light(void);
void light_reset(void);
void time_check(void);
void over_time_check(void);
	
void Motor1(void);
void Motor2(void);
void Motor3(void);
void Motor4(void);
void Motor5(void);
void Motor1_Dis( void);
void Motor2_Dis( void);
void Motor3_Dis( void);
void Motor4_Dis( void);
void Motor5_Dis( void);

LED LED1={1,0,0,0};
LED LED2={2,0,0,0};
LED LED3={3,0,0,0};
LED LED4={4,0,0,0};
LED LED5={5,0,0,0};

void testTask(void const * argument)
{
	 srand(rrand_count);
//	Motor1( ); LED1.flag = 1;LED1.on_off=1;
//  Motor2( ); LED2.flag = 1;LED2.on_off=1;
//    Motor3( ); LED3.flag = 1;LED3.on_off=1; 
//  Motor4( ); LED4.flag = 1;LED4.on_off=1; 
  Motor5( ); LED5.flag = 1;LED5.on_off=1;
//	char InfoBuffer[1000];
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float speed[30];
	
	
	for(;;)
	{ 				
		aaaa = (rand()%5 + 1);
		Vibration_Read( );
		//*   风车超时检测，不用就屏蔽  *//
//		if(FC_Flag)
//		{
//			if( !(LED1.flag&&LED2.flag&&LED3.flag&&LED4.flag&&LED5.flag) )          
//		  {
//	     over_time_check( );
//		  }
//	  }
		//*                            *//
		osDelayUntil(&xLastWakeTime,50);
	}
}
void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	if((SystemState.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
				
				
//				if((SystemState.task_OutLine_Flag&0x02))
//				{
//					printf("ChassisContrlTask GG \n\t");
//					Chassis_Motor_Disable(&hcan2);
//					osDelayUntil(&xLastWakeTime,100);
//				} 
//				
//				
//				if((SystemState.task_OutLine_Flag&0x04))
//				{
//						printf("RemoteDataTask GG \n\t");
//						HAL_UART_DMAPause(&huart1);
//				    *USART1_RX_DATA = 0;
//						osDelayUntil(&xLastWakeTime,100);
//				} 
//				
//				if((SystemState.task_OutLine_Flag&0x08))
//				{
//						printf("GimbalContrlTask GG \n\t");
//					  Cloud_Platform_Motor_Disable(&hcan1);
//						osDelayUntil(&xLastWakeTime,100);
//				} 
//				
//				if((SystemState.task_OutLine_Flag&0x10))
//				{
//						printf("GunTask GG \n\t");
//						osDelayUntil(&xLastWakeTime,100);
//				} 
//				
//				if((SystemState.task_OutLine_Flag&0x20))
//				{
//						printf("LedTask GG \n\t");
//						osDelayUntil(&xLastWakeTime,100);
//				} 


//				if((SystemState.task_OutLine_Flag&0x40))
//				{
//						printf("vOutLineCheckTask GG \n\t");
//						osDelayUntil(&xLastWakeTime,100);
//				} 

		
				osDelayUntil(&xLastWakeTime,Check_PERIOD);
	
	}
	
	
}

void Vibration_Read( )
{
	
	if(LED1.state && LED1.on_off)
	{
		osDelay(10);
		if(LED1.state)
		{
			 Motor1_Dis( );
			 LED1.on_off = 0;
			 LED1.flag = 1;
			 rand_light();
			 reset_count = 0;
		}
  }
	
	if(LED2.state && LED2.on_off)
	{
		osDelay(10);
		if(LED2.state)
		{
			 Motor2_Dis( );
			 LED2.on_off = 0;
			 LED2.flag = 1;
			 rand_light();
			 reset_count = 0;
		}
	}
	
	if(LED3.state && LED3.on_off)
	{
		osDelay(10);
		if(LED3.state)
		{
			 Motor3_Dis( );
			 LED3.on_off = 0;
			 LED3.flag = 1;
			 rand_light();
			 reset_count = 0;
		}
	}
	
	if(LED4.state && LED4.on_off)
	{
		osDelay(10);
		if(LED4.state)
		{
			 Motor4_Dis( );
			 LED4.on_off = 0;
			 LED4.flag = 1;
			 rand_light();
			 reset_count = 0;
		}
	}
	
	if(LED5.state && LED5.on_off)
	{
		osDelay(10);
		if(LED5.state)
		{
			 Motor5_Dis( );
			 LED5.on_off = 0;
			 LED5.flag = 1;
			 rand_light();
			 reset_count = 0;
		}
	}
}



void rand_light(void)         //随机亮灯
{
	  LED_rand = (rand()%5 + 1);
		while(1)
		{
			if(LED_rand == LED1.ID)
			{ 
			   if(LED1.flag == 0)
					 break;
			}
			
			if(LED_rand == LED2.ID)
			{ 
			   if(LED2.flag == 0)
					 break;
			}
			
			if(LED_rand == LED3.ID)
			{ 
			   if(LED3.flag == 0)
					 break;
			}
			
			if(LED_rand == LED4.ID)
			{ 
			   if(LED4.flag == 0)
					 break;
			}
			
			if(LED_rand == LED5.ID)
			{ 
			   if(LED5.flag == 0)
					 break;
			}
			
			time_check( ); 
			
			LED_rand = (rand()%5 + 1);
		}

		
		switch(LED_rand)
		{
			case 1:Motor1( ); osDelay(30);LED1.on_off = 1; break;
		  case 2:Motor2( ); osDelay(30);LED2.on_off = 1; break;
			case 3:Motor3( ); osDelay(30);LED3.on_off = 1; break;
			case 4:Motor4( ); osDelay(30);LED4.on_off = 1; break;
			case 5:Motor5( ); osDelay(30);LED5.on_off = 1; break;
			default:  ;
		}
}


void light_reset(void)
{
	Motor1_Dis( );  LED1.state = 0;LED1.flag = 0; LED1.on_off = 0;
	Motor2_Dis( );  LED2.state = 0;LED2.flag = 0; LED2.on_off = 0;
	Motor3_Dis( );  LED3.state = 0;LED3.flag = 0; LED3.on_off = 0;
	Motor4_Dis( );  LED4.state = 0;LED4.flag = 0; LED4.on_off = 0;
	Motor5_Dis( );  LED5.state = 0;LED5.flag = 0; LED5.on_off = 0;

}	

void time_check(void) 
{ 
  if( LED1.flag && LED2.flag && LED3.flag && LED4.flag && LED5.flag )
	{ 
		 Motor1( ); 
	   Motor2( ); 
	   Motor3( );
	   Motor4( );
	   Motor5( ); 
		
		 osDelay(4000);
		light_reset( );

	}
}	

void over_time_check(void)
{
	 if(reset_count > 2500)
	{
		reset_count = 0;
		light_reset( );
		rand_light( );
	}
	
}

void Motor1( )
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET);
}

void Motor2( )
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_SET);
}

void Motor3( )
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_SET);
}

void Motor4( )
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, GPIO_PIN_SET);
}

void Motor5( )
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, GPIO_PIN_SET);
}

void Motor1_Dis( )
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_RESET);
}

void Motor2_Dis( )
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_RESET);
}

void Motor3_Dis( )
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, GPIO_PIN_RESET);
}

void Motor4_Dis( )
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, GPIO_PIN_RESET);
}

void Motor5_Dis( )
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, GPIO_PIN_RESET);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/