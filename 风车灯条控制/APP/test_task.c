#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "communication.h "
#include "usart.h"
#include "pid.h"

extern uint32_t rrand_count;
uint8_t	rand_count=1;
uint8_t LED_rand;
uint8_t aaaa;
uint8_t tx_date[8];
CAN_TxHeaderTypeDef	 Chassis_Motor_Header;
CAN_RxHeaderTypeDef  Rx_Header;

static  int16_t Yaw_Current_Value = 0;
pid_t pid_pit_spd   = {0};	//pit轴速度环
float Set_speed_z;
uint8_t u_send_jy[4];

void Vibration_Read( void);
void rand_light(void);
void light_reset(void);
void time_check(void);
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

///////////////////////////////////////////////////////////
void TestTask_EN(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  srand(rrand_count);
//	Motor1( ); LED1.flag = 1;LED1.on_off=1;
//  Motor2( ); LED2.flag = 1;LED2.on_off=1;
    Motor3( ); LED3.flag = 1;LED3.on_off=1; 
//  Motor4( ); LED4.flag = 1;LED4.on_off=1; 
//  Motor5( ); LED5.flag = 1;LED5.on_off=1;
	
	for(;;)
  {
		aaaa = (rand()%5 + 1);
		Vibration_Read( );
	
		osDelayUntil(&xLastWakeTime,50);
   }
}

/////////////////////////////////////////////////////////
void TestTask_MA(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{    
      IMU_Get_Data();
		Set_speed_z = 600;
			 pid_calc(&pid_pit_spd,(-imu_data.gz), Set_speed_z);
			 Yaw_Current_Value = pid_pit_spd.pos_out;
		
		  u_send_jy[0] = 0xFF;
			u_send_jy[1] = Yaw_Current_Value>>8;
			u_send_jy[2] = Yaw_Current_Value;
			u_send_jy[3] = 0xFE;
			
			HAL_UART_Transmit(&huart2,u_send_jy,4, 10);
		  osDelayUntil(&xLastWakeTime,10);
	}
}
////////////////////////////////////////////////////////
void TestTask_LS(void const * argument)
{
	for(;;)
	{
   osDelay(200);
	}
}
///////////////////////////////////////////////////

void Vibration_Read( )
{
	
	if(LED1.state && LED1.on_off)
	{
		osDelay(10);
		if(LED1.state)
		{
			 Motor1_Dis( );
			 LED1.on_off = 0;
			 rand_light();
		}
  }
	
	if(LED2.state && LED2.on_off)
	{
		osDelay(10);
		if(LED2.state)
		{
			 Motor2_Dis( );
			 LED2.on_off = 0;
			 rand_light();
		}
	}
	
	if(LED3.state && LED3.on_off)
	{
		osDelay(10);
		if(LED3.state)
		{
			 Motor3_Dis( );
			 LED3.on_off = 0;
			 rand_light();
		}
	}
	
	if(LED4.state && LED4.on_off)
	{
		osDelay(10);
		if(LED4.state)
		{
			 Motor4_Dis( );
			 LED4.on_off = 0;
			 rand_light();
		}
	}
	
	if(LED5.state && LED5.on_off)
	{
		osDelay(10);
		if(LED5.state)
		{
			 Motor5_Dis( );
			 LED5.on_off = 0;
			 rand_light();
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
			case 1:Motor1( ); LED1.flag = 1;LED1.on_off = 1; break;
		  case 2:Motor2( ); LED2.flag = 1;LED2.on_off = 1; break;
			case 3:Motor3( ); LED3.flag = 1;LED3.on_off = 1; break;
			case 4:Motor4( ); LED4.flag = 1;LED4.on_off = 1; break;
			case 5:Motor5( ); LED5.flag = 1;LED5.on_off = 1; break;
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


void Motor_roll(CAN_HandleTypeDef * hcan,signed int speed)
{
	    Chassis_Motor_Header.DLC = 0x08;
			Chassis_Motor_Header.IDE = CAN_ID_STD;
			Chassis_Motor_Header.RTR = CAN_RTR_DATA;
			Chassis_Motor_Header.StdId = 0x10;

	    uint32_t TxMailbox;
	
			tx_date[0] = 8;        //长度
	    tx_date[1] = 0x10;     //ID
	    tx_date[2] = 0x90;     //速度指令
      tx_date[3] = 0;
	    tx_date[4] = ((speed>>(24-8*3))&0xff);
	    tx_date[5] = ((speed>>(24-8*2))&0xff);
	    tx_date[6] = ((speed>>(24-8*1))&0xff);
	    tx_date[7] = ((speed>>(24-8*0))&0xff);
	
	
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Header, tx_date, &TxMailbox );
	
	
	
}