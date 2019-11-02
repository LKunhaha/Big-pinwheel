/* 包含头文件----------------------------------------------------------------*/
#include "chassis_task.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId Chassis_QueueHandle;

/* 内部常量定义--------------------------------------------------------------
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
float pid_calc(pid_t* pid, float get, float set);
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
													float dstVmmps_Y,float dstVmmps_W);
----------------------------------------------------------------------------
*/
/* 外部变量声明--------------------------------------------------------------*/
moto3508_type  moto_3508_set = {.flag = 0}; 
extern float power;		//功率  	_测试变量
int8_t chassis_disable_flg;
/* 调用的外部函数原型声明----------------------------------------------------------*/

/* 内部变量------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //底盘电机位置环
pid_t pid_3508_spd[4];			 //底盘电机速度环
pid_t pid_3508_current[4];	 //底盘电机电流环节	
pid_t pid_chassis_follow = {0};//底盘跟随位置环
pid_t pid_chassis_follow_spd = {0};//底盘跟随速度环

//int chiss_count;

static float Current_set[4] = {0};  //传递给功率限制的缓存

//测试变量
int16_t angle[2];

extern int16_t yaw_speed;
#define CHASSIS_PERIOD 5

/* 内部函数原型声明----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 1000,
									1.5f,	0.0f,	20.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
	 pid_3508_pos.deadband=150;
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,10000,1000,
	                4.0f, 0.01f , 20.0f  );
//	  pid_chassis_follow.deadband=10;
	 PID_struct_init(&pid_chassis_follow_spd, POSITION_PID,4000,1000,
	                0.8f, 0.0f , 0.0f  );
	
		for(int i=0; i<4; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000,
										1.5f,	0.1f,	0.1f	);  //4 motos angular rate closeloop.
		}
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	底盘控制任务
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
  /*数据初始化*/
	static float  wheel[4]={0,0,0,0};
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  chassis_disable_flg=0;
  
  /*功能初始化*/
	Chassis_pid_init();
  
	for(;;)
	{

    RefreshTaskOutLineTime(ChassisContrlTask_ON);
    /*底盘模式*/
//		switch(chassis_gimble_Mode_flg)
//		{	
//			case 0:	{	//分离	
//        /*麦轮解算得出wheel[4]*/
//			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W);
//			}break;
//      
//			case 1:	{	//底盘跟随云台
//	  	/*有两个组合：
//			*						 1. get:yaw_get.totalangle 
//			* 							set:0     			因为一开机，云台所处的位置是在哪里，哪里的totalangle就是0。
//			*			  		 2. get:yaw_get.angle
//			*								set:枪口在正中心时候的云台绝对值 
//			*/	
//        /*跟随位置环*/
//			pid_calc(&pid_chassis_follow,yaw_get.total_angle,0);
//        /*跟随速度环*/ 
//			pid_calc(&pid_chassis_follow_spd,-yaw_speed,pid_chassis_follow.pos_out);
//		
//        /*麦轮解算得出wheel[4]*/
//			motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,pid_chassis_follow_spd.pos_out); 																																												
//			}break;
//		}
    /*速度环计算*/
//		for(int i=0; i<4; i++)
//			{		
//				//pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
//        pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, 600);
//			}
		
		/**********功率限制*********/

//			Current_set[0] = pid_3508_spd[0].pos_out;
//			Current_set[1] = pid_3508_spd[1].pos_out;
//			Current_set[2] = pid_3508_spd[2].pos_out;
//			Current_set[3] = pid_3508_spd[3].pos_out;			
			
//			printf("befeor:%f   ",Current_set[0]);
//		  power_limit(Current_set);
//			printf("after:%f\n\r",Current_set[0]);
			
//			pid_3508_spd[0].pos_out = Current_set[0];			
//			pid_3508_spd[1].pos_out = Current_set[1];
//			pid_3508_spd[2].pos_out = Current_set[2];
//			pid_3508_spd[3].pos_out = Current_set[3];

							
	  /************end***********/	
			
      /*驱动电机*/
//      if(chassis_disable_flg==1)//失能
//			{
//				  Chassis_Motor_Disable(&hcan2);
//			}
//			else
//			{
//					Chassis_Motor(&hcan2,
//												pid_3508_spd[0].pos_out,
//												pid_3508_spd[1].pos_out, 
//												pid_3508_spd[2].pos_out, 
//												pid_3508_spd[3].pos_out);

//      }        
//				
//			if(0){  //数据发送和任务检测   _待续
			//	if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0) == 1)            //弹仓检测  低电平触发发送0为子弹空
																															 //                    发送1为弹夹满
			//	{
			//		data1 = 1;
			//	}
			//	else if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0) == 0)
			//	{	
			//	data1 = 0;
			//	}
			//	sendata();
			//		
			//	
			//	
			//		if(data_pro_task_flag==1) { data_pro_task_flag=0;flag_counter=0;}				//错误检测
			//		else flag_counter++;
			//		if(flag_counter>100)  { NVIC_SystemReset();}
			//		if(flag_counter>200)
			//			flag_counter = 0;
//				}
			
			
			Cloud_Platform_Motor(&hcan2,minipc_rx.yaw_out,minipc_rx.yaw_out);
//			Cloud_Platform_Motor(&hcan2,500,500);//调试用
//			chiss_count++;
			osDelay(5); 
  }
}


