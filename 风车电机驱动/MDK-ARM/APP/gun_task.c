/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define GUN_PERIOD  10
#define BLOCK_TIME 5000
#define REVERSE_TIME 2000
#define prepare_flag HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)
#define READY    1
#define NO_READY 0
/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
extern uint8_t shot_frequency;
extern float Golf_speed;
//Power_Heat * power_heat;
/* 外部函数原型声明-----------------------------------------------------------
float pid_calc(pid_t* pid, float get, float set);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
pid_t pid_shot_spd[2]  = {0};	//摩擦轮速度环
pid_t pid_stir_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
  /*拨弹电机*/
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.7f,	0.0f,	1.8f);  
		//pid_dial_pos.deadband = 10;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.0f,	0.15f	);  
		/*拨盘电机*/
		PID_struct_init(&pid_stir_spd, POSITION_PID, 6000, 5000,
									3.0f,	0.0f,	0.0f	); 	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪口热量限制任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
  /*设定发弹*/
  uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
  int32_t set_stir_speed = 0;
	static uint8_t set_cnt = 0;
  static uint8_t block_flag;
  static float check_time = 0;
  static uint8_t contiue_flag = 0;
	for(;;)
	{
		RefreshTaskOutLineTime(GunTask_ON);

 /*判断发射模式*/
    if(ptr_heat_gun_t.sht_flg)
    {
     Stir_Motor(&hcan1,pid_stir_spd.pos_out);
		 minipc_rx.state_flag=0;
		 set_speed=0;	
	  }
    osDelayUntil(&xLastWakeTime,GUN_PERIOD);
  }
}

