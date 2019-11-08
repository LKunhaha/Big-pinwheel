#ifndef  __data_pro_task_H
#define  __data_pro_task_H
#ifdef __cplusplus
 extern "C" {
#endif
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
	 

#include "pid.h"
#include "string.h"
#include "minipc.h"
#include "protocol.h"
#include "gun_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "communication.h"
#include "Power_restriction.h"	 
/* 本模块向外部提供的数据类型定义--------------------------------------------*/	

/* 本模块向外部提供的宏定义--------------------------------------------------*/	 
#define  NORMAL_SPEED_MAX 	1000
#define  NORMAL_SPEED_MIN  -1000
//#define  HIGH_SPEED_MAX 		4000
//#define  HIGH_SPEED_MIN    -4000`
#define  ACC_SPEED    30 
#define  DEC_SPEED    300 

#define WS_Num_Max 210
typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	float dstVmmps_W;
	char  flag;
}moto3508_type;

typedef struct
{
	uint32_t head[3];
	uint32_t date_ws[WS_Num_Max*24];
	uint32_t tail[3];
}WS_buff_type;
/* 本模块向外部提供的接口常量声明--------------------------------------------*/	
extern float power; 				 //底盘功率 _测试
extern int8_t chassis_gimble_Mode_flg;
extern float chassis_Current; 
extern float	 chassis_Volt; 
extern moto3508_type  moto_3508_set; 
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/		 
void Referee_Data_Task(void const * argument);
void Remote_Data_Task(void const * argument); 
void MiniPC_Data_task(void const * argument);
void RemoteControlProcess(void);
void MouseKeyControlProcess(void);
void hard_brak(void);
void Remote_Ctrl(void);
void Minipc_Pid_Init(void);
void WS_PWM_Transmit(void);

/* 全局配置区----------------------------------------------------------------*/
	
#ifdef __cplusplus
}
#endif
#endif /*__ usart_task_H */

