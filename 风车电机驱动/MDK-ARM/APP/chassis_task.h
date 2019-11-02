#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif 
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
	 
#include "pid.h"
#include "gimbal_task.h"	 
#include "mecanum_calc.h"
#include "Motor_USE_CAN.h"
#include "Power_restriction.h"
#include "data_pro_task.h"
/* 本模块向外部提供的数据类型定义--------------------------------------------*/	
typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	float dstVmmps_W;
	char  flag;
}moto3508_type;


/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/
extern moto3508_type  moto_3508_set; 
extern int8_t chassis_disable_flg;
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Chassis_Task(void const * argument);

/* 全局配置区----------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif 
