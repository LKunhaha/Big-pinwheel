/*************************************************************************************
*	@file			SystemState.c
* @author	 	   by  donglin   
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "SystemState.h"
#include "tim.h"
/* External variables --------------------------------------------------------------*/
/* Internal variables --------------------------------------------------------------*/
SystemStateDef SystemState={0};
float g_TimePer[100]={0};
float g_Time_DeviceOutLine[DeviceTotal_No]={0};//外设最近一次通信时间数组
float g_Time_TASKOutLine[TASKTotal_No]={0};//外设最近一次通信时间数组
/* Private function prototypes ---------------------------------------------------*/
//断线检测检测
void OutLine_Check()
{
	short num=0;//临时变量累加用
	float time=GetSystemTimer();//当前系统时间

	for(num=0;num<DeviceTotal_No;num++)
	{
		if(time-g_Time_DeviceOutLine[num]>OutLine_Time)
		{
			MyFlagSet(SystemState.OutLine_Flag,(num));//设置断线标志
		}
		else
		{
			MyFlagClear(SystemState.OutLine_Flag,(num));//清除断线标志
		}
	}
}

//断线检测检测
void TASK_Check()
{
	short num=0;//临时变量累加用
	float time=GetSystemTimer();//当前系统时间

	for(num=0;num<TASKTotal_No;num++)
	{
		if(time-g_Time_TASKOutLine[num]>OutLine_Time)
		{
			MyFlagSet(SystemState.task_OutLine_Flag,(num));//设置断线标志
		}
		else
		{
			MyFlagClear(SystemState.task_OutLine_Flag,(num));//清除断线标志
		}
	}
}


//=====================================================
//							  内部函数
//
//====================================================


//断线检测
void vOutLineCheck_Task(void const *argument)
{

	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		RefreshTaskOutLineTime(vOutLineCheckTask_ON);
		
		
		TASK_Check();//任务检测
		OutLine_Check();//断线检测
		osDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
		
	}
}



int SystemState_Inite()
{
	int state;
	SystemState.Enable=0;
	SystemState.State=0;
	SystemState.Task=0;
	SystemState.Time=0;
	SystemState.htim=&htim6;//计时器请设定 每 10us 记一个数  重载值为 100-1 (1ms)  例如 Timer3 主频168M 预分频 (840-1) 重载值 (100-1)
//	state=HAL_TIM_Base_Start_IT(SystemState.htim);//启动时间计数器
  return state;
}

//中断刷新中调用 更新系统时间 ms
 void RefreshSysTime(void)
{
		SystemState.Time+=1;
}


//获得系统时间
uint32_t GetSystemTimer()
{
	return HAL_GetTick();
}


//刷新外设通信时间时间数组
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	
	g_Time_DeviceOutLine[DevX_No]=GetSystemTimer();
	
}



//刷新任务时间数组
void RefreshTaskOutLineTime(TASK_NoDEF Task_No)
{
	
	g_Time_TASKOutLine[Task_No]=GetSystemTimer();
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/








