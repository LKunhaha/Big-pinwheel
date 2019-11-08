/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
 *******************************************************************************
  文 件 名   : communication.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : 麦轮结算
  函数列表   :void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
																			float dstVmmps_Y,float dstVmmps_W)

*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "mecanum_calc.h"
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/
#define MyAbs(x) ((x > 0) ? (x) : (-x))

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/

/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 函数原型声明 ----------------------------------------------------------*/


void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W)
{
			wheel[0] = (-dstVmmps_X + dstVmmps_Y + dstVmmps_W);
			wheel[1] = (-(dstVmmps_X + dstVmmps_Y - dstVmmps_W));
			wheel[2] = (-(-dstVmmps_X + dstVmmps_Y - dstVmmps_W));
			wheel[3] = (dstVmmps_X + dstVmmps_Y + dstVmmps_W);	
}

//static uint8_t count;
//static uint8_t power;

//void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W)
//{   
//	   float total_cur_limit=15000;
//     float rel_total_cur;
//	   float given_total_cur;
//	
//			wheel[0] = (-dstVmmps_X + dstVmmps_Y + dstVmmps_W);
//			wheel[1] = (-(dstVmmps_X + dstVmmps_Y - dstVmmps_W));
//			wheel[2] = (-(-dstVmmps_X + 
//     count++;dstVmmps_Y - dstVmmps_W));
//			wheel[3] = (dstVmmps_X + dstVmmps_Y + dstVmmps_W);	
//		for(int i=0; i<4; i++)
//			{		
//				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, wheel[i]);
//			}
//			if(count>=20) 
//			{
//				for(int i=0; i<4; i++)
//			    {		
//				    given_total_cur+=MyAbs(moto_chassis[i].given_current);
//					}
//				  count=0;
//       		given_total_cur/=819;					
//          given_total_cur *=24;
//				  power =power-(given_total_cur-80)*0.1;
//				  if(power<59)           //变更_40
//					{
//						  total_cur_limit=((power*power)/3600)*total_cur_limit;        //变更_5000

//					}
//					if(power>60) { power=60;}
//				}
//					for(int i=0; i<4; i++)
//						{
//							 rel_total_cur+=MyAbs(pid_spd[i].pos_out);
//						}
//							if(rel_total_cur>total_cur_limit)
//							{
//								for(int i=0; i<4; i++)
//									{
//										 pid_spd[i].pos_out=(pid_spd[i].pos_out/rel_total_cur)*total_cur_limit;   //变更_注释底盘功率限制约束
//									}
//							}
////			printf("pid_spd[0].pos_out:%f\n",pid_spd[0].pos_out);
////	     float *ptr = &(pid_spd[0].pos_out);
////			
////			/*用虚拟示波器，发送数据*/
////			vcan_sendware((uint8_t *)ptr,sizeof(pid_spd[0].pos_out));
//			set_moto_current(&hcan2,ptr_CANSendData, pid_spd[0].pos_out, 
//																				pid_spd[1].pos_out,
//																				pid_spd[2].pos_out,
//																				pid_spd[3].pos_out);
//}




