/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/
#define GIMBAL_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/
Pos_Set  yaw_set;
Pos_Set  yaw_set_follow;
Pos_Set  pit_set;
int8_t gimbal_disable_flg;

/* 调用的外部函数原型声明------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环


pid_t pid_yaw_jy61 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_yaw_jy61_follow = {0}; 
pid_t pid_pit_jy61 = {0};
pid_t pid_yaw_jy61_spd = {0};
pid_t pid_yaw_jy61_follow_spd = {0};
pid_t pid_pit_jy61_spd = {0};


/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/
  #if imu
  /*imu pid parameter*/
  /*暂时稳定版*/
  /*50*/
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
									5.0f, 0.0f, 14.8f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  /*在调版*/
  /*35*/
//  PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
//									4.0f, 0.05f, 15.5f); 
//	PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
//                  2.0f, 0.0f, 0.0f );

	
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  10.0f, 0.02f, 10.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  #endif
  #if jy61
/*暂时稳定版*/
#define YAW_JY 0
#define PIT_JY 0

#if PIT_JY
/*pit陀螺仪反馈*/
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
                  6.0f, 0.05f, 28.0f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.2f, 0.0f, 0.0f );
#else
/*pit编码器反馈*/
  PID_struct_init(&pid_pit_jy61, POSITION_PID, 5000, 1000,
                  7.0f, 0.03f, 10.5f); //	
  PID_struct_init(&pid_pit_jy61_spd, POSITION_PID, 5000, 1000,
                  2.5f, 0.0f, 0.0f ); 
#endif

/*yaw陀螺仪反馈*/
  PID_struct_init(&pid_yaw_jy61_follow, POSITION_PID, 5000, 300,
                  6.0f, 0.03f, 30.0f); //	
  PID_struct_init(&pid_yaw_jy61_follow_spd, POSITION_PID, 5000, 100,
                  2.5f, 0.0f, 0.0f );

/*yaw编码器反馈*/
  PID_struct_init(&pid_yaw_jy61, POSITION_PID, 5000, 300,
                  10.0f, 0.1f, 4.0f); //	
  PID_struct_init(&pid_yaw_jy61_spd, POSITION_PID, 5000, 100,
                  2.5f, 0.0f, 0.0f );
  #endif

	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
//	yaw_set.expect = 0; 
//	pit_set.expect = 0;
//	yaw_set.mode   = 0;
//  yaw_set_follow.expect = 0; 
//	yaw_set_follow.mode   = 0;
//	gimbal_disable_flg=0;
//	Pitch_Current_Value=0;
//	Yaw_Current_Value=0;
//	gimbal_pid_init();
//  /*云台保护*/
//	uint16_t pit_protect_correct_1 = 0;
//	uint16_t pit_protect_correct_2 = 0;
//  if(pit_get.offset_angle < 5000)
//  {
//    pit_protect_correct_1 = 8192;
//    pit_protect_correct_2 = 0;
//  }
//  else
//  {
//    pit_protect_correct_1 = 0;
//    pit_protect_correct_2 = 8192;
//  }
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
//			
//	for(;;)		
//    {
//	
//	   RefreshTaskOutLineTime(GimbalContrlTask_ON);
//      #if imu
//        IMU_Get_Data();
//        //yaw轴
//      
//        pid_calc(&pid_yaw, yaw_get.total_angle,yaw_set.expect);
//        pid_calc(&pid_yaw_spd,(imu_data.gz), pid_yaw.pos_out);
//        //pit轴
//        pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
//        pid_calc(&pid_pit_spd,(imu_data.gy), pid_pit.pos_out);
////        pid_calc(&pid_pit_spd,(imu_data.gy), 0);
//      
//        Pitch_Current_Value=(-pid_pit_spd.pos_out); 
//		    Yaw_Current_Value= (-pid_yaw_spd.pos_out);
//      #endif
//      #if jy61
//      IMU_Get_Data();
//      yaw_set.expect = minipc_rx.angle_yaw + yaw_set.expect;
//      pit_set.expect = minipc_rx.angle_pit + pit_set.expect;
//      minipc_rx.angle_yaw = 0;
//      minipc_rx.angle_pit = 0;
//      /*云台限位保护*/
//      /*pit正常0-670，7450-8192*/
//      if((pit_set.expect + pit_get.offset_angle) > (630 + pit_protect_correct_2) &&\
//          (pit_set.expect + pit_get.offset_angle) < (2000 + pit_protect_correct_2))
//      {
//        pit_set.expect = (630 + pit_protect_correct_2) - pit_get.offset_angle;
//      }
//      if((pit_set.expect + pit_get.offset_angle) > (6500 - pit_protect_correct_1) &&\
//          (pit_set.expect + pit_get.offset_angle) < (7500 - pit_protect_correct_1))
//      {
//        pit_set.expect = (7500 - pit_protect_correct_1) - pit_get.offset_angle;
//      }
//      /*yaw轴模式判断*/
//      switch(chassis_gimble_Mode_flg)
//      {
//        case 0://分离，yaw使用编码器
//        {
//          /*yaw轴云台保护*/
//          if((yaw_set.expect + yaw_get.offset_angle) > 2400)
//          {
//            yaw_set.expect = 2380 - yaw_get.offset_angle;
//          }
//          if((yaw_set.expect + yaw_get.offset_angle) < 1100)
//          {
//            yaw_set.expect = 1115 - yaw_get.offset_angle;
//          }
//          pid_calc(&pid_yaw_jy61,(yaw_get.total_angle),yaw_set.expect);
//          pid_calc(&pid_yaw_jy61_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61.pos_out);
//          Yaw_Current_Value= (-pid_yaw_jy61_spd.pos_out);
//        }break;
//        case 1://跟随，yaw使用陀螺仪
//        {
//          /*yaw轴云台保护*/
//          
//          if(yaw_get.angle > 2400)
//          {
//            if(yaw_set_follow.expect <= yaw_set_follow.expect_last)
//            goto last;
//            yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle-5;
//          }

//          if(yaw_get.angle < 1100)
//          {
//            if(yaw_set_follow.expect >= yaw_set_follow.expect_last)
//            goto last;
//            yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;
//          }
//      last:pid_calc(&pid_yaw_jy61_follow,(ptr_jy61_t_yaw.final_angle),yaw_set_follow.expect);
//          pid_calc(&pid_yaw_jy61_follow_spd,(ptr_jy61_t_angular_velocity.vz), pid_yaw_jy61_follow.pos_out);
//          Yaw_Current_Value= (-pid_yaw_jy61_follow_spd.pos_out);
//        }break;
//      }

//        #if PIT_JY
//        //pit轴陀螺仪
//        pid_calc(&pid_pit_jy61, (ptr_jy61_t_pit.final_angle*22.76), pit_set.expect);
//        pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
//        #else
//        //pit轴编码器
//        pid_calc(&pid_pit_jy61, pit_get.total_angle, pit_set.expect);
//        pid_calc(&pid_pit_jy61_spd,(ptr_jy61_t_angular_velocity.vy), pid_pit_jy61.pos_out);
//        #endif
//        Pitch_Current_Value=(-pid_pit_jy61_spd.pos_out); 
//		    
//      #endif
////        Pitch_Current_Value = 0;
////        Yaw_Current_Value = 0;
//        /*驱动电机*/
//				if(gimbal_disable_flg==1)//失能
//				{
//					Cloud_Platform_Motor_Disable(&hcan1);
//				}
//				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
//        yaw_set_follow.expect_last = yaw_set_follow.expect;

			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
//			
//   }
// 
}
