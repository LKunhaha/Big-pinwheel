/* 包含头文件----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
#include "Motor_USE_CAN.h"
/* 内部宏定义----------------------------------------------------------------*/
#define press_times  20
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
//extern osSemaphoreId Dubs_BinarySemHandle;
/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};

#define REMOTE_PERIOD 5
#define MINIPC_PERIOD 2
#define REFEREE_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/

/* 调用的外部函数原型声明------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
int16_t XY_speed_max = 3000;
int16_t XY_speed_min = -3000; 
int16_t W_speed_max = 2000;
int16_t W_speed_min = -2000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 100;
int8_t chassis_gimble_Mode_flg;
//volatile float remain_power=0.0;   //底盘功率 _待续
//float power; 				 //底盘功率 _测试

//float chassis_Current; 
//float	 chassis_Volt; 


/* 内部函数原型声明-----------------------------------------------------------*/


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 6000, 5000,
									1.0f,	0.01f, 1.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000,
									1.0f,	0.01f, 1.0f	);   
		//pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
}
/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void ChassisModeProcess()
{
  /*遥控杆数据处理*/
	         if(chassis_gimble_Mode_flg==1) //XY运动，底盘跟随云台
					 {
							moto_3508_set.dstVmmps_X=-((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
					 }
					 else//WY运动，底盘云台分离
					 {
							moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
					 }
   if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
   switch(RC_Ctl.rc.s1)
    {
      case 1://上,急停
      {
        /*底盘急停*/
        hard_brak();
      }break;
      case 2://下，底盘跟随
      {

      }break;
      case 3://中,底盘分离
      {
       
      }break;
      default:break;
    
    }
  }
}
void ShotProcess()
{	
  
  if(press_counter >= press_times)//左按键延迟，时间由press_time控制
	{
		press_counter=press_times+1;
    switch(RC_Ctl.rc.s1)
      {
        case 1://上,只传送电机开
        {
 
        }break;
        case 3://中,只拨盘单发
        {
          
        }break;
        case 2://下，传送电机和拨盘一起
        {
           /*拨盘电机*/
        }break;
        
        default:break;
    }
  }
}
/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
	
	if(SHIFT_Press)//交接时要求调整速度低
      {
        XY_speed_max = 500;//(NORMAL_SPEED_MAX)*3.5;
        XY_speed_min = -500;//(NORMAL_SPEED_MIN)*3.5;
        W_speed_max = 500;
        W_speed_min = -500; 
      }
  else//正常速度
  {
     XY_speed_max = 3000;//(NORMAL_SPEED_MAX)*3.5;
     XY_speed_min = -3000;//(NORMAL_SPEED_MIN)*3.5;
     W_speed_max = 2000;
     W_speed_min = -2000;
  }
	/*Y向速度*/
  if(W_Press)                       moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下W键
  else if(S_Press)                  moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下S键
  else{  
        if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
        if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
        if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
  }
  /*分离时*/
	if(chassis_gimble_Mode_flg == 0)
  {
    
    /*W向速度*/
    if(A_Press)                       moto_3508_set.dstVmmps_W -= ACC_SPEED;//按下A键
    else if(D_Press)                  moto_3508_set.dstVmmps_W += ACC_SPEED;//按下D键
    else{  
          if(moto_3508_set.dstVmmps_W>-DEC_SPEED&&moto_3508_set.dstVmmps_W<DEC_SPEED) 	 moto_3508_set.dstVmmps_W = 0;
          if(moto_3508_set.dstVmmps_W>0) 	                   moto_3508_set.dstVmmps_W -= DEC_SPEED;
          if(moto_3508_set.dstVmmps_W<0) 		                 moto_3508_set.dstVmmps_W += DEC_SPEED;
    }
    /*X向速度*/
    if(Q_Press)                        moto_3508_set.dstVmmps_X += ACC_SPEED; //按下Q键
    else if(E_Press)    		           moto_3508_set.dstVmmps_X -= ACC_SPEED;//按下E键
    else{
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
    }
  }
  /*跟随时*/
  else
  {
    /*X向速度*/
    if(D_Press)                        moto_3508_set.dstVmmps_X -= ACC_SPEED; //按下D键
    else if(A_Press)    		           moto_3508_set.dstVmmps_X += ACC_SPEED;//按下A键
    else{
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
    }
  }
}


/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	紧急停止函数
	*	@retval	
****************************************************************************************/
void hard_brak()
{
		moto_3508_set.dstVmmps_X=0;
		moto_3508_set.dstVmmps_Y=0;
		moto_3508_set.dstVmmps_W=0;
}


/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
extern volatile uint8_t RemoteData_flag;
void Remote_Data_Task(void const * argument)
{
	uint32_t NotifyValue;
	
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	
	for(;;)
	{
			
			RefreshTaskOutLineTime(RemoteDataTask_ON);
				switch(RC_Ctl.rc.s2)
				{
          /*上*/
					case 1: ChassisModeProcess();break; 
          /*中*/
					case 3: MouseKeyControlProcess();break;
          /*下*/
					case 2: ShotProcess();break;
          
					default :break;
				}					
				
			VAL_LIMIT(moto_3508_set.dstVmmps_X, XY_speed_min, XY_speed_max);
			VAL_LIMIT(moto_3508_set.dstVmmps_Y, XY_speed_min, XY_speed_max);	
			VAL_LIMIT(moto_3508_set.dstVmmps_W, W_speed_min, W_speed_max);
				

            press_counter++;
        osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
		}

}

/***************************************************************************************
**
	*	@brief	JSYS_Task(void const * argument)
	*	@param
	*	@supplement	裁判系统数据处理任务
	*	@retval	
****************************************************************************************/
void Referee_Data_Task(void const * argument)
{
	    tFrame   *Frame;
    portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	    uint32_t NotifyValue;
	for(;;)
	{
    RefreshTaskOutLineTime(RefereeTask_ON);
    NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
        
			  NotifyValue=0;
        uint8_t *buff=USART6_RX_DATA;
			for(int8_t i=0;i<USART6_RX_NUM;i++)
			{
					if(buff[i]==0xA5)
					{
					   Frame = (tFrame *)&buff[i];
						
					    if( verify_crc16_check_sum((uint8_t *)Frame, Frame->FrameHeader.DataLength + sizeof(tFrameHeader) + sizeof(tCmdID) + sizeof(Frame->CRC16))
		             && verify_crc8_check_sum((uint8_t *)Frame,sizeof(tFrameHeader)))
								 {
									 if(Frame->CmdID==PowerANDHeat)
									 {
											current_get.Current_Referee = Frame->Data.PowerANDHeat.chassisCurrent;
											limit.Volt_Referee = Frame->Data.PowerANDHeat.chassisVolt;
											limit.PowerRemain_Referee=Frame->Data.PowerANDHeat.chassisPowerBuffer;
											ptr_heat_gun_t.rel_heat = Frame->Data.PowerANDHeat.shootHeat0;
											limit.Power_Referee = Frame->Data.PowerANDHeat.chassisPower;
											
									 }
									 if(Frame->CmdID==GameInfo)
									 {
											ptr_heat_gun_t.roboLevel=Frame->Data.GameInfo.roboLevel;
                      
									 }
									 if(Frame->CmdID==ShootData)
									 {
											ptr_heat_gun_t.shted_bullet++;
									 }
											 i=i+sizeof(Frame);
								}
					}
				
			}
					if(printf_Referee){ 
		printf("shootHeat0:%d\tchassisPowerBuffer:%f\n",
						Frame->Data.PowerANDHeat.shootHeat0,Frame->Data.PowerANDHeat.chassisPowerBuffer);
		}

	 }
    osDelayUntil(&xLastWakeTime, REFEREE_PERIOD);
 }
}	
/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	视觉数据处理任务
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	minipc_rx.state_flag = 0;
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;
  uint32_t NotifyValue;
	Minipc_Pid_Init();
	for(;;)
	{
		
		 portTickType xLastWakeTime;
		 xLastWakeTime = xTaskGetTickCount();
		
	   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			NotifyValue=0;
			Get_MiniPC_Data();
				
//			pid_calc(&pid_minipc_yaw, (int16_t)minipc_rx.angle_yaw, 0);
//			pid_calc(&pid_minipc_pit, (int16_t)minipc_rx.angle_pit, 0);
//			pid_minipc_yaw.pos_out=-(pid_minipc_yaw.pos_out);
//			pid_minipc_pit.pos_out=-(pid_minipc_pit.pos_out);
//			
//			yaw_set.expect_pc += pid_minipc_yaw.pos_out;
//			pit_set.expect_pc += pid_minipc_pit.pos_out;

//			yaw_set.expect=minipc_rx.angle_yaw+yaw_get.total_angle;
//			pit_set.expect=minipc_rx.angle_pit+pit_get.total_angle;
//			yaw_set.mode = minipc_rx.state_flag;
			
//			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
		}
	}
}

