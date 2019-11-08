/* 包含头文件----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
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
moto3508_type  moto_3508_set = {.flag = 0};
/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};


WS_buff_type WS_buff_frame;
uint8_t gBuffer[WS_Num_Max]={0x00};
static uint8_t rBuffer1[WS_Num_Max]={0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
                              0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
	                            0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
	                            0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
                              0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
	                            0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
	                            0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
                              0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
                              0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00};

//															static uint8_t rBuffer2[WS_Num_Max]={0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
//                              0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
//	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
//	                            0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
//	                            0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
//                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
//                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
//                              0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
//	                            0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
//                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
//	                            0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
//                              0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
//                              0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
//                              0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
//	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00};
static uint8_t rBuffer2[WS_Num_Max]={0xdd,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
                                     0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
	                                   0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
	                            0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
                              0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
                              0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,
                              0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
	                            0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
	                            0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
                              0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
	                            0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,
                              0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,
	                            0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00};

static uint8_t rBuffer3[WS_Num_Max]={0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,
                              0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
	                            0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
	                            0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,
	                            0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
                              0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
                              0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,
                              0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
	                            0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,
                              0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,
	                            0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
                              0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
                              0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,
                              0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd,
	                            0xdd,0x00,0x00,0x00,0xdd,0xdd,0xdd,0x00,0x00,0x00,0xdd,0xdd};

uint8_t rBuff_1[WS_Num_Max];
uint8_t rBuffer4[WS_Num_Max]={0x00};
uint8_t bBuffer[WS_Num_Max]={0x00};

uint8_t WS_Status = 1;
uint8_t WS_Flag = 1;
uint8_t WS_fl_count = 0;
uint8_t FC_Flag = 0;

#define REMOTE_PERIOD 2
#define MINIPC_PERIOD 2
#define WS_BIT_1 50u 
#define WS_BIT_0 30u 
/* 外部变量声明--------------------------------------------------------------*/

/* 调用的外部函数原型声明------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
int16_t XY_speed_max = 6000;
int16_t XY_speed_min = -6000; 
int16_t W_speed_max = 3000;
int16_t W_speed_min = -3000; 
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
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
}
/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{
      /*遥控杆数据处理*/
	         if(chassis_gimble_Mode_flg==1) //XY运动，底盘跟随云台
					 {
						  pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
							yaw_set_follow.expect = yaw_set_follow.expect +(0x400-RC_Ctl.rc.ch2)/20;	
							moto_3508_set.dstVmmps_X=-((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
             
             yaw_set.expect = yaw_get.total_angle;//更新分离编码器期望
					 }
					 else//WY运动，底盘云台分离
					 {
						  pit_set.expect = pit_set.expect +(0x400-RC_Ctl.rc.ch3)/20;	
							yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;	
							moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=-((RC_Ctl.rc.ch1-0x400)*5);
             
             yaw_set_follow.expect = ptr_jy61_t_yaw.final_angle;//更新跟随陀螺仪期望
					 }

			/*左按键数据处理*/	
					if(press_counter >= press_times)//左按键延迟，时间由press_time控制
					{
						press_counter=press_times+1;
            /*按键检查*/
            switch(RC_Ctl.rc.s1)
            {
              case 1://上
              {
                /*发射*/
                shot_anjian_counter++;
                if(shot_anjian_counter > shot_frequency)//非连续触发信号
                {
                  ptr_heat_gun_t.sht_flg=1;//单发
                  press_counter=0;
                  shot_anjian_counter=0;
                }
              }break;
              case 2://下
              {
                /*发射*/
                shot_anjian_counter++;
                if(shot_anjian_counter > shot_frequency)//非连续触发信号
                {
                  ptr_heat_gun_t.sht_flg=2;//3发
                  press_counter=0;
                  shot_anjian_counter=0;
                }
                /**/
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
              }break;
              case 3://中
              {
                /*发射*/
                ptr_heat_gun_t.sht_flg=0;
                    
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
	
	memset(gBuffer,0,210);
	memset(bBuffer,0,210);
	if(RC_Ctl.key.v & 0x10 )//设置速度档位，每档速度增加550
					{
							//p++;//shift正常挡位
						XY_speed_max = 3000;//(NORMAL_SPEED_MAX)*3.5;
						XY_speed_min = -3000;//(NORMAL_SPEED_MIN)*3.5;
					}
			
					if(RC_Ctl.key.v & 0x01)                       moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下W键
					else if(RC_Ctl.key.v & 0x02)                  moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下S键
					else{  
							 	if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
								if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
								if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
					}


					if(RC_Ctl.key.v & 0x04)                        moto_3508_set.dstVmmps_X += ACC_SPEED; //按下D键
					else if(RC_Ctl.key.v & 0x08)    		           moto_3508_set.dstVmmps_X -= ACC_SPEED;//按下A键
					else{
									if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
									if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
									if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
					}


					//鼠标（移动速度*1000/50）
					pit_set.expect = pit_set.expect+RC_Ctl.mouse.y/2;	
					
					
					//云台跟随底盘 c键 
					if(RC_Ctl.key.v & 0x2000)
					{
						moto_3508_set.flag = !moto_3508_set.flag;
					}
					
					if(RC_Ctl.mouse.press_l==1)        //鼠标左键单发
					{
						press_counter++;
							if(press_counter>=10)
			    	{
							press_counter=10+1;
							ptr_heat_gun_t.sht_flg=1;
							press_counter=0;
			    	}
					}
					else 	if(RC_Ctl.key.v & 0x100)     //r键3连发
					{
						press_counter++;
						if(press_counter>=5)
			    	{
							press_counter=5+1;
							ptr_heat_gun_t.sht_flg=2;
							press_counter=0; 
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
	  pit_set.expect=0;
	  yaw_set.expect=0;
	
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

void WS_PWM_Transmit(void)
{
	 uint8_t i, j, n;
	WS_buff_frame.head[0] = 0;
	WS_buff_frame.head[1] = 0;
	WS_buff_frame.head[2] = 0;
	WS_buff_frame.tail[0] = 0;
	WS_buff_frame.tail[1] = 0;
	WS_buff_frame.tail[2] = 0;
	if(WS_Flag)
	{
	    switch(WS_Status)
    {
	     case 1:
	   {
	     for(i = 0; i < WS_Num_Max; i++)
      {
       for(j = 0; j < 8; j++)
        {
         WS_buff_frame.date_ws[24 * i + j]= /*(gBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : */WS_BIT_0; 
         WS_buff_frame.date_ws[24 * i + j + 8]  = (rBuffer1[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0;
         WS_buff_frame.date_ws[24 * i + j + 16]= /*(bBuffer[i] & (0x80 >> j)) ? WS_BIT_1 :*/ WS_BIT_0;
        }
	    }
	   WS_Status = 2;  //  2
			break;
     }
       case 2:
     {
	    for(i = 0; i < WS_Num_Max; i++)
       {
        for(j = 0; j < 8; j++)
         {
          WS_buff_frame.date_ws[24 * i + j]= /*(gBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : */WS_BIT_0; 
          WS_buff_frame.date_ws[24 * i + j + 8]  = (rBuffer2[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0;
          WS_buff_frame.date_ws[24 * i + j + 16]= /*(bBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : */WS_BIT_0;
         }
       }
	     WS_Status = 3;  //3  
			 break;
     }
       case 3:
     {
	     for(i = 0; i < WS_Num_Max; i++)
        {
         for(j = 0; j < 8; j++)
          {
           WS_buff_frame.date_ws[24 * i + j]= /*(gBuffer[i] & (0x80 >> j)) ? WS_BIT_1 :*/ WS_BIT_0; 
           WS_buff_frame.date_ws[24 * i + j + 8]  = (rBuffer3[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0;
           WS_buff_frame.date_ws[24 * i + j + 16]= /*(bBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : */WS_BIT_0;
          }
        }
	      WS_Status = 1;  //1
				break;
     }
		 
		 case 4:
     {
	     for(i = 0; i < WS_Num_Max; i++)
        {
         for(j = 0; j < 8; j++)
          {
           WS_buff_frame.date_ws[24 * i + j]= /*(gBuffer[i] & (0x80 >> j)) ? WS_BIT_1 :*/ WS_BIT_0; 
           WS_buff_frame.date_ws[24 * i + j + 8]  = (rBuffer4[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0;
           WS_buff_frame.date_ws[24 * i + j + 16]= /*(bBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : */WS_BIT_0;
          }
        }
	      WS_Status = 2;  //1
				break;
     }
        default: break;

     }
  }else
	{
		for(n = 0; n < WS_fl_count; n++)
		{
			rBuff_1[n] = 0xdd;
			rBuff_1[83-n] = 0xdd;
			rBuff_1[84+n] = 0xdd;
			rBuff_1[167-n] = 0xdd;
			rBuff_1[168+n] = 0xdd;
		}
		for(i = 0; i < WS_Num_Max; i++)
        {
         for(j = 0; j < 8; j++)
          {
           WS_buff_frame.date_ws[24 * i + j]= (gBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0; 
           WS_buff_frame.date_ws[24 * i + j + 8]  = (rBuff_1[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0;
           WS_buff_frame.date_ws[24 * i + j + 16]= (bBuffer[i] & (0x80 >> j)) ? WS_BIT_1 : WS_BIT_0;
          }
        }
		WS_fl_count++;
		if(WS_fl_count>42)
		{
			WS_fl_count = 42;
		}
	}
	 __HAL_TIM_DISABLE(&htim5);
	 HAL_TIM_PWM_Start_DMA(&htim5,TIM_CHANNEL_2, (uint32_t*)&WS_buff_frame,6+WS_Num_Max*24);
	  i = 0, j = 0, n = 0;

}
void Remote_Data_Task(void const * argument)
{
	uint32_t NotifyValue;
	
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	 
	for(;;)
	{
		 if(0 == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10))
		 {
			 osDelayUntil(&xLastWakeTime, 15);
			 if(0 == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10))
			 {
				 if (FC_Flag == 0)
			        FC_Flag = 1;
			   else FC_Flag = 0;
			 }
		 }
//	  WS_PWM_Transmit();

			osDelayUntil(&xLastWakeTime, 150);
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
	
	    uint32_t NotifyValue;
	for(;;)
	{
				   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
//			printf("running!!!\n");
			  NotifyValue=0;
			
				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1); //GRE_H
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

			yaw_set.expect=minipc_rx.angle_yaw+yaw_get.total_angle;
			pit_set.expect=minipc_rx.angle_pit+pit_get.total_angle;
			yaw_set.mode = minipc_rx.state_flag;
			
			osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
		}
	}
}

