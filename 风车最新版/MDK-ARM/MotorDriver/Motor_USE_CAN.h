/*************************************************************************************
*	@file			Motor_USE_CAN.h
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************/
#ifndef __MOTOR_USE_CAN_H
#define __MOTOR_USE_CAN_H

/* Includes ------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "can.h"
/* Exported macro ------------------------------------------------------------*/
#define FILTER_BUF_LEN		5
/* Exported types --------------------------------------------------------*/
typedef enum
{
  /*CAN1*/
  CAN_6623_YAW = 0X205,
  CAN_6623_PIT = 0X206,
  /*ÔÆÌ¨->µ×ÅÌ*/
  CAN_Remote = 0x110,
  CAN_Yaw = 0x120,
  CAN_Stir = 0x120,
  CAN_Error = 0x119,
  /*µ×ÅÌ->ÔÆÌ¨*/
  CAN_Referee = 0x011,
  CAN_Chassis = 0x911,
  /*CAN2*/
  CAN_2006_B = 0X201,
	CAN_3508_M1 = 0x202,
	CAN_3508_M2 = 0x203,
		
}CAN_Message_ID;

typedef struct{
	int16_t	 			speed_rpm;
	int16_t  			real_current;
	int16_t  			given_current;
	uint8_t  			hall;
	uint16_t 			angle;				//abs angle range:[0,8191]
	uint16_t 			last_angle;	//abs angle range:[0,8191]
	uint16_t			offset_angle;
	int32_t				round_cnt;
	int32_t				total_angle;
  int32_t       total_ture_angle;
	uint8_t				buf_idx;
	uint16_t			angle_buf[FILTER_BUF_LEN];
	uint16_t			fited_angle;	
	uint32_t			msg_cnt;
  int32_t      run_time;
	int32_t      cmd_time;
	int32_t      reverse_time;
}moto_measure_t;

/* Exported constants------------------------------------------------------------*/
extern moto_measure_t   moto_chassis_get[];
extern moto_measure_t   moto_dial_get;   
extern moto_measure_t   moto_M_get[2]; 
extern moto_measure_t   pit_get;
extern moto_measure_t   yaw_get;
/* Internal functions ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
void Cloud_Platform_Motor_Correct(CAN_HandleTypeDef * hcan);
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan);

void Shot_Motor(CAN_HandleTypeDef * hcan,int16_t bo_value,int16_t M1_value,int16_t M2_value);

void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_total_angle(moto_measure_t *p);

void CAN_Send_YK( CAN_HandleTypeDef * hcan,int16_t key_v, int16_t rc_ch0, int16_t rc_ch1, uint8_t rc_s1, uint8_t rc_s2);
void CAN_Send_Error( CAN_HandleTypeDef * hcan, int16_t OutLine_Flag, int16_t task_OutLine_Flag );
void CAN_Send_YT( CAN_HandleTypeDef * hcan, int16_t yaw_angle, int16_t yaw_speed , uint8_t flag , uint8_t flag1);
void CAN_Send_Stir( CAN_HandleTypeDef * hcan,uint8_t flag);
void CAN_GET_DP( CAN_HandleTypeDef * hcan);

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

