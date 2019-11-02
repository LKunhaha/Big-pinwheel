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
  CAN_2006_B = 0X201,
  CAN_3508_STIR = 0X202,
  CAN_6623_YAW = 0X205,
  CAN_6623_PIT = 0X206,
  /*CAN2*/
	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
	CAN_YT = 0x120,
	CAN_YK = 0x110,
	CAN_ERROR =  0x119
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
extern moto_measure_t   moto_dial_get;   //					_×¢ÊÍ
extern moto_measure_t   moto_bo;
extern moto_measure_t   moto_stir_get;
extern moto_measure_t   pit_get;
extern moto_measure_t   yaw_get;
extern uint8_t flag ; 
/* Internal functions ------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
void Cloud_Platform_Motor_Correct(CAN_HandleTypeDef * hcan);
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan);

void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan);

void Stir_Motor(CAN_HandleTypeDef * hcan,int16_t value);
void Stir_Motor_Disable(CAN_HandleTypeDef * hcan);

void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);

void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_total_angle(moto_measure_t *p);

void CAN_RX_YK(CAN_HandleTypeDef * hcan);
void CAN_RX_YT(CAN_HandleTypeDef * hcan);
void CAN_SEND_DP( CAN_HandleTypeDef * hcan,int16_t speed_rpm1, int16_t speed_rpm2, 
	int16_t speed_rpm3, uint8_t speed_rpm4);
void CAN_RX_ERROR(CAN_HandleTypeDef * hcan);

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

