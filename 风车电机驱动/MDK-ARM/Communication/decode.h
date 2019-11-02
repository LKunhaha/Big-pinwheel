/******************** (C) COPYRIGHT 2019 Ncu-Roboteam *********************
* File Name          : decode.h
* Author             : 
* Date First Issued  : 2019.01.02
* Description        : 
* History			 :
********************************************************************************/
#ifndef _DECODE_H
#define	_DECODE_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "atom_imu.h"


#define		CAL_ACC			0
#define		CAL_GYRO		1	
#define		CAL_MAG			2
#define		EULER_DATA		3
#define		QUAT_DATA		4
#define		TEMPERATURE		5


#define		PACKET_CAL_ACC					(1<<CAL_ACC)    
#define		PACKET_CAL_GYRO					(1<<CAL_GYRO)    
#define		PACKET_CAL_MAG					(1<<CAL_MAG)    
#define		PACKET_EULER_DATA				(1<<EULER_DATA)    
#define		PACKET_QUAT_DATA				(1<<QUAT_DATA)    
#define		PACKET_TEMPERATURE				(1<<TEMPERATURE)    

#define 	RATE_50		50
#define		RATE_100	100
#define		RATE_200	200
#define		RATE_400	400
#define		RATE_500	500    //swift mode only
#define		RATE_800	800    //swift mode only
#define		RATE_1000       1000   //swift mode only


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

typedef struct
{
	u16 dataID;
	u8 dataLen;
//	u8 reserve;
	float accX;
	float accY;
	float accZ;
}SaberData_KAL_ACC_HandleType;

typedef struct
{
	u16 dataID;
	u8 dataLen;
//	u8 reserve;
	float gyroX;
	float gyroY;
	float gyroZ;
}SaberData_CAL_GYRO_HandleType;

typedef struct
{
	u16 dataID;
	u8 dataLen;
//	u8 reserve;
	float roll;
	float pitch;
	float yaw;
	float reserve;
}SaberData_Euler_HandleType;

typedef struct
{
	SaberData_CAL_GYRO_HandleType gyroCal;
	SaberData_KAL_ACC_HandleType accKal;
	SaberData_Euler_HandleType euler;
}saberDataHandleType;


extern saberDataHandleType saberDataHandle;

void DataPacketParser(void);
void SelectPackets(char enable);
void Atom_setDataPacketConfigReq(u8* pData, u8 dataLen);















#endif


