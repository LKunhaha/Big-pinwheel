/******************** (C) COPYRIGHT 2019 Ncu-Roboteam *********************
* File Name          : decode.c
* Author             : 
* Date First Issued  : 2019.01.02
* Description        : 
* History			 :decode the imu data
********************************************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "atom_imu.h"
#include "decode.h"
#include <string.h>


saberDataHandleType saberDataHandle;

void SelectPackets(char enable)
{
	u16 pid = 0;
	ConfigSingleDataPacket_HandleType Packet[6];
	u8 index = 0;
	for (int i = 0; i < 6; i++)
	{
		switch (i)
		{
			case CAL_ACC:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_ACC | 0x8000;
				else
					pid = SESSION_NAME_CAL_ACC;
				break;
			case CAL_GYRO:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_GYRO | 0x8000;
				else
					pid = SESSION_NAME_CAL_GYRO;
				break;
			case CAL_MAG:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_MAG | 0x8000;
				else
					pid = SESSION_NAME_CAL_MAG;
				break;
			case EULER_DATA:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_EULER | 0x8000;
				else
					pid = SESSION_NAME_EULER;
				break;
			case QUAT_DATA:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_QUAT | 0x8000;
				else
					pid = SESSION_NAME_QUAT;
				break;
			case TEMPERATURE:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_TEMPERATURE | 0x8000;
				else
					pid = SESSION_NAME_TEMPERATURE;
				break;
			default:
				break;

                }
		Packet[index].reserve0 = 0xff;
		Packet[index].reserve1 = 0xff;
		Packet[index].packetID = pid;
		index++;
		
	}
	Atom_setDataPacketConfigReq((u8*)&Packet, index * 4);

}

void Atom_setDataPacketConfigReq(u8* pData, u8 dataLen)
{
	sendPacket(MADDR_OUT, CLASS_ID_HOSTCONTROL_CMD, CMD_ID_SET_DATA_PAC_CFG, 00, pData, dataLen);
}

void DataPacketParser(void)
{
	u8 index = 0;
	u16 PID = 0;
	u8 pl = 0;
	index += 6;
	while( index-6 < UartData.Frame.Payload_length -2 )
	{
		PID = ( (UartData.buf[index]+(UartData.buf[index+1]<<8) ) & 0x7FFF);
//		pl = *(UartData.Frame.pData + index +2);
		pl = UartData.buf[index+2];
		
		if(PID == SESSION_NAME_KAL_ACC)
		{
			index += 3;
			memcpy(&saberDataHandle.accKal.accX,&(UartData.buf[index]),PL_KAL_DATA);
			saberDataHandle.accKal.dataID = PID;
			saberDataHandle.accKal.dataLen = pl;
			index += PL_KAL_DATA;
		}
		
		else if( PID ==  SESSION_NAME_CAL_GYRO) 
		{
			index += 3;
			memcpy(&saberDataHandle.gyroCal.gyroX,&(UartData.buf[index]),PL_CAL_DATA);
			saberDataHandle.gyroCal.dataID = PID;
			saberDataHandle.gyroCal.dataLen = pl;
			index += PL_CAL_DATA;
		}
		/**********中间几个包只为了让索引值增加，因此不做其他处理*************/
		else if (PID == SESSION_NAME_KAL_MAG)
		{
			index += 3;
			index += PL_KAL_DATA;
		}
		else if(PID == SESSION_NAME_QUAT)
		{
			index += 3;
			index += PL_QUAT_EULER;
		}
		/*****************end**********************/
		
		else if (PID ==  SESSION_NAME_EULER)
		{
			index += 3;
			memcpy(&saberDataHandle.euler.roll,&(UartData.buf[index]),PL_QUAT_EULER);
			saberDataHandle.euler.dataID = PID;
			saberDataHandle.euler.dataLen = pl;
			index += PL_QUAT_EULER;
		}
	}
	
	
}


