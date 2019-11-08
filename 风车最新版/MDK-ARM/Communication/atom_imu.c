/******************** (C) COPYRIGHT 2019 Ncu-Roboteam *********************
* File Name          : atom_imu.c
* Author             : 
* Date First Issued  : 2019.01.02
* Description        : 
* History			 :
********************************************************************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "atom_imu.h"
#include "decode.h"
#include "string.h"
/*********************global value**********************/
uint8_t  temp_data = 0;						//接收一个数据
uint16_t USART8_RX_NUM;						//每次接收数据的数量
uint16_t send_data_count = 0;				//数据接收计数
uint8_t  errorCode_Ack;

uint8_t receiveAck = 0 ;
uint8_t receiveCid ;
uint8_t receiveMid ;
MidProtocol_StructType UartData;
Buffer_Type HOST_Buffer;
/************************end***************************/



u8  Atom_BCC(u8 *addr, u16 len)
{
	unsigned char *DataPoint;
	DataPoint = addr;
	unsigned char XorData = 0;
	unsigned short DataLength = len;

	while (DataLength--)
	{
		XorData = XorData ^ *DataPoint;
		DataPoint++;
	}

	return XorData;
}

//底层发送代码
int  AtomCmd_Compose_Send(u8 MADDR, u8 ClassID, u8 msgID, u8* payloadBuffer, u16 PayloadLen)
{
	int index = 0;
	int ret = 0;
	u8 crc;
	uint8_t pBuf[48] = {0};

	//unsigned char *pBuf = (unsigned char*)malloc(ATOM_CMDLEN_NO_PAYLOAD + PayloadLen);

	//Header
	pBuf[index++] = 'A';
	pBuf[index++] = 'x';
	pBuf[index++] = MADDR;
	pBuf[index++] = ClassID;
	pBuf[index++] = msgID;
	pBuf[index++] = PayloadLen & 0xff;
	//Payload
	
	for (int i = 0; i < PayloadLen; i++)
		pBuf[index++] = payloadBuffer[i];
	
	
	//BCC
	crc = Atom_BCC((u8*)pBuf, ATOM_HEADER_LEN + PayloadLen);
	pBuf[index++] = crc & 0xff;

	//Footer
	pBuf[index++] = 'm';
        
   
 	HAL_UART_Transmit(&huart8, pBuf, index,100);
       
        
	return ret;
}

//将底层代码封装了一层再继续发送
void sendPacket(u8 MADDR, u8 classID, u8 msgID, u8 res, u8* payloadData, u16 payloadLen)
{
	AtomCmd_Compose_Send(MADDR, classID, msgID, (u8*)payloadData, payloadLen);

}

//模式配置
void Atom_switchModeReq(char mode)
{
//	u8 config_mode[8]={0x41,0x78,0xFF,0x01,0x02,0x00,0xC5,0x6D};
	if (mode == CONFIG_MODE)
	{
		sendPacket(MADDR_OUT, CLASS_ID_OPERATION_CMD, CMD_ID_SWITCH_TO_CFG_MODE, 00, NULL, 0x00);
	}
	else if (mode == MEASURE_MODE)
	{
		sendPacket(MADDR_OUT, CLASS_ID_OPERATION_CMD, CMD_ID_SWITCH_TO_MEASURE_MODE, 00, NULL, 0x00);
	}
// 	HAL_UART_Transmit(&huart8, config_mode, 8,100);
}

//等待应答
char waitAck(void (*pFunction)(char mode),uint8_t cid,uint8_t mid,uint16_t parameter)
{
	uint8_t S_count = 0;
	uint8_t timeout_count = 0;
	
    HAL_Delay(100);
	
	while(1)
    {
        //等待应答
        if( ( receiveAck == 1 ) && (receiveCid == cid) && (receiveMid == (mid | 0x80 )) )
            break;
        S_count++;
        
        /*------------------Timeout processing--------------*/
        if (S_count == 20)	        //Retransmission after 2s
        {
            S_count = 0;
            pFunction(parameter);
            timeout_count++;
            if (timeout_count == 4)	    //Unable to receive after 4 retransmissions
            {
				//////////HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
				//若超时则第一个灯亮
				return 0;
            }
        }
        HAL_Delay(100);
    }	
    if (errorCode_Ack != 0)
    {
        /*---------User Code End-------------*/
    }
    
    receiveAck = 0;
    receiveCid = 0;
    receiveMid = 0;
    
    return 1;
}

//usart6 rx handle
void UartRxMacControler(void)
{
//	memcpy(HOST_Buffer.buffer,&(UartData.Frame.Header_1),USART8_RX_NUM);
	
	memcpy(&(UartData.Frame.Header_1),HOST_Buffer.buffer,USART8_RX_NUM);
	
	if((UartData.Frame.Header_1 == 0x41) && (UartData.Frame.Header_2 == 0x78))
	{
//		UartData.Frame.tail = *(&(UartData.Frame.Header_1)+7+UartData.Frame.Payload_length);
		UartData.Frame.tail = HOST_Buffer.buffer[USART8_RX_NUM-1];
		if(UartData.Frame.tail == 0x6d)
		{		
			//测量模式，返回的Saber数据包，标志CID=0x06,MID=0x81
			if((UartData.Frame.CID == 0x06) && (UartData.Frame.MID == 0x81)) 
			{
				 //解析
				DataPacketParser();
			}
			else
			{
				//表示是从机应答
				receiveAck = 1;                                 		//Received ack flag
				receiveCid = (UartData.Frame.CID & 0x0F);          	//The high four bit is errorCode
				errorCode_Ack = ((UartData.Frame.CID &0xF0) >>4 );  
				receiveMid = UartData.Frame.MID ;      
			}		
		}
	}
}


//IMU配置
int IMU_Config(void)
{
	u8 ret;
	/******step 1:config mode*************************/
	Atom_switchModeReq(CONFIG_MODE);
	ret = waitAck(Atom_switchModeReq,CLASS_ID_OPERATION_CMD , CMD_ID_SWITCH_TO_CFG_MODE,CONFIG_MODE);
	//成功第二个灯亮
	if (ret == 1)
	{
			/////////HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	if (ret == 0)
	{return ret;}
	
	/******step 2:select packets*************************/
		// Enable packets,  use SET_BIT to enable packet and use CLEAR_BIT to disable packet.
	int bitEnable = 0;
	/*-------------------Uesr Code Begin------------------*/
	//Users can set the packet according to their own needs
	SET_BIT(bitEnable, PACKET_CAL_ACC);
	//SET_BIT(bitEnable, PACKET_CAL_ACC);
	SET_BIT(bitEnable, PACKET_CAL_GYRO);
	//SET_BIT(bitEnable, PACKET_CAL_MAG);
	SET_BIT(bitEnable, PACKET_EULER_DATA);
	//SET_BIT(bitEnable, PACKET_EULER_DATA);
	SET_BIT(bitEnable, PACKET_QUAT_DATA);
	//SET_BIT(bitEnable, PACKET_QUAT_DATA);
	/*-------------------Uesr Code End-------------------*/

	SelectPackets(bitEnable);
	ret = waitAck(SelectPackets,CLASS_ID_HOSTCONTROL_CMD , CMD_ID_SET_DATA_PAC_CFG,bitEnable);
	//成功第三个灯亮
	if (ret == 1)
	{
			////////////HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	if (ret == 0)
	{return ret;}

	return 0;
}


