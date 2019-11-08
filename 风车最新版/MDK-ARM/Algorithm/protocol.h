#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "String.h"



#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#define PROTOCAL_FRAME_MAX_SIZE  200

/** 
  * @brief  frame header structure definition
  */
//内部变量声明
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
//外部变量声明

typedef __packed struct
{
	uint8_t SOF;          //数据起始字节，固定为0xA5          
	uint16_t DataLength;  //数据长度
	uint8_t Seq;          //包序号
	uint8_t CRC8;         //帧头CRC校验
}tFrameHeader;//帧头

typedef enum                 //枚举类型，命令id_变更
{
	GameInfo = 0x0001,      //比赛机器人状态    发送频率 10 Hz
	DamagedData = 0x0002,             //伤害数据，实时发送
	ShootData = 0x0003,                //射击数据，实时发送
	PowerANDHeat = 0x0004,							//功率和热量数据50hz频率
	RfidData = 0x0005,								//场地交互数据检测到RFID后10hz周期发送
	GameData = 0x0006,								//比赛结果数据
	BuffChangeData = 0x0007,					//buff状态任意buff状态改变后发送一次
	PositionData = 0x0008,						//机器人位置信息和枪口朝向位置
	SelfDefinedData =0x0100, //学生自定义数据      id号_变更  
	Wrong = 0x1301       //枚举无效，只是为了使该枚举大小为2字节
}tCmdID; 

typedef __packed struct
{
	uint16_t stageRemainTime;       //比赛剩余时间（从倒计时三分钟开始计算，单位 s）
	uint8_t gameProgress;     //比赛进程
	uint8_t roboLevel;        //机器人等级
	uint16_t remainHp;        //剩余血量
	uint16_t maxHp;           //最大血量
}tGameInfo; //比赛机器人状态（0x0001）

typedef __packed struct
{

	uint8_t armorType :4;
	uint8_t hurtType : 4;
	
}tDamagedData;   //伤害数据(0x002)

typedef __packed struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float  bulletSpeed;
	
}tShootData;   //射击数据(0x003)

typedef __packed struct
{
	
 float chassisVolt;
 float chassisCurrent;
 float chassisPower;
 float chassisPowerBuffer;
	uint16_t shootHeat0;
	uint16_t shootHeat1;
	
}tPowerANDHeat;   //功率和热量数据50hz频率(0x004)


typedef __packed struct
{
	
	uint8_t cardType;
	uint8_t cardldx;

}tRfidData;							//场地交互数据检测到RFID后10hz周期发送(0x005)

typedef __packed struct
{
	
	uint8_t winner;

}tGameData;								//比赛结果数据(0x006)

typedef __packed struct
{
	
	uint16_t buffMusk;

}tBuffChangeData;					//buff状态任意buff状态改变后发送一次(0x007)

typedef __packed struct
{
	
  float x;
	float y;
	float z;
  float yaw;
	
}tPositionData;						//机器人位置信息和枪口朝向位置(0x008)
typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t data4;
}tSelfDefine;                     //自定义数据(0x100)


typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	__packed union 
	{
		tGameInfo    			GameInfo;  				  //比赛机器人状态    发送频率 10 Hz
		tDamagedData  		DamagedData;        //伤害数据，实时发送
		tShootData     		ShootData;          //射击数据，实时发送
		tPowerANDHeat			PowerANDHeat;				//功率和热量数据50hz频率
		tRfidData					RfidData;						//场地交互数据检测到RFID后10hz周期发送
		tGameData					GameData;						//比赛结果数据
		tBuffChangeData		BuffChangeData;			//buff状态任意buff状态改变后发送一次
		tPositionData			PositionData;				//机器人位置信息和枪口朝向位置
		tSelfDefine       SelfDefinedData; 		//学生自定义数据      
	}Data;
	uint16_t        CRC16;         //之前所有数据CRC校验   注意此数据和之前的数据可能不连续，所以不要直接使用，若需要直接使用，必须在此赋值
}tFrame;  //数据帧


//typedef __packed struct
//{
//	tFrameHeader    FrameHeader;
//	tCmdID          CmdID;
//  tSelfDefine     SelfDefine;
//	uint16_t        CRC16;         //之前所有数据CRC校验   注意此数据和之前的数据可能不连续，所以不要直接使用，若需要直接使用，必须在此赋值
//}tFrame;  //数据帧


typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tGameInfo       GameInfo;   
	uint16_t        CRC16;         //数据CRC校验
}tGameInfoFrame;  //比赛机器人状态（0x0001）
typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tDamagedData    DamagedData;   
	uint16_t        CRC16;         //数据CRC校验
}tDamagedDataFrame; //实时血量变化数据（0x0002）
typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tShootData      ShootData;   
	uint16_t        CRC16;         //数据CRC校验
}tShootDataFrame;    //射击数据(0x003)  

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tPowerANDHeat   PowerANDHeat;   
	uint16_t        CRC16;         //数据CRC校验
}tPowerANDHeatFrame;   //功率和热量数据50hz频率(0x004)    

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tRfidData   		RfidData;   
	uint16_t        CRC16;         //数据CRC校验
}tRfidDataFrame;			//场地交互数据检测到RFID后10hz周期发送(0x005)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tGameData   		GameData;   
	uint16_t        CRC16;         //数据CRC校验
}tGameDataFrame;								//比赛结果数据(0x006)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tBuffChangeData BuffChangeData;   
	uint16_t        CRC16;         //数据CRC校验
}tBuffChangeDataFrame;					//buff状态任意buff状态改变后发送一次(0x007);	

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tPositionData   PositionData;   
	uint16_t        CRC16;         //数据CRC校验
}tPositionDataFrame;					 	//机器人位置信息和枪口朝向位置(0x008)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tSelfDefine     SelfDefine;   
	uint16_t        CRC16;         //数据CRC校验
}tSelfDefineFrame;               //自定义数据(0x100);	


//内部函数
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t  append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint16_t append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

//接口函数
void Send_FrameData(tCmdID cmdid, uint8_t * pchMessage,uint8_t dwLength);
void sendata(void);

#endif
