#ifndef __minipc_H
#define __minipc_H
#include "stm32f4xx.h"
#include "main.h"
#include "communication.h "

typedef struct{

unsigned char 		frame_header; 		  //帧头0xFF
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//当前状态：【0 未瞄准目标 】【 1 近距离锁定目标】【2 远距离锁定目标】【3 打符模式】
	int16_t 					distance;     			//目标距离
unsigned char 		frame_tail; 	  	  //帧尾0xFE
}Minipc_Rx;

typedef struct{

unsigned char 		frame_header; 		  //帧头0xFF
unsigned char 		cmd1;     					//cmd1
unsigned char 		cmd2;     					//cmd2 
unsigned char 		frame_tail; 	  	  //帧尾0xFE
}Minipc_Tx;


extern Minipc_Rx minipc_rx;
extern Minipc_Tx minipc_tx;

extern uint8_t USART2_RX_DATA[(SizeofMinipc)];		//MiniPC
extern uint16_t USART2_RX_NUM;

void Get_MiniPC_Data(void);
void Send_MiniPC_Data(unsigned char cmd1,unsigned char cmd2,unsigned char state);


#endif
