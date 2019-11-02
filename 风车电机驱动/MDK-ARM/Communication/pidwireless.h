#ifndef __PIDWIRELESS_H
#define __PIDWIRELESS_H
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "pid.h"
#include "protocol.h"
#include "communication.h "



enum UART_State
{
	UART_Waiting = 0,
	UART_Receiving,
	UART_Success,
	UART_Failed,
};

/*
此文件中定义了三个结构体，分别为PID_struct，MsgsFrame_struct，pid_t，
其中第一个结构体是囊括在第二个结构体中，程序运行时的数据传递只是在第二个和第三个结构体中，
所以定义结构体变量时只会定义第二三个结构体变量，第一个结构体在第二个结构体中定义。



工作量只有把pid结构的数据传递进来，方法有三种，
第一增加两个pid结构体之间进行赋值的环节
第二是将源程序的结构体改成此调参的结构体
第三是将此调参pid结构体换成源程序的pid结构体
*/
#define SizeofPid  100
extern uint8_t  UsartRx_Info[SizeofPid];//无线调参
extern uint8_t  UsartTx_Info[SizeofPid];//无线调参



//pid结构体，一共九项数据，完全是为了MsgsFrame_struct结构体服务。
typedef struct{
    unsigned char motor_ID;
    unsigned char PID_Mode;//增量式：1；位置式：0
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Kp_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Ki_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Kd_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }P_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }I_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;  
    }D_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }PID_out_max;
} PID_struct;
//定义了所有要发送的所有数据，包括pid数据和其他通信协议。
typedef struct{
    unsigned char SOF;
    unsigned char Data_Length;
    unsigned char seq;
    unsigned char crc8;
    unsigned char cmd_ID;
    PID_struct PID;
    union{
        unsigned char tempChar[2];
        int tempInt;
    }crc16;
}MsgsFrame_struct;


void PID_ParamsSet(MsgsFrame_struct *pidSet);
void PID_ParamsUpload(unsigned char motor_ID);
void PID_Regulator_Decode(void);
void PID_UART_IRQHandler(UART_HandleTypeDef * huart,uint8_t Res);

#endif
