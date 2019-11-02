#ifndef __JUDGE_UNPACK_TASK_H__
#define __JUDGE_UNPACK_TASK_H__
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 本模块向外部提供的数据类型定义--------------------------------------------*/

/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/

/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void judgement_uart_init(void);
void judge_unpack_task(void const *argu);
/* 全局配置区----------------------------------------------------------------*/
/* communication task period time (ms) */
#define JUDGE_HUART   huart3

#define JUDGE_UART_TX_SIGNAL   ( 1 << 0 )
#define JUDGE_UART_IDLE_SIGNAL ( 1 << 1 )
#define JUDGE_DMA_FULL_SIGNAL  ( 1 << 2 )

#endif
