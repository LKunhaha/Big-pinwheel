#ifndef __TEST__TASK_H
#define __TEST__TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "main.h"
#include "can.h"
/* Private function prototypes -----------------------------------------------*/
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;


typedef struct 
{
	
 uint8_t ID;
 uint8_t state;
 uint8_t flag;       //判断是否亮过灯
 uint8_t on_off;     //判断此时灯条的状态
}LED;








void TestTask_EN(void const * argument);
void TestTask_MA(void const * argument);
void Motor_roll(CAN_HandleTypeDef * hcan,signed int speed);


#endif