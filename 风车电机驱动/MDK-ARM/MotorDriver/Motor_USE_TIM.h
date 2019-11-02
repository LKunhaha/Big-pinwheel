#ifndef  __Motor_USE_TIM_H
#define  __Motor_USE_TIM_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "tim.h"

#define highspeed 2000
#define lowspeed  1900

void TIM5_PWM_Init(uint32_t speed1,uint32_t speed2);
void GUN_Init(void);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Friction_Wheel_Motor_Stop(void);


#endif
