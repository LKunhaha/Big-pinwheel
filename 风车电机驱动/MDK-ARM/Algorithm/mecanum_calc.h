#ifndef __mecanum_calc_H
#define __mecanum_calc_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
	 

typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32; 
typedef struct
{
		float vx,vy,vw;
} mecanum_t;

typedef struct
{
    float           hvx;     // forward/back
    float           hvy;     // left/right
    float           vw;      // rotate
    int16_t         wheel_speed[4];
} chassis_t;



/* Extern  ------------------------------------------------------------------*/
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W);


//void get_moto_measure(moto_measure_t *ptr,CanReceiveData_Type_t prt_Can2ReceiveData);
//void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
//void get_total_angle(moto_measure_t *p);
//void get_moto_offset(moto_measure_t *ptr,CanReceiveData_Type_t prt_Can2ReceiveData);

//void motor_move_setvmmps(float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W);
//void set_moto_current(CAN_HandleTypeDef *hcan,CanSendData_Type *  ptr_CANSendData, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
extern chassis_t chassis;
extern mecanum_t mecanum;

#ifdef __cplusplus
}
#endif
#endif 
