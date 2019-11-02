
/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
 *******************************************************************************
  文 件 名   : pid.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : pid计算环节
  函数列表   : float pid_calc(pid_t* pid, float get, float set)
							 PID_struct_init(	pid_t* pid,
																uint32_t mode,
																uint32_t maxout,
																uint32_t intergral_limit,   
																float 	kp, 
																float 	ki, 
																float 	kd)
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "pid.h"
#include "Power_restriction.h"
#include <math.h>
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/
#define ABS(x)		((x>0)? (x): (-x)) 

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/
#if 0
pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy61 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
pid_t pid_3508_pos;      		//底盘电机位置环
pid_t pid_3508_spd[4];			//底盘电机速度环
pid_t pid_3508_current[4];	//底盘电机电流环控制
#endif
/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 函数原型声明 ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: 限幅函数
	** Input: 	
	**			   a :要限制的变量的指针
	**				ABS_MAX :幅值
	** Output: NULL
	**************************************************************
**/

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/**
	**************************************************************
	** Descriptions: pid参数初始化
	** Input: 	
	**			   pid_t *    pid,    					pid的指针
	**    	   uint32_t 	mode, 						pid模式
	**  	 	 	 uint32_t	  maxout,						限幅值
	**    		 uint32_t 	intergral_limit,	积分环限幅
	**   		 	 float 	    kp, 
	**   			 float   		ki, 
	**   			 float 			kd
	** Output: NULL
	**************************************************************
**/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{   
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;    
}
/**
	**************************************************************
	** Descriptions: 限幅中途更改参数设定函数(调试)
	** Input: 	
  **				*pid：要求更改的pid的指针	
	**			   kp :
	**				 ki :
  ** 				 kd ：
	** Output: NULL
	**************************************************************
**/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
	**************************************************************
	** Descriptions: pid计算函数
	** Input: 	
  **				 *pid :		要进行计算的pid的指针
	**					get :   实际值
	**					set :   目标值
	** Output: NULL
	**************************************************************
**/
float pid_calc(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}

/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_sp_calc(pid_t* pid, float get, float set, float gyro){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
				if(fabs(pid->i) >= 0.001f)
					pid->iout += pid->i * pid->err[NOW];
				else
					pid->iout = 0;
        pid->dout = -pid->d * gyro/100.0f;	
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        abs_limit(&(pid->iout), pid->IntegralLimit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        abs_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}
/**
	**************************************************************
	** Descriptions: pid参数初始化
	** Input: 	
	**			   pid_t *    pid,    					pid的指针
	**    	   uint32_t 	mode, 						pid模式
	**  	 	 	 uint32_t	  maxout,						限幅值
	**    		 uint32_t 	intergral_limit,	积分环限幅
	**   		 	 float 	    kp, 
	**   			 float   		ki, 
	**   			 float 			kd
	** Output: NULL
	**************************************************************
**/void PID_struct_init(
											pid_t* pid,
											uint32_t mode,
											uint32_t maxout,
											uint32_t intergral_limit,   
											float 	kp, 
											float 	ki, 
											float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);	
}



void pid_test_init(){
	

	//为了解决上位机调参的时候第一次赋值的时候清零其他参数， 应该提前把参数表填充一下！

}
