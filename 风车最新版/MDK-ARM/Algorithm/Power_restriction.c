/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
 *******************************************************************************
  文 件 名   : Power_restriction.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : 底盘功率限制
  函数列表   :void power_limit(float  Current_get[4])


*******************************************************************************/
/*
* 电压输出与电流输入的关系:
*			VOUT = VOUT(Q) + Vsens*I;
*	VOUT(Q)为静态输出电压，即VCC/2，也就是无输入时，输出的电压。Vsens是传感器灵敏度
*（使用的型号的系数是40MV/A）I的方向是从IP+流向IP-的电流
*	eg:
*			VCC为5V，I电流为10A，那么输出即为5V / 2 + 40MV/A * 10A = 2.9V
*			VCC为3.3V，I电流为10A，那么输出即为3.3V / 2 + 40MV/A*10A = 2.05V
* 结果：
*			I = (VOUT - VOUT(Q))/Vsens
*			I = (V_get - 2.5)/0.04; 	  //接5V电压
*			I = (V_get - 1.65)/0.04;		//接3.3V电压
*/
/* 包含头文件 ----------------------------------------------------------------*/
#include "Power_restriction.h"
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/
#define MyAbs(x) ((x > 0) ? (x) : (-x))
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/

/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/
/*
*
*	数据说明:ADC缓存数据 , 乘以系数0.8058608{3300/4095}
*  			  得到实际电压值
*	采样时间:采集一次数据的时间是(转换周期+采样周期)/时钟(12位采集是15个周期)
*				  所以配置的采样时间是(12+3)/(84/4)M = 0.7142us
*	   Note:实际调用数据的间隔最小5ms，所以创建一个长度为10的数组，每次使用数据的时候
*		   		对10个数据取平均值，然后再创建一个10位的数组存储历史数据，进行窗口滑动滤
*					波 (因为现在还没有硬件设计，暂时用这种方法简单滤波),数据有效时长延长到0.7ms
*/
uint32_t  uhADCxConvertedValue[10] = {0};  
Limit  limit = {.PowerRemain_Calculat = 60};
Current_GET  current_get = {0};
MyTimeTick  time_for_limit = {0}; //限制时间计时
MyTimeTick  time_for_RP = {0};  //remain power 缓存能量计算计时
//测试变量

/* 函数原型声明 ----------------------------------------------------------*/


/*
** Descriptions: 交换两变量值
** Input: 需要交换的两个变量float型
** Output: NULL
*/
void swap(float *a,float *b)
{
 float c;
 c=*a;*a=*b;*b=c;
}
/*
** Descriptions: 将数组分成两部分，前一部分的值均比后一部分值小
** Input: 要求的数据的开头和末尾
** Output: 返回分界点
*/
int Partition(float data[],int low,int high)
{
 float pivokey;
 pivokey=data[low];
 while(low<high)
 {
  while(low<high&&data[high]>=pivokey)
   high--;
  swap(&data[low],&data[high]);

  while(low<high&&data[low]<=pivokey)
   low++;
  swap(&data[low],&data[high]);
 }
 return low;
}
/*
** Descriptions: 进行的递归的调用，达到排序的目的
** Input: 需要进行排序的数组指针，以及对应的范围
** Output: NULL
*/
void QSort(float data[],int low,int high)
{
 if(low<high)
 {
  int pivokey=Partition(data,low,high);
  QSort(data,low,pivokey-1);
  QSort(data,pivokey+1,high);
 }
}
/*
** Descriptions: 求得缓存的中值
** Input:对应的缓存指针，缓存的数组长度需要为10
** Output:中值
*/
float Median_value_fliter(uint32_t *buff,int length)
{	
	uint32_t mybuff[length];
	memcpy(mybuff,buff,length);
	QSort((float*)mybuff, 0, length-1);
	return buff[(int)((length-1)/2)];
}
/*
** Descriptions: 中值平均滤波
** Input:对应的缓存指针，缓存的数组长度需要为10
** Output:滤去最大最小值的平均值
*/
float Median_average_fliter(uint32_t *buff,int length)
{
	int16_t sum = 0;
	uint32_t mybuff[length];
	memcpy(mybuff,buff,length);
	QSort((float*)mybuff, 0, length-1);
	for(uint8_t i = 1;i < length-1;i++)
	{
		sum += mybuff[i];
	}
	sum = sum/(length-2);
	return sum;
}
/*
** Descriptions: 均值滤波
** Input: 需要滤波的缓存的指针
** Output: 滤波结果
*/
float Average_value_fliter(uint32_t *buff)
{
	float sum = 0;
	uint32_t mybuff[10];
	memcpy(mybuff,buff,10);
	for(uint8_t i = 0;i < 10;i++ )
	{
		sum += mybuff[i];
	}
	sum *= 0.1f;
	return sum;
}
/*
** Descriptions: 窗口滑动滤波
** Input: 需要滤波的缓存的指针
** Output: 滤波结果
*/
float Window_sliding_filter(float *buff)
{
	float sum = 0;
	for(uint8_t i = 0; i < 10; i++) {
	buff[i] = buff[i+1]; // 所有数据左移，低位仍掉
	sum += buff[i];
  }
	
	return sum;
}

/*
** Descriptions: 一阶低通滤波
** Input: 
** Output: 滤波结果
*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}

/*
** Descriptions: 限幅滤波
** Input:   相邻的两次数据
** Output: 滤波结果
*/
float Limit_filter(float oldData,float newData,float val)
{
	if(abs(newData-oldData)>val)
		{
			return oldData;
		}
	else
		{
			return newData;
		}
}

/*
** Descriptions: 粗略测量任务时间(s)
** Input: 时间结构体的指针
** Output: NULL
*/
void	MyTime_statistics(MyTimeTick *time_tick)
{

	time_tick->time_now = HAL_GetTick();
	if(time_tick->time_last == 0)//避免第一次计数的时候last为零
	{
		time_tick->time = time_tick->time_now - time_tick->time_now;
	}else
	{
		time_tick->time = time_tick->time_now - time_tick->time_last;		
	}
	time_tick->time_last = time_tick->time_now;
	//切记清零总时间
	time_tick->total_time_ms += time_tick->time;
	time_tick->total_time_s = time_tick->total_time_ms * 0.001f;
}
/*
** Descriptions: 清空时间结构体
** Input: 
**				MyTimeTick *: 		时间结构体的指针
**				flag:  选择清空的内容
**								flag = 1  清空全部
**								flag = 2	清空总时间(防止连续调用时的计时溢出错误)
** Output: NULL
*/
void MyTime_memset(MyTimeTick *time_tick ,char flag)
{
	if(flag == 1)
	{
		memset(time_tick,0,sizeof(MyTimeTick));
	}else if(flag == 2)
	{
		time_tick->total_time_ms = 0;
		time_tick->total_time_s = 0;
	}
	
}

/*
** Descriptions: ADC数据采集并滤波
** Input: NULL
** Output: 数据采集值
*/
void Get_ADC_Value(void)
{
		uint32_t *buff = uhADCxConvertedValue;
		//第一个滤波要关闭ADC的dma
		int getbuff = 0;
		for(uint8_t i = 0;i < 10;i++)
		{
			getbuff += buff[i];
		}
		/*current_get.CurrentBuff1_get =  Median_value_fliter(buff,10);
			current_get.CurrentBuff1_get =  Median_average_fliter(buff,10);*/
		current_get.CurrentBuff1_get =  getbuff * 0.1f;
		current_get.CurrentBuff2_fliter[10] = current_get.CurrentBuff1_get;
		current_get.CurrentBuff2_get = Window_sliding_filter(current_get.CurrentBuff2_fliter)*0.1f;
	
		if(current_get.Current_Offset_num > 200)
		{
			
			current_get.CurrentCalculat = (current_get.CurrentBuff2_get * (0.00080566f) - 2.50f) * 25.0f -
																		 current_get.Current_Offset;
		}
		else
		{
			current_get.Current_Offset_num++;
			current_get.CurrentCalculat = (current_get.CurrentBuff2_get * (0.00080566f) - 2.50f) * 25.0f;
			
			if(current_get.Current_Offset_num > 50)
			{		
				current_get.Current_Offset += current_get.CurrentCalculat - current_get.Current_Referee;
			}
			if(current_get.Current_Offset_num > 200)
			{				
				current_get.Current_Offset = current_get.Current_Offset/150.0f;
			}
		}

}

/*
** Descriptions: 缓存能量计算
** Input: NULL
** Output: NULL
*/

void Remain_Power_Calculate(void)
{
		/*采集时间*/
		MyTime_statistics(&time_for_RP);
	
		/*能量缓存计算*/
		if(limit.Power_Calculat >80)
		{
			limit.PowerRemain_Calculat -= (limit.Power_Calculat - 80) * time_for_RP.time * 0.01f;
		}
		if(limit.Power_Calculat <80)
		{
			limit.PowerRemain_Calculat = 60-(limit.Power_Calculat - 80) * time_for_RP.time * 0.01f;
		}
		
		/*清空总时间*/
		MyTime_memset(&time_for_RP,2);		
		
		/*恢复能量缓存*/
		VAL_LIMIT(limit.PowerRemain_Calculat, 0.0f, 60.0f);

}

/*
** Descriptions: 功率限制
** Input: NULL
** Output: 功率限制值
*/

/*
*	如何充分利用裁判系统传回的功率和能量缓存
*
*/
void power_limit(float  Current_get[4])
{
		
		float total_current = 0;
		float T0 = 0;
		float P0 = 0;
		float RE = 0;
		float V0 = 0;
		total_current = MyAbs(Current_get[0]) + MyAbs(Current_get[1])+\
										MyAbs(Current_get[2]) + MyAbs(Current_get[3]);
		
		/*电流采集*/
		Get_ADC_Value();

		/*功率计算*/
		if(limit.Volt_Referee != 0)//防止裁判系统失效
		{
			limit.Power_Calculat = current_get.CurrentCalculat * limit.Volt_Referee;

		}else
		{
			limit.Power_Calculat = current_get.CurrentCalculat * 23.0f;		

		}
		
		/*能量缓存计算*/
		Remain_Power_Calculate();
		
		/*功率限制*/

	
//			printf("\n power:%4f .\n",limit.Power_Calculat);
}

