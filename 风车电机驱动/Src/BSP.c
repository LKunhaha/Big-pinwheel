/*************************************************************************************
*	@file			BSP.c
* @author	 	
*	@version 	V1.0
*	@date			
* @brief		NONE
*************************************************************************************//* Includes ------------------------------------------------------------------------*/
#include "BSP.h"
/* External variables --------------------------------------------------------------*/
volatile unsigned long long FreeRTOSRunTimeTicks;
extern CAN_HandleTypeDef hcan1;

extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart4_rx;
extern DMA_HandleTypeDef  hdma_usart5_rx;
extern DMA_HandleTypeDef  hdma_usart6_rx;

/* USER CODE END PFP */
uint8_t USART2_RX_CAN_fc[4];
/* USER CODE BEGIN 0 */

/* Internal variables --------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------*/
void ConfigureTimerForRunTimeStats(void)  
{
	FreeRTOSRunTimeTicks = 0;
	//MX_TIM3_Init(); //周期50us，频率20K
}

/**
	**************************************************************
	** Descriptions:初始化
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void BSP_Init(void)
{
	/*引脚*/
  MX_GPIO_Init();
  MX_DMA_Init();
  /*CAN*/
  MX_CAN2_Init();
  MX_CAN1_Init();
  /*串口*/
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  /*ADC*/
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /*定时器*/
  MX_TIM4_Init();
  MX_TIM2_Init();
  /*开启  */
  HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);
	/*开启中断*/ 
	HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,4);	
//  HAL_UART_Receive_DMA(&huart2,USART2_RX_CAN_fc,4);			
//  HAL_UART_Receive_DMA(&huart3,LightBand2,sizeofLB); 	  	
//  HAL_UART_Receive_DMA(&huart4,buff,20); 				 	  
//	HAL_UART_Receive_DMA(&huart5,QR_Buff,11);
//  HAL_UART_Receive_DMA(&huart6,color_buff,54);      
	/*关闭半传输完成中断*/
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);	//关闭串口1半传输完成中断
//	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);	//关闭串口2半传输完成中断
//	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);	//关闭串口3半传输完成中断
//	__HAL_DMA_DISABLE_IT(&hdma_usart4_rx,DMA_IT_HT);	//关闭串口4半传输完成中断
//	__HAL_DMA_DISABLE_IT(&hdma_usart5_rx,DMA_IT_HT);  //关闭串口5半传输完成中断
//	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);	//关闭串口6半传输完成中断
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_Delay(1000);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




