/* 包含头文件----------------------------------------------------------------*/
#include "judge_unpack_task.h"
#include "protocol.h"
#include "communicate.h"
#include "judgement_info.h"
#include "infantry_info.h"
#include "info_interactive.h"
#include "cmsis_os.h"
#include "string.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
extern uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
extern uint8_t pc_dma_rxbuff[2][UART_RX_DMA_SIZE];
static uart_dma_rxdata_t judge_rx_obj;
static unpack_data_t judge_unpack_obj;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/

/* 外部变量声明--------------------------------------------------------------*/
UBaseType_t judge_comm_surplus;

/* 外部函数原型声明----------------------------------------------------------*/

/* 内部变量------------------------------------------------------------------*/

/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	judge_unpack_task(void const *argu)
	*	@param
	*	@supplement	裁判系统解析任务
	*	@retval	
****************************************************************************************/
void judge_unpack_task(void const *argu)
{
  osEvent event;
  
  /* open judge uart receive it */
//  judgement_uart_init();
  
  while (1)
  {
    event = osSignalWait(JUDGE_UART_TX_SIGNAL | \
                         JUDGE_UART_IDLE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      //receive judge data puts fifo
      if (event.value.signals & JUDGE_UART_IDLE_SIGNAL)
      {
        dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);
        unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);
      }
      
      //send data to judge system
//      if (event.value.signals & JUDGE_UART_TX_SIGNAL)
//      {
//        send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
//      }
      
    }
    
//    judge_comm_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

