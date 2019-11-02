

-时钟          输入					PLLCK系数					        输出					  
168MHz			   HSE   24MHz			M=24，N=336，P=2			SYSCLK					168MHz
                                              APB1-peripheral clock	  42MHz
                                              APB1-timer clock		    84MHz
                                              APB2-peripheral clock 	84MHz
                                              APB2-timer clock		    168MHz

GPIO：
	引脚		功能/备注	
  PB14    LED1
  PE15    LED2
  PE14    LED3
  PF12    LED4//
  PF11    LED5//
  PB2     LED6
  PB1     LED7
  PB0     LED8
  
  PC1     KEY1  上拉
  PC2     KEY2  上拉
  PA6     KEY3  上拉
  PA5     KEY4  上拉


-串口：			-  引脚		  	功能/备注				         DMA_request        DMA-stream           波特率              	优先级

-USART1   -  PB6/PB7   UART1_TX/UART1_RX      USART1_RX        DMA2-Stream2          115200   				  (5，0)/dma对应相同
-USART2   -  PD5/PD6   UART2_TX/UART2_Rx      USART2_RX				 DMA1_Stream5					 115200
-USART3   -  PB10/PB11 UART3_TX/UART3_RX	    USART3_RX        DMA1-Stream1          115200					    NULL	
-UART4    -  PC10/PC11 UART4_TX/UART4_Rx      UART4_RX         DMA1-Stream2          115200
-USART5   -  PC12/PD2  UART5_TX/UART5_RX      USART5_RX				 DMA1-Stream0          115200				   	 (6,0) /
-USART6   -  PG14/PG9	 UART6_TX/UART6_RX      USART6_RX				 DMA2-Stream1          115200				   	 (6,0) /

-定时器             引脚       		                 功能/备注  			  模式			         分频系数			   重载系数		  	   计数模式
  
  
-CAN：
    PD0     ------> CAN1_RX																															(6,0)
    PD1     ------> CAN1_TX 																												
	
    PB12    ------> CAN2_RX																														  (6,0)
    PB13    ------> CAN2_TX 


  