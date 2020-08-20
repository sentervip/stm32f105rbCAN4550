/*----------------------------------------------------------------------------
 * Name:    CanDemo.c
 * Purpose: CAN example for MCBSTM32E
 * Note(s): possible defines set in "options for target - C/C++ - Define"
 *            __USE_LCD   - enable Output on LCD
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "stm32f10x.h"                            /* STM32F10x Definitions    */
#include "CAN.h"                                  /* STM32 CAN adaption layer */
#include "stdint.h"
#include "string.h"

//led pin define
#define RCC_LED1 RCC_APB2Periph_GPIOA
#define PORT_LED1 GPIOA
#define LED1_PIN GPIO_Pin_15


#define RCC_LED2 RCC_APB2Periph_GPIOC
#define PORT_LED2 GPIOC
#define LED2_PIN GPIO_Pin_10

#define RCC_LED3 RCC_APB2Periph_GPIOC
#define PORT_LED3 GPIOC
#define LED3_PIN GPIO_Pin_11

#define RCC_LED4 RCC_APB2Periph_GPIOC
#define PORT_LED4 GPIOC
#define LED4_PIN GPIO_Pin_12


//swtich pin define
#define RCC_KEY1 RCC_APB2Periph_GPIOB
#define PORT_KEY1 GPIOB
#define KEY1_PIN GPIO_Pin_12

#define RCC_KEY2 RCC_APB2Periph_GPIOB
#define PORT_KEY2 GPIOB
#define KEY2_PIN GPIO_Pin_13

#define RCC_KEY3 RCC_APB2Periph_GPIOB
#define PORT_KEY3 GPIOB
#define KEY3_PIN GPIO_Pin_14

#define RCC_KEY4 RCC_APB2Periph_GPIOB
#define PORT_KEY4 GPIOB
#define KEY4_PIN GPIO_Pin_15

#define LED1_ON() GPIO_ResetBits(PORT_LED1,LED1_PIN)
#define LED1_OFF() GPIO_SetBits(PORT_LED1,LED1_PIN)

#define LED2_ON() GPIO_ResetBits(PORT_LED2,LED2_PIN)
#define LED2_OFF() GPIO_SetBits(PORT_LED2,LED2_PIN)

#define LED3_ON() GPIO_ResetBits(PORT_LED3,LED3_PIN)
#define LED3_OFF() GPIO_SetBits(PORT_LED3,LED3_PIN)

#define LED4_ON() GPIO_ResetBits(PORT_LED4,LED4_PIN)
#define LED4_OFF() GPIO_SetBits(PORT_LED4,LED4_PIN)


#define READ_SWTICH_1() (!GPIO_ReadInputDataBit(PORT_KEY1,KEY1_PIN))
#define READ_SWTICH_2() (!GPIO_ReadInputDataBit(PORT_KEY2,KEY2_PIN))
#define READ_SWTICH_3() (!GPIO_ReadInputDataBit(PORT_KEY3,KEY3_PIN))
#define READ_SWTICH_4() (!GPIO_ReadInputDataBit(PORT_KEY4,KEY4_PIN))


uint32_t CAN1_STATUS;
uint32_t CAN2_STATUS;

CanTxMsg msg1;
CanTxMsg msg2;

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
        
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {

  msTicks++;                        /* increment counter necessary in Delay() */
	
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay_Ms (uint32_t dlyTicks) {
  uint32_t curTicks;
	curTicks = 0;
	msTicks = 0;
//  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}



//LED灯和按键引脚初始化
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	
	RCC_APB2PeriphClockCmd( RCC_KEY1|RCC_KEY2|RCC_KEY3|RCC_KEY4 , ENABLE); 	
	RCC_APB2PeriphClockCmd( RCC_LED1|RCC_LED2|RCC_LED3|RCC_LED4 , ENABLE); 	
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE); 	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	
	
	/**
	*	LED 
	*/					 
	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED4, &GPIO_InitStructure);
	
	/**
	*	USB ENABLE -> PC5
	*/		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);
	GPIO_Init(GPIOC, &GPIO_InitStructure);


		/**
	*	DI -> PB12-PPB15
	*/	
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY4, &GPIO_InitStructure);

	LED1_ON();
	LED2_ON();
	LED3_ON();
	LED4_ON();

}

//TTL串口初始化
void Uart2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  USART_InitTypeDef USART_InitStructure; 
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
 
  /*
  *  USART2_TX -> PA2 , USART2_RX ->	PA3
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  USART_InitStructure.USART_BaudRate = 19200; 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure); 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_ClearFlag(USART2,USART_FLAG_TC);
  
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  		
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  //  启动串口
  USART_Cmd(USART2, ENABLE);   
	
}


/*******************************************************************************
* Function Name  : USART2_IRQHandler 串口中断接收函数
* Description    : \
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t aucRecv_Data_temp;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  			
    { 
			aucRecv_Data_temp = USART2->DR; 
		
		}
}


//串口发送
void USART2_Send(void)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
		 USART_SendData(USART2, 0x55);
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void)  {

	uint8_t i;
	SysTick_Config(72000000 / 1000);         /* SysTick 100 usec IRQ       */

	GPIO_Configuration();

	CAN_GPIO_Config();
	CAN1_Config(125);//波特率125K
	CAN1_Filter_Config(0,0,0,0);

	  /* Transmit */
	msg1.StdId = 0x123;
	msg1.ExtId = 0x1234;
	msg1.RTR = CAN_RTR_DATA;
	msg1.IDE = CAN_ID_STD;
	msg1.DLC = 8;
	memset(msg1.Data,0x11,0x08);

	
	
	CAN2_Config(125);//波特率125K
	CAN2_Filter_Config(0,14,0,0);

		  /* Transmit */
	msg2.StdId = 0x321;
	msg2.ExtId = 0x4321;
	msg2.RTR = CAN_RTR_DATA;
	msg2.IDE = CAN_ID_STD;
	msg2.DLC = 8;
	memset(msg2.Data,0x22,0x08);
	
	Uart2_Init();
	
	while(1)
	{
		//按键1按下发送数
			if(READ_SWTICH_1())
			{			
					Delay_Ms(10);
					if(READ_SWTICH_1())
					{				
						LED1_ON();
						Delay_Ms(10);
						CAN1_wrMsg(msg1);
						LED1_OFF();
					}
			}
			
			//按键2按下发送数
			if(READ_SWTICH_2())
			{		
					Delay_Ms(10);
					if(READ_SWTICH_2())
					{					
						LED2_ON();
						Delay_Ms(10);
						CAN2_wrMsg(msg2);
						LED2_OFF();
					}
			}
			
			
			if(READ_SWTICH_3())
			{			
					Delay_Ms(10);
					if(READ_SWTICH_3())
					{	
						
						
						LED3_ON();
						USART2_Send();
						Delay_Ms(10);
						LED3_OFF();
					}
			}
			
		
			if(READ_SWTICH_4())
			{			
					Delay_Ms(10);
					if(READ_SWTICH_4())
					{	
						LED4_ON();
						USART2_Send();
						Delay_Ms(10);
						LED4_OFF();
					}
			}
			
			//can1 收到id：321  数据 22 22 22 22 22 22 22 22
			if(CAN1_RxMessage.StdId == 0x321)
			{			
				for(i=0;i<8;i++)
				{
					if(CAN1_RxMessage.Data[i] != 0x22)
					{
						break;
					}
				}
				if(i==8)
				{
					LED1_ON();
					Delay_Ms(10);
					memset(&CAN1_RxMessage,0,sizeof(CAN1_RxMessage));
			
					LED1_OFF();
				}			
			}
			
			//can2 收到id：123  数据 11 11 11 11 11 11 11 11
			if(CAN2_RxMessage.StdId == 0x123)
			{			
				for(i=0;i<8;i++)
				{
					if(CAN2_RxMessage.Data[i] != 0x11)
					{
						break;
					}
				}
				if(i==8)
				{
					LED2_ON();
					Delay_Ms(10);
					memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
			
					LED2_OFF();
				}			
			}
			
			


	  }
}
