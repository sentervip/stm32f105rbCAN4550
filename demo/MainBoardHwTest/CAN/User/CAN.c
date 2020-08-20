/*----------------------------------------------------------------------------
 * Name:    CAN.c
 * Purpose: low level CAN functions
 * Note(s): 
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

#include "STM32F10x.h"
#include "CAN.h"
#include "stdint.h"

/* Private typedef -----------------------------------------------------------*/
#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

/* Private define ------------------------------------------------------------*/
#define __CAN1_USED__
#define __CAN2_USED__

//#define STM32F10X_CL

#define __REMAP_CAN1__
#define __REMAP_CAN2__

/* Define the STM32F10x hardware depending on the used evaluation board */

#define CAN_BAUDRATE  1000      /* 1MBps   */
/* #define CAN_BAUDRATE  500*/  /* 500kBps */
/* #define CAN_BAUDRATE  250*/  /* 250kBps */
/* #define CAN_BAUDRATE  125*/  /* 125kBps */
/* #define CAN_BAUDRATE  100*/  /* 100kBps */ 
/* #define CAN_BAUDRATE  50*/   /* 50kBps  */ 
/* #define CAN_BAUDRATE  20*/   /* 20kBps  */ 
/* #define CAN_BAUDRATE  10*/   /* 10kBps  */ 

#if defined __CAN1_USED__
  #define RCC_APB2Periph_GPIO_CAN1    RCC_APB2Periph_GPIOB
  #define GPIO_Remapping_CAN1         GPIO_Remap1_CAN1
  #define GPIO_CAN1                   GPIOB
  #define GPIO_Pin_CAN1_RX            GPIO_Pin_8
  #define GPIO_Pin_CAN1_TX            GPIO_Pin_9
	
#endif

#if defined __CAN2_USED__  
  #define RCC_APB2Periph_GPIO_CAN2    RCC_APB2Periph_GPIOB
  #define GPIO_Remapping_CAN2         GPIO_Remap_CAN2
  #define GPIO_CAN2                   GPIOB  
  #define GPIO_Pin_CAN2_RX            GPIO_Pin_5
  #define GPIO_Pin_CAN2_TX            GPIO_Pin_6
#endif 



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg CAN1_TxMessage;
CanTxMsg CAN2_TxMessage;
CanRxMsg CAN1_RxMessage;
CanRxMsg CAN2_RxMessage;
/* Private function prototypes -----------------------------------------------*/
void NVIC_Config(void);
void CAN_Config(void);
void LED_Display(uint8_t Ledstatus);
void Init_RxMes(CanRxMsg *RxMessage);
void Delay(void);


/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN_GPIO_Config(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	
	  /* GPIO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	#ifdef  __CAN1_USED__	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		#ifdef __REMAP_CAN1__
		GPIO_PinRemapConfig(GPIO_Remapping_CAN1 , ENABLE);
		#endif
		 /* Configure CAN pin: RX */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN1_RX;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIO_CAN1, &GPIO_InitStructure);
		
		/* Configure CAN pin: TX */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN1_TX;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIO_CAN1, &GPIO_InitStructure);
	#endif


	#ifdef  __CAN2_USED__				
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN2, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
		#ifdef __REMAP_CAN2__
		GPIO_PinRemapConfig(GPIO_Remapping_CAN2 , ENABLE);
		#endif
		 /* Configure CAN pin: RX */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN2_RX;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIO_CAN2, &GPIO_InitStructure);
		
		/* Configure CAN pin: TX */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN2_TX;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIO_CAN2, &GPIO_InitStructure);
	#endif
	
	 
 
}


/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN1_Config(uint32_t baud)
{  
  /* CAN register init */
  CAN_DeInit(CAN1);
//  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE; /* 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳 */
  CAN_InitStructure.CAN_ABOM = DISABLE; /* 自动离线禁止，自动离线：一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出 */
  CAN_InitStructure.CAN_AWUM = DISABLE; /* 自动唤醒禁止，有报文来的时候自动退出休眠	*/
  CAN_InitStructure.CAN_NART = DISABLE; /* 报文重传, 如果错误一直传到成功止，否则只传一次 */
  CAN_InitStructure.CAN_RFLM = DISABLE; /* 接收FIFO锁定, 1--锁定后接收到新的报文不要，0--接收到新的报文则覆盖前一报文	*/
  CAN_InitStructure.CAN_TXFP = ENABLE;  /* 发送优先级  0---由标识符决定  1---由发送请求顺序决定	*/
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /* 模式	*/
 
  /* 波特率计算方法 */ 	
	//CANbps= Fpclk/((BRP+1)*((CAN_BS1)+(CAN_BS2)+1)
	//Fpclk = 36M  
	//BRP = CAN_Prescaler - 1
	//sample = ( 1 + CAN_BS1) /(1 + CAN_BS1 + CAN_BS2)
	
	switch(baud)
	{
		case 1000:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 9;         /* 波特率预分频数 */  
		break;
		
	case 500:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 12;         /* 波特率预分频数 */  
		break;
	
		case 250:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 18;         /* 波特率预分频数 */  
		break;
		
		case 125:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 36;         /* 波特率预分频数 */  
		break;
		
	case 100:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 45;         /* 波特率预分频数 */  
		break;
	
	case 50:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 90;         /* 波特率预分频数 */  
		break;
	
	}
	 
	CAN_Init(CAN1, &CAN_InitStructure);
	
	NVIC_Config();

}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN2_Config(uint32_t baud)
{  
  /* CAN register init */
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE; /* 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳 */
  CAN_InitStructure.CAN_ABOM = DISABLE; /* 自动离线禁止，自动离线：一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出 */
  CAN_InitStructure.CAN_AWUM = DISABLE; /* 自动唤醒禁止，有报文来的时候自动退出休眠	*/
  CAN_InitStructure.CAN_NART = DISABLE; /* 报文重传, 如果错误一直传到成功止，否则只传一次 */
  CAN_InitStructure.CAN_RFLM = DISABLE; /* 接收FIFO锁定, 1--锁定后接收到新的报文不要，0--接收到新的报文则覆盖前一报文	*/
  CAN_InitStructure.CAN_TXFP = ENABLE;  /* 发送优先级  0---由标识符决定  1---由发送请求顺序决定	*/
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /* 模式	*/
 
  /* 波特率计算方法 */ 	
	//CANbps= Fpclk/((BRP+1)*((CAN_BS1)+(CAN_BS2)+1)
	//Fpclk = 36M  
	//BRP = CAN_Prescaler - 1
	//sample = ( 1 + CAN_BS1) /(1 + CAN_BS1 + CAN_BS2)
	
	switch(baud)
	{
		case 1000:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 9;         /* 波特率预分频数 */  
		break;
		
	case 500:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 12;         /* 波特率预分频数 */  
		break;
	
		case 250:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 18;         /* 波特率预分频数 */  
		break;
		
		case 125:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 36;         /* 波特率预分频数 */  
		break;
		
	case 100:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 45;         /* 波特率预分频数 */  
		break;
	
	case 50:
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
			CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;      /* 时间段1 */
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;      /* 时间段2 */
			CAN_InitStructure.CAN_Prescaler = 90;         /* 波特率预分频数 */  
		break;
	
	}
	 
	
	CAN_Init(CAN2, &CAN_InitStructure);
	
	NVIC_Config();

}



void CAN1_Filter_Config(uint32_t can_id,uint8_t CAN_FilterNumber,uint8_t format, uint8_t Filter_Mode )
{
	uint32_t   CAN_msgId     = 0;
	
	if(CAN_FilterNumber>=14)
	{
		return;
	}
	
	if (format == STANDARD_FORMAT)  
		{         /*   Standard ID                 */			
				CAN_msgId = (can_id<<21)|CAN_ID_STD;
		}
		else
		{
			CAN_msgId = (can_id<<3)|CAN_ID_EXT;
		}
	
  CAN_FilterInitStructure.CAN_FilterNumber = CAN_FilterNumber;
  CAN_FilterInitStructure.CAN_FilterMode = Filter_Mode; //CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_msgId>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow = CAN_msgId;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_msgId>>16;;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow =  CAN_msgId; 
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

}


void CAN2_Filter_Config(uint32_t can_id,uint8_t CAN_FilterNumber,uint8_t format, uint8_t Filter_Mode )
{
	uint32_t   CAN_msgId     = 0;
	
	if((CAN_FilterNumber < 14)||(CAN_FilterNumber > 27))
	{
		return;
	}
	
	if (format == STANDARD_FORMAT)  
		{         /*   Standard ID                 */			
				CAN_msgId = (can_id<<21)|CAN_ID_STD;
		}
		else
		{
			CAN_msgId = (can_id<<3)|CAN_ID_EXT;
		}
	
  CAN_FilterInitStructure.CAN_FilterNumber = CAN_FilterNumber;
  CAN_FilterInitStructure.CAN_FilterMode = Filter_Mode; //CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_msgId>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow = CAN_msgId;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_msgId>>16;;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow =  CAN_msgId; 
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

}



/**
  * @brief  Configures the NVIC for CAN.
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	  
#ifndef STM32F10X_CL
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	//  /* IT Configuration for CAN1 */  
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	
#else
#ifdef  __CAN1_USED__ 
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	//  /* IT Configuration for CAN1 */  
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
#endif
	
	

 

#ifdef __CAN2_USED__
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 /* IT Configuration for CAN2 */  
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
#endif /*__CAN1_USED__*/
	
#endif /* STM32F10X_CL*/  



//错误中断
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	//  /* IT Configuration for CAN1 */  
  CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);

}


void CAN1_wrMsg (CanTxMsg msg) 
{

	CAN_Transmit(CAN1, &msg);
}



void CAN2_wrMsg (CanTxMsg msg) 
{	
	CAN_Transmit(CAN2, &msg);
}



#ifndef STM32F10X_CL
void USB_LP_CAN1_RX0_IRQHandler(void)
#else
void CAN1_RX0_IRQHandler(void)
#endif
{
 
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);  /* 清除挂起中断 */
  CAN_Receive(CAN1,CAN_FIFO0,&CAN1_RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */

}




void CAN2_RX0_IRQHandler(void)
{
 
	CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);  /* 清除挂起中断 */
  CAN_Receive(CAN2,CAN_FIFO0, &CAN2_RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */

}


void CAN1_SCE_IRQHandler(void)
{
	CAN1_STATUS |= CAN_IT_BOF;
//	CAN_GetITStatus(CAN1,CAN_IT_ERR)
//	CAN_DeInit(CAN1);
}


void CAN2_SCE_IRQHandler(void)
{
//	CAN_GetITStatus(CAN1,CAN_IT_ERR)
	CAN2_STATUS |= CAN_IT_BOF;
//	CAN_DeInit(CAN1);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */
