/*----------------------------------------------------------------------------
 * Name:    CAN.h
 * Purpose: low level CAN definitions
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

#ifndef __CAN_H
#define __CAN_H

#include "stdint.h"

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1

#define MAX_LENGTH 1000

typedef struct  {
  unsigned int   id;                 // 29 bit identifier
  unsigned char  data[8];            // Data field
  unsigned char  len;                // Length of data field in bytes
  unsigned char  format;             // 0 - STANDARD, 1- EXTENDED IDENTIFIER
  unsigned char  type;               // 0 - DATA FRAME, 1 - REMOTE FRAME
} CAN_msg;


void CAN1_wrMsg         (CanTxMsg msg);
void CAN1_rdMsg         (CanRxMsg msg);
void CAN2_wrMsg         (CanTxMsg msg);
void CAN2_rdMsg         (CanRxMsg msg);

//extern CAN_msg       CAN1_RxMsg[MAX_LENGTH];      // CAN message for receiving                 
//extern CAN_msg       CAN2_RxMsg[MAX_LENGTH];      // CAN message for receiving    

//extern CanRxMsg CAN1_RxMessage;
//extern CanRxMsg CAN2_RxMessage;

extern CanTxMsg CAN1_TxMessage;
extern CanTxMsg CAN2_TxMessage;
extern CanRxMsg CAN1_RxMessage;
extern CanRxMsg CAN2_RxMessage;

extern uint32_t CAN1_STATUS;
extern uint32_t CAN2_STATUS;


void CAN1_Config(uint32_t baud);
void CAN_GPIO_Config(void);
void CAN2_Config(uint32_t baud);
void CAN1_Filter_Config(uint32_t can_id,uint8_t CAN_FilterNumber,uint8_t format, uint8_t Filter_Mode );
void CAN2_Filter_Config(uint32_t can_id,uint8_t CAN_FilterNumber,uint8_t format, uint8_t Filter_Mode );
#endif // __CAN_H


