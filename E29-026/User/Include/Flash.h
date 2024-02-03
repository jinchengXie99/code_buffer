
/************************ (C) COPYRIGHT 2015 FT *******************************
* File Name          : 
* Author             : Application Team  Tom.wang 
* Version            : V2.0.0
* Date               : 06/15/2015
* Description        : 
********************************************************************************
/*******************************************************************************
* All Rights Reserved
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

#define STARTPAGEROMADDRESS 0X3F00 

typedef struct
{
  uint8  WriteValue;       //ROM 写入值
  uint8  ReadValue;        //ROM 读出值
}ROM_TypeDef;

extern ROM_TypeDef xdata  Rom;  
  
extern void Write2Byte2Flash(uint8 xdata *BlockStartAddr,uint16 NewData2Flash);
extern void Write1Byte2Flash(uint8 xdata *BlockStartAddr,uint8 NewData2Flash);
extern uint8 Get1ByteFromFlash(uint8 xdata *BlockStartAddr);
  
#endif