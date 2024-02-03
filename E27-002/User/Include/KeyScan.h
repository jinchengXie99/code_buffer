/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
* File Name          : KeyScan.h
* Author             : Billy Long Fortiortech  Market Dept
* Version            : V1.0
* Date               : 01/07/2015
* Description        : This file contains all the common data types used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/ 

/* Define to prevent recursive inclusion --------------------------------------------------------*/
#ifndef __KEYSCAN_H_
#define __KEYSCAN_H_

#define D1              GP36
#define D2              GP37
#define D3              GP00
#define D4              GP01
#define D5              GP02
#define D6              GP03

#define SW1              GP15 //温度  
#define SW2              GP10 //转速
#define SW3              GP13 //冷风按键

#define LOCK            GP05  //档位锁定键
#define OPENKEY         GP06  //开机键

#define flz             GP22  //负离子
#define HW1             GP10
#define HW2             GP11 

#define ON                 0
#define OFF                1
#define LEDON              0
#define LEDOFF             1
#define HWON               0
#define HWOFF              1

/* Exported types -------------------------------------------------------------------------------*/
typedef struct
{
  uint8 SW1_Flag;
  uint8 SW2_Flag;
  uint8 SW3_Flag;
  uint8 SW4_Flag;
  uint8 SW5_Flag;
  
  uint8 Key1Value;
  uint8 Key2Value;
  uint8 Key3Value;
  
  uint8 Key1Valueflag;
  uint8 Key2Valueflag;
  
  int16 Key1PressCnt;
  int16 Key2PressCnt;
  int16 Key3PressCnt;
  int16 Key4PressCnt;
  int16 Key5PressCnt;
  
  int8  KeyValuetotal;
  int8  OldKeyValuetotal;
  uint8 ChangeKeyFlg;
  uint8 FlashWriteOnetimes;
  uint8  KeyLongPressFlag;  
  
} KeyScanParam_TypeDef;


/* Exported variables ---------------------------------------------------------------------------*/
extern KeyScanParam_TypeDef xdata KS;

/* Exported functions ---------------------------------------------------------------------------*/
extern void KeyInit(void);
extern int  KeyValue(void);
extern void KeyScan(void);

#endif

