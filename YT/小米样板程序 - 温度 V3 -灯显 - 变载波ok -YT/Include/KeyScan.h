/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： KeyScan.h
 * 文件标识：
 * 内容摘要： KeyScan  & Led
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2019年12月19日
 *
 *******************************************************************************/
#ifndef __KEYSCAN_H
#define __KEYSCAN_H

#include "basic.h"


#define   KEY0        (GPIO2_PDI & 0x0020) //P2.5

#define   LED_ON      (GPIO0_PDO &= 0x7FFF) //LED3-P0.15
#define   LED_OFF     (GPIO0_PDO |= 0x8000)

#define   KEY_FLITER  10//ms

#define   KEY_START       0 
#define   KEY STOP        1

typedef struct
{
//key0
  s16 nKey0Value;
  s16 nKey0PressCnt;
  s16 nFlagKey0Press;  
} KeyControl_t;


extern KeyControl_t key_state;

extern void KeyScan(KeyControl_t *pKeyState);
extern void KeyInit(void);
#endif

