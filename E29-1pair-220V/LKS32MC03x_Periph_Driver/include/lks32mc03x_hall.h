/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_hall.h
 * 文件标识：
 * 内容摘要： HALL驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   HMG
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/
#ifndef __lks32mc03x_HALL_H
#define __lks32mc03x_HALL_H

/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"

typedef struct
{
   __IO uint32_t CFG;
   __IO uint32_t INFO;
   __IO uint32_t WIDTH;
   __IO uint32_t TH;
   __IO uint32_t CNT;

} HALL_TypeDef;

typedef struct
{
   uint16_t FilterLen;       /*滤波长度,0对应长度1,1023对应长度1024滤波长度*/
   uint8_t ClockDivision;    /*分频 0~3:/1 /2 /4 /8*/
   uint8_t Filter75_Ena;     /*使能第一级7/5滤波,高电平有效*/
   uint8_t HALL_Ena;         /*使能HALL,高电平有效*/
   uint8_t HALL_CHGDMA_Ena;  /*HALL信号变化DMA请求使能,高电平有效*/
   uint8_t HALL_OVDMA_Ena;   /*HALL计数器溢出DMA请求使能,高电平有效*/
   uint8_t Capture_IRQ_Ena;  /*HALL信号变化中断使能,高电平有效*/
   uint8_t OverFlow_IRQ_Ena; /*HALL计数器溢出中断使能,高电平有效*/
   uint32_t CountTH;         /*HALL计数器门限值*/
   uint8_t softIE;           /* 软件中断使能 */
} HALL_InitTypeDef;

#define HALL_CLK_DIV1 ((uint32_t)0x00)
#define HALL_CLK_DIV2 ((uint32_t)0x01)
#define HALL_CLK_DIV4 ((uint32_t)0x02)
#define HALL_CLK_DIV8 ((uint32_t)0x03)

#define HALL_CAPTURE_EVENT ((uint32_t)0x00010000)
#define HALL_OVERFLOW_EVENT ((uint32_t)0x00020000)

void HALL_Init(HALL_InitTypeDef *HALL_InitStruct);
void HALL_StructInit(HALL_InitTypeDef *HALL_InitStruct);

uint32_t HALL_GetFilterValue(void);
uint32_t HALL_GetCaptureValue(void);
uint32_t HALL_GetCount(void);

uint32_t HALL_IsCaptureEvent(void);
uint32_t HALL_IsOverFlowEvent(void);

void HALL_Clear_IRQ(void);

#endif /*__lks32mc03x_HALL_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
