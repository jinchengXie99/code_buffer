/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： hardware_init.h
 * 文件标识：
 * 内容摘要： 硬件初始化头文件定义
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet Li
 * 完成日期： 2020年8月16日
 *
 * 修改记录1：
 * 修改日期：2020年8月16日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet Li
 * 修改内容：创建
 *
 *******************************************************************************/
#ifndef __HARDWARE_INIT_H
#define __HARDWARE_INIT_H

#include "basic.h"

void PGA_Init(void);
void DAC_Init(void);
void CMP_init(void);
void MCPWM_init(void);
void UTimer_init(void);
void GPIO_init(void);
void UART1_init(void);
void ADC_init(void);
void get_ic_temp_init(void);
void Clock_Init(void);
void Hardware_init(void);

void Motor0_ADC_SampleSetting(void);
void Motor1_ADC_SampleSetting(void);
void AlternateOut_CH012_CH3(void);
void MCPWM1_CH3_RegUpdate(void);

void ADC_NormalModeCFG(void);

#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

