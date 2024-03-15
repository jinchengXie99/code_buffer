/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： MC_math.h
 * 文件标识：
 * 内容摘要： 电机控制相关数学计算函数
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
 /*------------------------------prevent recursive inclusion -------------------*/
#ifndef __MC_MATCH_H
#define __MC_MATCH_H

#include "Global_Variable.h"

#define DivSQRT_3               (s16)0x49E6     /* 1/sqrt(3) in q1.15 format=0.5773315*/
#define SIN_MASK                0x0300
#define U0_90                   0x0200
#define U90_180                 0x0300
#define U180_270                0x0000
#define U270_360                0x0100

UINT16 CurrentMagCalc(INT16 cx,INT16 cy);
s16 wGet_Atan(s32 x, s32 y);

#endif
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
