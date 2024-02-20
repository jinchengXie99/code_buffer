/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： MC_IPD.h
 * 文件标识：
 * 内容摘要： 电机初始位置定位函数声明
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_IPD_H
#define __MC_IPD_H

#include "mc_type.h"


typedef struct
{
    u32 wIPD_PlusWidthSet;                        /* IPD位置检测，注入脉冲宽度设置，单位：clock周期数 */
    u32 wIPD_IdleWaitSet;                         /* IPD位置检测，续流结束等待宽度设置，单位：clock周期数 */
    u16 nPWM_PERIOD;                              /* PWM 占空比 */

    u8  bIPD_State;                               /* IPD状态标志 */
    u8  bIPD_StepFlg;                             /* IPD步骤标志 */

    u16 IPD_Angle;                                /* IPD得到的角度值 */

    s16 AdcSampCurr[7];                           /* ADC 采样值 */
  
    volatile u16 hDriverPolarity;                 /* 取MCPWM_IO1、MCPWIO23配置里面CHxN、CHxP的极性的配置 */
  
    u8 bCurrentSampleType;                        /* 电流采样类型，单电阻，双电阻，三电阻 */

} stru_IPD_CtrProcDef;


void IPD_RotorPosEst(void);

#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

