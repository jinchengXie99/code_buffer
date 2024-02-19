/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： PID_regulators.h
 * 文件标识：
 * 内容摘要： PID控制头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet
 * 完成日期： 2020年8月5日
 *
 * 修改记录1：
 * 修改日期： 2020年8月5日
 * 版 本 号： V 1.0
 * 修 改 人： Howlet
 * 修改内容： 创建
 *
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    s16 hKp_Gain;
    u16 hKp_Divisor;
    s16 hKi_Gain;
    u16 hKi_Divisor;
    s32 hLower_Limit_Output;     //Lower Limit for Output limitation
    s32 hUpper_Limit_Output;     //Lower Limit for Output limitation
    s32 wLower_Limit_Integral;   //Lower Limit for Integral term limitation
    s32 wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
    s32 wIntegral;
} stru_PIparams;

typedef struct 
{
    s32 wUpperLimitOutput;       /* Lower Limit for Output limitation */
    s32 wLowerLimitOutput;       /* Lower Limit for Output limitation */
    u16 KP;                      /* 比例系数 */
    u16 KI;                      /* 积分系数 */
    s32 wIntegral;               /* 积分器 */
    s32 wInError;                /* 输入误差 */
    s32 wLastError;              /* 上次输入误差 */
    s32 wErrorC;
}stru_PIRegulator;

typedef struct
{
    s32 wUpperLimitOutput;       /*速度环输出最大值*/
    s32 wLowerLimitOutput;       /*速度环输出最小值*/
    u16 KP;                      /* 比例系数 */
    u16 KI;                      /* 积分系数 */
    s32 wInError;                /* 输入误差 */
    s64 wIntegral;               /* 积分器 */
    s64 lACC;                    /* 中间计算变量*/
}stru_SpeedPIRegulator;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
s16 PI_Regulator(s16, s16, stru_PIparams *);

s16 PD_Regulator(s16 hPresentFeedback, stru_PIparams *PD_Struct);

s16 HL_PI_AntiDump(s16 DesiredValue,s16 MeasuredValue,stru_PIparams *pParams);

s16 CurrentPIRegulator(stru_PIRegulator *Reg);

s16 SpeedPIRegulator(stru_SpeedPIRegulator *pReg);

/* Exported variables ------------------------------------------------------- */

#endif 

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

