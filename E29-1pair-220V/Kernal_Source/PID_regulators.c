/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： PID_regulators.c
 * 文件标识：
 * 内容摘要： PID处理函数
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

/* Standard include ----------------------------------------------------------*/
#include "basic.h"
#include "MC_type.h"
#include "PID_regulators.h"


 /*******************************************************************************
 函数名称：    s16 PI_Regulator(s16 hReference, s16 hPresentFeedback, stru_PIparams *PID_Struct)
 功能描述：    PI调节器 绝对值式
 输入参数：    s16 DesiredValue: 期望值 
               s16 MeasuredValue:反馈值 
               stru_PIparams *PID_Struct 结构体指针
 输出参数：    PI调节结果
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
s16 PI_Regulator(s16 DesiredValue, s16 MeasuredValue, stru_PIparams *PID_Struct)
{
    s32 wError, wProportional_Term, wIntegral_Term;
    volatile s32 houtput_32;
    s64 dwAux;

    // error computation
    wError = (s32)(DesiredValue - MeasuredValue);

    // Proportional term computation
    wProportional_Term = PID_Struct->hKp_Gain * wError;

    // Integral term computation
    if (PID_Struct->hKi_Gain == 0)
    {
        PID_Struct->wIntegral = 0;
    }
    else
    {
        wIntegral_Term = PID_Struct->hKi_Gain * wError;
        dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);

        if (dwAux > PID_Struct->wUpper_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
        }
        else if (dwAux < PID_Struct->wLower_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
        }
        else
        {
            PID_Struct->wIntegral = (s32)(dwAux);
        }
    }

    houtput_32 = ((wProportional_Term >> PID_Struct->hKp_Divisor)
                  + (PID_Struct->wIntegral >> PID_Struct->hKi_Divisor));
    if (houtput_32 >= PID_Struct->hUpper_Limit_Output)
    {
        return (PID_Struct->hUpper_Limit_Output);
    }
    else if (houtput_32 < PID_Struct->hLower_Limit_Output)
    {
        return (PID_Struct->hLower_Limit_Output);
    }
    else
    {
        return ((s16)(houtput_32));
    }
}


 /*******************************************************************************
 函数名称：    s16 CurrentPIRegulator(stru_PIRegulator *Regulator)
 功能描述：    电流环PI  增量式PI
 输入参数：    stru_PIRegulator *Regulator 结构体指针
 输出参数：    PI调节结果
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           WenCai Zhao          创建
 *******************************************************************************/
s16 CurrentPIRegulator(stru_PIRegulator *Reg)
{
    long    ACC;
    int AX;

    ACC = (long)(Reg->wInError - Reg->wLastError) * Reg->KP;   /* 比例项计算 */
    ACC = (ACC << 4) + (long)(Reg->wInError) * Reg->KI;        /* 积分项计算 */
    Reg->wIntegral = ACC + Reg->wIntegral;

    if(Reg->wIntegral > Reg->wUpperLimitOutput)                /* 输出最大值限幅 */
    {
        Reg->wIntegral = Reg->wUpperLimitOutput;
    }
    else if(Reg->wIntegral < Reg->wLowerLimitOutput)         /* 输出最小值限幅 */
    {
        Reg->wIntegral = Reg->wLowerLimitOutput;
    }

    AX = Reg->wIntegral >> 16;

    Reg->wLastError = Reg->wInError;                           /* 记录上次误差值 */

    return(AX);                                                          
}

 /*******************************************************************************
 函数名称：   s16 SpeedPIRegulator(stru_SpeedPIRegulator *Reg)
 功能描述：    速度环PI  绝对式PI
 输入参数：    stru_SpeedPIRegulator *Reg 结构体指针
 输出参数：    PI调节结果
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/10/7      V1.0           WenCai Zhao          创建
 *******************************************************************************/
s16 SpeedPIRegulator(stru_SpeedPIRegulator *pReg)
{
    s32 t_AX;
    s64 t_OUT_LIM_H, t_OUT_LIM_L;
    
    t_OUT_LIM_H = (s64)(pReg->wUpperLimitOutput) << 22;      /*输出限幅值*/
    t_OUT_LIM_L = (s64)(pReg->wLowerLimitOutput) << 22; 
   
    pReg->lACC = (s64)pReg->wInError * pReg->KI;             /*积分项计算*/
  
    pReg->wIntegral = pReg->wIntegral + pReg->lACC;
    
    if(pReg->wIntegral > t_OUT_LIM_H)
    {
       pReg->wIntegral = t_OUT_LIM_H;
    }
    else if(pReg->wIntegral < t_OUT_LIM_L)
    {
        pReg->wIntegral = t_OUT_LIM_L;
    }
    
    pReg->lACC = (s64)(pReg->wInError) * pReg->KP * 128;     /*比例项计算*/

    pReg->lACC = pReg->lACC + pReg->wIntegral;
    
    if(pReg->lACC > t_OUT_LIM_H)
    {
        pReg->lACC = t_OUT_LIM_H;
    }
    else if(pReg->lACC < t_OUT_LIM_L)
    {
        pReg->lACC = t_OUT_LIM_L;
    }
    
    t_AX = (pReg->lACC >> 22);
    
    return (t_AX);
}

/******************** (C) COPYRIGHT 2008 LINKO Semiconductor *******************/

