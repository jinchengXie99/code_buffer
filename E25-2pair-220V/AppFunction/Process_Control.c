/*******************************************************************************
 * 版权所有 (C)2016, LINKO Semiconduct
 *
 * 文件名称： Process_Control.c
 * 文件标识：
 * 内容摘要： 电机控制流程函数集
 * 其它说明： 无
 * 当前版本： V 2.0
 * 作    者： WhenCai Zhao Howlet Li
 * 完成日期： 2020年9月10日
 *
 * 修改记录1：
 *    修改日期：2019年12月26日
 *    版 本 号：V 1.0
 *    修 改 人：WhenCai Zhao
 *    修改内容：创建
 *
 * 修改记录2：
 *    修改日期：2020年9月10日
 *    版 本 号：V2.0
 *    修 改 人：Howlet Li
 *    修改内容：格式整理
 *
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Global_Variable.h"


/*****************************************************************************
 * 函数名   : s16 OpenLoopCurRamp(stru_OpenForceRunDef *this)
 * 说明     : 开环电流爬坡
 * 设计思路 ：1.电流爬坡
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
s16 OpenLoopCurRamp(stru_OpenForceRunDef *this)
{
  s32 ax;
  ax = this->nStartCurSet << 3;
  if(this->wStartCurRamp < ax)
  {
    this->wStartCurRamp += this->nCurrentAccStep;
    if(this->wStartCurRamp >= ax)
    {
      this->wStartCurRamp = ax;
    }
  }
  if(this->wStartCurRamp > ax)
  {
    this->wStartCurRamp -= this->nCurrentDecStep;
    if(this->wStartCurRamp < ax)
    {
      this->wStartCurRamp = ax;
    }
  }
  this->nStartCurRef = this->wStartCurRamp >> 3;

  if(this->wStartCurRamp == ax)
  {
    return (1);
  }
  else
  {
    return (0);
  }
}

/*****************************************************************************
 * 函数名   : void SpeedReferenceGen(stru_OpenForceRunDef *this)
 * 说明     : 速度爬坡
 * 设计思路 ：1.速度爬坡
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void SpeedReferenceGen(stru_OpenForceRunDef *this)
{
  s32 ax;
  ax = this->wOpen2CloseFreq << 7;
  if(this->wRampFreqRef < (ax))
  {
    this->wRampFreqRef += this->nFreqAccStep;
    if(this->wRampFreqRef >= ax)
    {
      this->wRampFreqRef = ax;
    }
  }
  if(this->wRampFreqRef > ax)
  {
    this->wRampFreqRef -= this->nFreqDecStep;
    if(this->wRampFreqRef <= ax)
    {
      this->wRampFreqRef = ax;
    }
  }
}

/*******************************************************************************
 函数名称：    s16 GetCurrentTemperature(s16 ADC_value)
 功能描述：    得到当前温度值
 输入参数：    ADC_value: ADC通道14为温度传感器，参数输入需要以右对齐格式，
               当ADC设置为左对齐时，需要右移4位
 输出参数：    无
 返 回 值：    t_Temperture：当前温度值，单位：1个Lsb代表0.1度
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
s16 GetCurrentTemperature(s16 ADC_value)
{
  s16 t_Temperture;
//    t_Temperture = (m_TempertureCof.nOffsetB - ((s32)m_TempertureCof.nCofA * ADC_value) / 1000);

  return t_Temperture;
}

/*******************************************************************************
 函数名称：    void ReadLKS05X_UniqueID(void)
 功能描述：    得到芯片唯一ID值
 输入参数：    无
 输出参数：    无
 返 回 值：    LKS_ID1,LKS_ID2,LKS_ID3,LKS_ID4合成一个128bit唯一芯片ID
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void ReadLKS05X_UniqueID(void)
{
  volatile uint32_t LKS_ID1;
  volatile uint32_t LKS_ID2;
  volatile uint32_t LKS_ID3;
  volatile uint32_t LKS_ID4;
//    LKS_ID1 = Read_NVR(0x0000084C);
//    LKS_ID2 = Read_NVR(0x00000850);
//    LKS_ID3 = Read_NVR(0x00000854);
//    LKS_ID3 = Read_NVR(0x00000858);
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */



