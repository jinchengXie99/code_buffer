///**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
//* File Name          : Main.c
//* Author             : Fortiortech Appliction Team
//* Version            : V1.0
//* Date               : 01/07/2015
//* Description        : This file contains main function used for Motor Control.
//***************************************************************************************************
//* All Rights Reserved
//**************************************************************************************************/ 


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

#include <UserGlobal.h>
#include <UserDefine.h>

/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------*/
void CalMaxRPM(void);
/* Private functions ----------------------------------------------------------------------------*/

#define FG_OUT          GP01 // OLD     //GP07 // NEW
#define LED_OUT         GP02 

//void  *ClearStruct  (void *s, size_t n)
//{
//  size_t cnt;
//  u8  *RamAddress;
//  
//  RamAddress = s;
//  for(cnt = 0;cnt<n;cnt++)
//  {
//    *(RamAddress+cnt) = 0;
//  }
//}

#if (FG_ENABLE == 1)
//----------------------------------------------------------------------------//
// 说明：FG信号，此时非运行状态、FOC中断关闭，在main中处理FG信号
void FGInMain(void)
{
  if((mcState != mcRun)&&(mcState != mcTailWind))
  {
    if(AlarmFlag == 0)
      FG_OUT = 0;
    else
      FG_OUT = 1;
  }
}

//----------------------------------------------------------------------------//
// 说明：FG信号，此时运行状态、FOC中断，在FOC中断中处理FG信号
u8 RotorIndexLv;
void FGInISR(void)
{
  if((mcState == mcRun)||(mcState == mcTailWind))
  {
    // 二选一
    // ▼--------------------------------------▼ //
    // 1. 电角度360对应一个方波周期
//    if(FOC_THETA < 32767)
//    {
//      FG_OUT = 0;
//    }
//    else
//    {
//      FG_OUT = 1;
//    }
    // ◆--------------------------------------◆ //
    // 2. 电角度360对应2个方波周期
    if(RotorIndexLv == 0)
    {
      if(FOC__THETA < 16384)
      {
        RotorIndexLv = 1;
        
        FG_OUT = ~FG_OUT;
      }
    }
    else if(RotorIndexLv == 1)
    {
      if((FOC__THETA > 32768)&&(FOC__THETA < 49152))
        RotorIndexLv = 0;
    }
    // ▲--------------------------------------▲ //
  }
}
#endif

//----------------------------------------------------------------------------//
// 说明：故障状态脉宽输出
//void AlarmPulseOut(void)
//{
//  // 状态脉宽输出 0% ?? 20%欠压 40%过电流  60%卡住 60%过温 80%超速 100%正常，如同时发生，以下优先级进行
//  if(AlarmFlag == 0)
//    TIM5_DR = 24000*0;
//  else if((AlarmFlag & 0x01) == 0x01)  // 硬件过流
//    TIM5_DR = 24000*0.4;
//  else if((AlarmFlag & 0x02) == 0x02)  // 软件过流
//    TIM5_DR = 24000*0.4;
//  else if((AlarmFlag & 0x04) == 0x04)  // 过压
//    TIM5_DR = 24000*0.2;
//  else if((AlarmFlag & 0x08) == 0x08)  // 欠压
//    TIM5_DR = 24000*0.2;
//  else if((AlarmFlag & 0x10) == 0x10)  // 高温
//    TIM5_DR = 24000*0.6;
//  else if((AlarmFlag & 0x20) == 0x20)  // 超速
//    TIM5_DR = 24000*0.9;
//  else if((AlarmFlag & 0x40) == 0x40)  // 卡住
//    TIM5_DR = 24000*0.8;
//}

extern u8  LowVoltLv;
idata u16 TermTmr;
bit LightTurn;

//u8  LEDTmrX1ms;
//u8  LEDTmrX10ms;
//u8  LEDDuty;
//u8  LEDTo10msTmr;
//u8  LEDMiniPrdTmr;
//u8  LEDStage;
//u8  LEDFltCode;
//u8  ChargeVoltLevel;
//u8  RunVoltLevel;
//u8  LEDSparkTerm;

u8  SparkErrCode;
u8  SparkErrStep;

//----------------------------------------------------------------------------//
// 说明：故障显示1，根据故障情况确定闪烁次数
//void LEDFaultSpark(void)
//{
//  //----------------------------------//
////  if(++TermTmr >= 200)
////  {
////    TermTmr = 0;
////    LightTurn = ~LightTurn;
////  }
////  
////  LED1 = LightTurn;
////  LED2 = LightTurn;
////  LED3 = LightTurn;
//  
//  //----------------------------------//
//  if(SparkErrStep < SparkErrCode)
//  {
//    if(TermTmr <= 250)
//      LightTurn = 1;
//    else
//      LightTurn = 0;
//  }
//  else
//    LightTurn = 1;
//  
//  if(++TermTmr >= 300)
//  {
//    TermTmr = 0;
//    
//    if(++SparkErrStep > SparkErrCode)
//      SparkErrStep = 0;

//    if(AlarmFlag > 0)
//    {
//      if(AlarmHardBeark == 1)
//        SparkErrCode = 2;
//      else if(AlarmOverCurrent == 1)
//        SparkErrCode = 3;
//      else if(AlarmHighVolt == 1)
//        SparkErrCode = 4;
//      else if(AlarmLowVolt == 1)
//        SparkErrCode = 5;
//      else if(AlarmHighTemper == 1)
//        SparkErrCode = 6;
//      else if(AlarmHighSpeed == 1)
//        SparkErrCode = 7;
//      else if(AlarmStuck == 1)
//        SparkErrCode = 8;
//      else if(AlarmPhaseLoss == 1)
//        SparkErrCode = 9;
//    }
//    else if(LowVoltLv == 0)
//      SparkErrCode = 5;
//    else
//      SparkErrCode = 0;
//  }
//  
//  LED = LightTurn;
//}

//----------------------------------------------------------------------------//
// 说明：故障显示2，分长亮短亮，以区分更多故障
u8  SparkErrCodeH;
u8  SparkErrCodeL;
u16 SparkClock;
u16 SparkBlinkPrd;
u16 SparkOnPrd;
u8  SparkStepSum;
void LEDFaultSpark(void)
{
//  if(SparkClock == 0)
//  {
//    if(AlarmFlag > 0)
//    {
//      if((AlarmFlag & 0x0F) > 0)
//      {
//        SparkErrCodeH = 1;
//        
//        if(AlarmHardBeark == 1)
//          SparkErrCodeL = 1;
//        else if(AlarmOverCurrent == 1)
//          SparkErrCodeL = 2;
//        else if(AlarmHighVolt == 1)
//          SparkErrCodeL = 3;
//        else if(AlarmLowVolt == 1)
//          SparkErrCodeL = 4;
//      }
//      else
//      {
//        SparkErrCodeH = 2;
//        
//        if(AlarmHighTemper == 1)
//          SparkErrCodeL = 1;
//        else if(AlarmHighSpeed == 1)
//          SparkErrCodeL = 2;
//        else if(AlarmStuck == 1)
//          SparkErrCodeL = 3;
//        else if(AlarmPhaseLoss == 1)
//          SparkErrCodeL = 4;
//      }
//    }
//    else if(LowVoltLv == 0)
//    {
//      SparkErrCodeH = 1;
//      SparkErrCodeL = 4;
//    }
//    else
//    {
//      SparkErrCodeH = 1;
//      SparkErrCodeL = 0;
//    }
//  }
//  
//  SparkStepSum = SparkErrCodeH + 1 + SparkErrCodeL;
//  if(SparkErrStep < SparkErrCodeH)
//  {
//    SparkBlinkPrd = 500;
//    SparkOnPrd = 200;
//  }
//  else if(SparkErrStep < (SparkErrCodeH+1))
//  {
//    SparkBlinkPrd = 300;
//    SparkOnPrd = 0;
//  }
//  else if(SparkErrStep < SparkStepSum)
//  {
//    SparkBlinkPrd = 300;
//    SparkOnPrd = 50;
//  }
//  else //if(SparkErrStep = SparkShortSite)
//  {
//    SparkBlinkPrd = 200;
//    SparkOnPrd = 0;
//  }

//  SparkClock++;
//  
//  if(SparkClock < SparkOnPrd)
//    LightTurn = 1;
//  else if(SparkClock < SparkBlinkPrd)
//    LightTurn = 0;
//  else
//  {
//    SparkClock = 0;

//    if(++SparkErrStep > SparkStepSum)
//    {
//      SparkErrStep = 0;
//    }
//  }

//  LED_OUT = ~LightTurn;
}

//void LEDShow(void)
//{
//  if(LEDTmrX1ms <= LEDDuty)
//    LED = 1;
//  else
//    LED = 0;
//;
//  if(++LEDTmrX1ms >= 20)
//  {
//    LEDTmrX1ms = 0;
//  
//    if(++LEDTmrX10ms == 2)
//    {
//      LEDTmrX10ms = 0;
//      
//      
//      if(++LEDDuty >= 20)
//        LEDDuty = 0;
//    }
//  }
//}

