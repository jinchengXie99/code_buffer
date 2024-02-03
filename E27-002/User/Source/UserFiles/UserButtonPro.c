/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
* File Name          : Main.c
* Author             : Fortiortech Appliction Team
* Version            : V1.0
* Date               : 01/07/2015
* Description        : This file contains main function used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/ 


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------------------------*/

/* Private functions ----------------------------------------------------------------------------*/

#define START_PWR_CMD_LV  1                // 启动挡主电机挡位设置
//#define START_WIRE_CMD_LV  0               // 启动挡辅件挡位设置

#define KEY_GO            GP00
//#define KEY_HL            GP11
//#define LED_OUT           GP07
//#define DCM_IN            GP02

//#define HI_POWER_RATIO   (1.0)      // 380W
//#define MI_POWER_RATIO   (0.358)    // 120W
//#define LO_POWER_RATIO   (0.238)    // 70W

#define LV1_POWER         (float)(190.0)
#define LV2_POWER         (float)(198.0)
#define LV3_POWER         (float)(199.0)
#define LV4_POWER         (float)(199.0)

#define POWER_RATIO(P)    (float)(((P)-MIN_POWER)*(VSP_MAX_Q12-VSP_MIN_Q12)/(MAX_POWER-MIN_POWER)+VSP_MIN_Q12)/4096
#define LV1_POWER_RATIO   POWER_RATIO(LV1_POWER)
#define LV2_POWER_RATIO   POWER_RATIO(LV2_POWER)
#define LV3_POWER_RATIO   POWER_RATIO(LV3_POWER)
#define LV4_POWER_RATIO   POWER_RATIO(LV4_POWER)


//struct KeyIn KeyGo, KeyHL,KeyGo2;

//u8  KeyGoCmdOn;
//s8  KeyGoCmdLv;

//u8  KeyWireCmdOn;
//u8  KeyWireCmdLv;

//u8  KeyGoActCntX10ms;
//u8  StartOnTriger;

//u16 BtnSmp;
//u16 BtnZeroKeepCnt;
//u16 BtnLowKeepCnt;
//u16 BtnHighKeepCnt;
//u8  BtnLv;

//u16 Btn2Smp;
//u16 Btn2ZeroKeepCnt;
//u16 Btn2LowKeepCnt;
//u16 Btn2HighKeepCnt;
//u8  Btn2Lv;

//u16 DCMIBus;
//u16 DCMStage;
//u16 DCMDuty;
//u8  DCMOCCnt;
//u16 DCMDutyTmp;

//============================================================================//

// 说明：开关按钮控制

extern u8   FltReStartTrig;

//u8  StartPwrCmdLv;
//u8  StartPwrCmdLvAlt;

//u8  KeyGoLvStayCnt;
//u8  KeyGoLvNew;
//u8  KeyGoLvOld;
//u8  KeyGoLv;
void ReadKeys(void)
{
//  KeyGoLvNew = KEY_GO;
//  if(KeyGoLvNew == KeyGoLvOld)
//  {
//    if(++KeyGoLvStayCnt >= 5)
//    {
//      KeyGoLvStayCnt = 5;
//      KeyGoLv = KeyGoLvNew;
//    }
//  }
//  else
//  {
//    KeyGoLvStayCnt = 0;
//    KeyGoLvOld = KeyGoLvNew;
//  }
}
//----------------------------------------------------------------------------//
void CalKeysDuty(void)
{

//    KeyGo.Rise = 0;              // 清除上跳变沿
//    KeyGo.Fall = 0;              // 清除下跳变沿
//    KeyGo.PreLv = KeyGo.Lv;      // 保存上次值

//    KeyGo.Lv = KeyGoLv;//KEYGO_IN;

//    if(KeyGo.Lv != KeyGo.PreLv)  // 如本次不同于上次，判断是何种跳变
//    {
//      if(KeyGo.Lv == 1)          // 本次为高
//        KeyGo.Rise = 1;          // 上升沿
//      else                       // 否则为下降沿
//      KeyGo.Fall = 1;
//    }

////    if(KeyGo.Lv == 0)
////    {
////      if(++KeyGoActCntX10ms > 100)
////        KeyGoActCntX10ms = 100;
////    }
////    else
////      KeyGoActCntX10ms = 0;
//    
//    // 根据按键情况确定开关机 KeyGoCmdOn 和运行挡位 KeyGoCmdLv
//    
//    if(KeyGo.Fall == 1)    //if(KeyGo.Rise == 1)    //
//    {
//      if(KeyGoCmdOn == 0)
//      {
//        KeyGoCmdOn = 1;
//        if(KeyGoCmdLv == 0)
//          KeyGoCmdLv = 1;
//      }
//      else
//      {
//        if(++KeyGoCmdLv >= 2)
//        {
//          KeyGoCmdLv = 0;
//          KeyGoCmdOn = 0;
//          FltReStartTrig = 0;
//        }
//      }
//      
//      //StartPwrCmdLvAlt = KeyGoCmdLv;
//    }
//    
//    if(FltReStartTrig == 1)
//    {
//      if(KeyGoCmdOn == 0)
//        KeyGoCmdOn = 1;
//      FltReStartTrig = 0;
//    }
//    
//    if(KeyGoCmdOn == 0)
//    {
//      KeyGoDuty = _Q12(0);
//    }
//    else// if(KeyGoCmdOn == 1)
//    {
//      // 根据挡位确定输出量
//      if(KeyGoCmdLv == 0)
//      {
//        KeyGoDuty = _Q12(0);
//        //LED_OUT = 0;
//      }
//      else if(KeyGoCmdLv == 1)  // A挡功率
//      {
//        KeyGoDuty = _Q12(LV1_POWER_RATIO);
//        //LED_OUT = 0;
//      }
//      else if(KeyGoCmdLv == 2)  // B挡功率
//      {
//        KeyGoDuty = _Q12(LV2_POWER_RATIO);
//        //LED_OUT = 0;
//      }
//      else if(KeyGoCmdLv == 3)  // C挡功率
//      {
//        KeyGoDuty = _Q12(LV3_POWER_RATIO);
//        //LED_OUT = 0;
//      }
//    }
}
//----------------------------------------------------------------------------//
// 说明：开机挡位设置
void KeysLvInit(void)
{
//  StartPwrCmdLv = START_PWR_CMD_LV;
//  if(StartPwrCmdLv > 0)
//  {
//    KeyGoCmdOn = 1;
//    KeyGoCmdLv = StartPwrCmdLv;
//  }
}
