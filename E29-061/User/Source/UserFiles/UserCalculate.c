//  /**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
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

#define MAX_PWR_HI     MAX_POWER
#define MAX_PWR_LO     (MIN_POWER+((MAX_POWER+MIN_POWER)>>2))

/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------*/
s16 CalTemperOffset(u16 base,u16 val, u16 warm, u16 cool);

/* Private functions ----------------------------------------------------------------------------*/

extern idata u16 MaxSpdOfStop;
extern idata u32 MaxRPMOfStop;

extern idata u16 DrvTemperADCVal;
extern idata s16 DrvTemperCelsius;

extern idata u16 HeatTemperADCVal;
extern idata s16 HeatTemperCelsius;

extern u16 HighSpdInhibitX10ms;
extern u16 RunningX1ms;

extern u8  idata AlarmHighSpeed;           // 5 超速
extern u8  HiSpdLv;                  // 超速等级

extern u8  idata FlagOverCurrent;          // 软件过流
extern u8  idata AlarmOverCurrent;         // 1 软件过流
extern u8  OCLv;

s16 FocIXMax_Max;

//----------------------------------------------------------------------------//
// 说明：检测电流
//s16 FocIAAbs, FocIBAbs, FocICAbs;
s16 FocIA, FocIB, FocIC;
s16 FocIXMax;
s16 IXMaxOCCnt;
void CheckCurrent(void)
{
  FocIA = FOC__IA;
  FocIB = FOC__IB;
  FocIC = -FocIA - FocIB;

  if(FocIA >= 0)
    mcCurVarible.Abs_ia = FocIA;
  else
    mcCurVarible.Abs_ia = -FocIA;
  
  if(FocIB >= 0)
    mcCurVarible.Abs_ib = FocIB;
  else
    mcCurVarible.Abs_ib = -FocIB;
  
  if(FocIC >= 0)
    mcCurVarible.Abs_ic = FocIC;
  else
    mcCurVarible.Abs_ic = -FocIC;

  FocIXMax = mcCurVarible.Abs_ia;
  if(FocIXMax < mcCurVarible.Abs_ib)
    FocIXMax = mcCurVarible.Abs_ib;
  if(FocIXMax < mcCurVarible.Abs_ic)
    FocIXMax = mcCurVarible.Abs_ic;
  
  if(FocIXMax_Max < FocIXMax)
    FocIXMax_Max = FocIXMax;
  
  //if(MOE == 1)
  {
    //if(FocIXMax > I_VALUE(40))I_Value(I_BUS_SHUTDOWN)
    if(FocIXMax > I_Value(I_BUS_SHUTDOWN*1.2))
    {
      if(RunningX1ms >= 500)        // 注意计数器清零
      {
        if(++mcCurVarible.OverCurCnt >= 8)
        {
          FlagOverCurrent = 1;
          AlarmOverCurrent = 1;
          OCLv = 1;
          ShutPerform();
          SetFaultResatrt();
        }
      }
    }
    else
    {
      FlagOverCurrent = 0;
      if(mcCurVarible.OverCurCnt > 0)
      {
        mcCurVarible.OverCurCnt--;
      }
    }
  }
}

//----------------------------------------------------------------------------//
// 说明：温度计算公式
s16 CalTemperOffset(u16 base,u16 val, u16 warm, u16 cool)
{
  return(base - 25*(val-warm)/(cool-warm));
}
//----------------------------------------------------------------------------//
// 说明：驱动器温度检测
// 驱动器热敏电阻特定温度阻值
// ---------|-------|-------|-------|-------|-------|-------|
// ℃     -25       0      25      50      75     100     125
// R  662.729 157.265  47.500  17.266   7.258   3.427   1.778
#define CELSIUS_N25_ADC         DRV_CELSIUS_ADC(662.729)
#define CELSIUS_0_ADC           DRV_CELSIUS_ADC(157.265)
#define CELSIUS_P25_ADC         DRV_CELSIUS_ADC(47.500)
#define CELSIUS_P50_ADC         DRV_CELSIUS_ADC(17.266)
#define CELSIUS_P75_ADC         DRV_CELSIUS_ADC(7.258)
#define CELSIUS_P100_ADC        DRV_CELSIUS_ADC(3.427)
#define CELSIUS_P125_ADC        DRV_CELSIUS_ADC(1.778)
void CheckTemper(void)
{
  //DrvTemperADCVal = DrvTempADCSmpFilt;
  
  // >=25℃
  if(DrvTemperADCVal <= CELSIUS_P25_ADC)
  {
    // <50℃
    if(DrvTemperADCVal > CELSIUS_P50_ADC)
      DrvTemperCelsius = CalTemperOffset(50,DrvTemperADCVal,CELSIUS_P50_ADC,CELSIUS_P25_ADC);
    // <75℃
    else if(DrvTemperADCVal > CELSIUS_P75_ADC)
      DrvTemperCelsius = CalTemperOffset(75,DrvTemperADCVal,CELSIUS_P75_ADC,CELSIUS_P50_ADC);
    // <100℃
    else if(DrvTemperADCVal > CELSIUS_P100_ADC)
      DrvTemperCelsius = CalTemperOffset(100,DrvTemperADCVal,CELSIUS_P100_ADC,CELSIUS_P75_ADC);
//    else
//      DrvTemperCelsius = 100;
    // <125℃
    else if(DrvTemperADCVal > CELSIUS_P125_ADC)
      DrvTemperCelsius = CalTemperOffset(125,DrvTemperADCVal,CELSIUS_P125_ADC,CELSIUS_P100_ADC);
    // >125℃
    else
      DrvTemperCelsius = 125;
  }
  // <25℃
  else
  {
    DrvTemperCelsius = 25;
//    // >0℃
//    if(DrvTemperADCVal < CELSIUS_0_ADC)
//      DrvTemperCelsius = CalTemperOffset(25,DrvTemperADCVal,CELSIUS_P25_ADC,CELSIUS_0_ADC);
//    // >-25℃
//    else if(DrvTemperADCVal < CELSIUS_N25_ADC)
//      DrvTemperCelsius = CalTemperOffset(0,DrvTemperADCVal,CELSIUS_0_ADC,CELSIUS_N25_ADC);
//    else
//      DrvTemperCelsius = -25;
  }
}

//----------------------------------------------------------------------------//
// 说明：电热丝温度检测
// 电热丝热敏电阻特定温度阻值
// ---------|-------|-------|-------|-------|-------|-------|
// ℃     -25       0      25      50      75     100     125
// R 1287.405 325.711  100.00  35.882  14.665   6.6519   3.2750

// ---------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
// ℃      40      45      50      55      60      65      70      75      80      85
// R  53.1347 43.6115 36.0143 29.8255  24.816 20.7406 17.4087 14.6721 12.4143 10.5434

#define CELSIUS_N25_ADC_B         HEAT_CELSIUS_ADC(1287.405)
#define CELSIUS_0_ADC_B           HEAT_CELSIUS_ADC(325.711)
#define CELSIUS_P25_ADC_B         HEAT_CELSIUS_ADC(100.00)
#define CELSIUS_P50_ADC_B         HEAT_CELSIUS_ADC(35.882)
#define CELSIUS_P75_ADC_B         HEAT_CELSIUS_ADC(14.665)
#define CELSIUS_P100_ADC_B        HEAT_CELSIUS_ADC(6.6519)
#define CELSIUS_P125_ADC_B        HEAT_CELSIUS_ADC(3.2750)
void CheckHeatTemper(void)
{
  //HeatTemperADCVal HeatTemperCelsius
  // >=25℃
  if(HeatTemperADCVal <= CELSIUS_P25_ADC_B)
  {
    // <50℃
    if(HeatTemperADCVal > CELSIUS_P50_ADC_B)
      HeatTemperCelsius = CalTemperOffset(50,HeatTemperADCVal,CELSIUS_P50_ADC_B,CELSIUS_P25_ADC_B);
    // <75℃
    else if(HeatTemperADCVal > CELSIUS_P75_ADC_B)
      HeatTemperCelsius = CalTemperOffset(75,HeatTemperADCVal,CELSIUS_P75_ADC_B,CELSIUS_P50_ADC_B);
    // <100℃
    else if(HeatTemperADCVal > CELSIUS_P100_ADC_B)
      HeatTemperCelsius = CalTemperOffset(100,HeatTemperADCVal,CELSIUS_P100_ADC_B,CELSIUS_P75_ADC_B);
//    else
//      DrvTemperCelsius = 100;
    // <125℃
    else if(HeatTemperADCVal > CELSIUS_P125_ADC_B)
      HeatTemperCelsius = CalTemperOffset(125,HeatTemperADCVal,CELSIUS_P125_ADC_B,CELSIUS_P100_ADC_B);
    // >125℃
    else
      HeatTemperCelsius = 125;
  }
  // <25℃
  else
  {
    HeatTemperCelsius = 25;
//    // >0℃
//    if(HeatTemperADCVal < CELSIUS_0_ADC_B)
//      HeatTemperCelsius = CalTemperOffset(25,HeatTemperADCVal,CELSIUS_P25_ADC_B,CELSIUS_0_ADC_B);
//    // >-25℃
//    else if(HeatTemperADCVal < CELSIUS_N25_ADC_B)
//      HeatTemperCelsius = CalTemperOffset(0,HeatTemperADCVal,CELSIUS_0_ADC_B,CELSIUS_N25_ADC_B);
//    else
//      HeatTemperCelsius = -25;
  }
}
//----------------------------------------------------------------------------//
// 
void CalMaxRPM(void)
{
//  //if(KeyGoCmdLv == 3)
//  {
//    if(UBusX10 <= U_BUS_LOW_X10)
//      MaxSpdOfStop = U_LOW_MAX_SPD;
//    else if(UBusX10 <= U_BUS_TURN1_X10)
//    {
//      MaxSpdOfStop = U_LOW_MAX_SPD + 
//        (UBusX10 - U_BUS_LOW_X10)*((s32)U_TURN1_MAX_SPD-U_LOW_MAX_SPD)/(U_BUS_TURN1_X10 - U_BUS_LOW_X10);
//    }
//    else if(UBusX10 <= U_BUS_TURN2_X10)
//    {
//      MaxSpdOfStop = U_TURN1_MAX_SPD + 
//        (UBusX10 - U_BUS_TURN1_X10)*((s32)U_TURN2_MAX_SPD-U_TURN1_MAX_SPD)/(U_BUS_TURN2_X10 - U_BUS_TURN1_X10);
//    }
//    else if(UBusX10 <= U_BUS_HIGH_X10)
//    {
//      MaxSpdOfStop = U_TURN2_MAX_SPD + 
//        (UBusX10 - U_BUS_TURN2_X10)*((s32)U_HIGH_MAX_SPD-U_TURN2_MAX_SPD)/(U_BUS_HIGH_X10 - U_BUS_TURN2_X10);
//    }
//    else // if(UBusX10 > U_BUS_HIGH_X10)
//      MaxSpdOfStop = U_HIGH_MAX_SPD;
//    
//    //MaxRPMOfStop = (u32)MaxSpdOfStop<<2;
//    MaxRPMOfStop = ((uint32)MaxSpdOfStop*(MOTOR_SPEED_BASE>>2))>>13;
//  }
//  else if(KeyGoCmdLv == 2)
//  {
//    if(UBusX10 <= U_BUS_LOW_X10)
//      MaxSpdOfStop = U_LOW_MAX_SPD;
//    else if(UBusX10 <= U_BUS_TURN1_X10_A)
//    {
//      MaxSpdOfStop = U_LOW_MAX_SPD_A + 
//        (UBusX10 - U_BUS_LOW_X10)*((s32)U_TURN1_MAX_SPD_A-U_LOW_MAX_SPD_A)/(U_BUS_TURN1_X10_A - U_BUS_LOW_X10);
//    }
//    else if(UBusX10 <= U_BUS_TURN2_X10_A)
//    {
//      MaxSpdOfStop = U_TURN1_MAX_SPD_A + 
//        (UBusX10 - U_BUS_TURN1_X10_A)*((s32)U_TURN2_MAX_SPD_A-U_TURN1_MAX_SPD_A)/(U_BUS_TURN2_X10_A - U_BUS_TURN1_X10_A);
//    }
//    else if(UBusX10 <= U_BUS_HIGH_X10)
//    {
//      MaxSpdOfStop = U_TURN2_MAX_SPD_A + 
//        (UBusX10 - U_BUS_TURN2_X10_A)*((s32)U_HIGH_MAX_SPD_A-U_TURN2_MAX_SPD_A)/(U_BUS_HIGH_X10 - U_BUS_TURN2_X10_A);
//    }
//    else // if(UBusX10 > U_BUS_HIGH_X10)
//      MaxSpdOfStop = U_HIGH_MAX_SPD;
//    
//    MaxRPMOfStop = ((uint32)MaxSpdOfStop*(MOTOR_SPEED_BASE>>2))>>13;
//  }
//  else if(KeyGoCmdLv == 1)
//  {
//    if(UBusX10 <= U_BUS_LOW_X10)
//      MaxSpdOfStop = U_LOW_MAX_SPD;
//    else if(UBusX10 <= U_BUS_TURN1_X10_B)
//    {
//      MaxSpdOfStop = U_LOW_MAX_SPD_B + 
//        (UBusX10 - U_BUS_LOW_X10)*((s32)U_TURN1_MAX_SPD_B-U_LOW_MAX_SPD_B)/(U_BUS_TURN1_X10_B - U_BUS_LOW_X10);
//    }
//    else if(UBusX10 <= U_BUS_TURN2_X10_B)
//    {
//      MaxSpdOfStop = U_TURN1_MAX_SPD_B + 
//        (UBusX10 - U_BUS_TURN1_X10_B)*((s32)U_TURN2_MAX_SPD_B-U_TURN1_MAX_SPD_B)/(U_BUS_TURN2_X10_B - U_BUS_TURN1_X10_B);
//    }
//    else if(UBusX10 <= U_BUS_HIGH_X10)
//    {
//      MaxSpdOfStop = U_TURN2_MAX_SPD_B + 
//        (UBusX10 - U_BUS_TURN2_X10_B)*((s32)U_HIGH_MAX_SPD_B-U_TURN2_MAX_SPD_B)/(U_BUS_HIGH_X10 - U_BUS_TURN2_X10_B);
//    }
//    else // if(UBusX10 > U_BUS_HIGH_X10)
//      MaxSpdOfStop = U_HIGH_MAX_SPD;
//    
//    MaxRPMOfStop = ((uint32)MaxSpdOfStop*(MOTOR_SPEED_BASE>>2))>>13;
//  }
}

//----------------------------------------------------------------------------//
// 说明：
#if (COMMAND_MODE == CMD_MODE_PULSE)
u8  PulseSmpCnt;
u16 PulseDutySmp;
u16 PulseDutySum;

u16 PulseDutySmpOld;
u16 PulseDutySmpNew;
s16 PulseDutySmpDiff;
s16 PulseDutyAvr;

void CheckPulse(void)
{
  PulseDutySmpOld = PulseDutySmpNew;
  
  if(TIM3__DR > 0)
  {
    if(TIM3__ARR <200)
      PulseDutySmpNew = 0;
    else if(TIM3__ARR < 30000)
      PulseDutySmpNew = ((uint32)TIM3__DR*4096)/(TIM3__ARR);
    else
      PulseDutySmpNew = 4096;
  }
  else
  {
    if(GP11 == 0)
      PulseDutySmpNew = 0;
    else if(GP11 == 1)
      PulseDutySmpNew = 4096;
  }
  
  //定期清除脉冲输入信号
  TIM3__ARR       = 0;
  TIM3__DR        = 0;
  
  PulseDutySmpDiff = PulseDutySmpOld-PulseDutySmpNew;
  
  if((PulseDutySmpDiff < 512)&&(PulseDutySmpDiff > -512))
  {
    PulseDutySmp = PulseDutySmpNew;
  }
  
  PulseDutySum += PulseDutySmp;
  
  if(++PulseSmpCnt >= 4)
  {
    PulseSmpCnt = 0;
    PulseDutyAvr = PulseDutySum >> 2;
    PulseDutySum = 0;
  }

  //PulseDuty_Q12 = PulseDutySmp;
  PulseDuty_Q12 = PulseDutyAvr;
  //PulseDuty_Q12 = 4096-PulseDuty_Q12;
}
#endif


//----------------------------------------------------------------------------//
// 说明： 速度限制
// 若当前转速大于最大限制，降额处理，N次降额后仍然高速，关闭电机，待重新上电。
// 注意几个时长设置：高速非高速滤波时长、非高速保持时长、高速抑制持续时长、
u8  NormalSpdFiltCnt;
u8  MaxSpdFiltCnt;
u16 NormalSpdX10ms;
u16 MaxSpdX10ms;
u16 UltraSpdX10ms;

u8  HighSpdInhibitLv;        // 限速状态等级
u8  HighSpdRestartCnt;       // 限速重启计数
u16 HighSpdInhibitX10ms;

void SpeedRestriction(void)
{
//  
  if(HighSpdInhibitLv == 0)                           //
  {
    //if(RunningX1ms >= 100)                          // 运行n ms后
    {
      if(FOC__EOME <= MaxSpdOfStop)
      {
        if(++NormalSpdFiltCnt >= 50/10)               // 非高速计数达到一定值，确认当前非高速，清除高速计数
        {
          NormalSpdFiltCnt = 50/10;
          MaxSpdFiltCnt = 0;
          MaxSpdX10ms = 0;
        }
        
        if(++NormalSpdX10ms >= 3000/10)               // 非高速保持 N ms
        {
          NormalSpdX10ms = 3000/10;
          HighSpdRestartCnt = 0;
        }
      }
      else
      {
        if(++MaxSpdFiltCnt >= 50/10)                  // 高速计数达到一定值，确认当前高速，清除非高速计数
        {
          MaxSpdFiltCnt = 50/10;
          NormalSpdFiltCnt = 0;
          NormalSpdX10ms = 0;
        }
        
        if(HighSpdRestartCnt == 0)
        {
          if(++MaxSpdX10ms > HIHG_SPD_TIME/10)        // 高速保持Ns
          {
            MaxSpdX10ms = HIHG_SPD_TIME/10;
            HighSpdInhibitLv = 1;
          }
        }
        else
        {
          if(++MaxSpdX10ms > 100/10)                  // 高速保持Ns
          {
            MaxSpdX10ms = 100/10;
            HighSpdInhibitLv = 1;
          }
        }
        
        if(HighSpdInhibitLv == 1)
        {
          if(++HighSpdRestartCnt > HIHG_SPD_RESTARTS) 
          {
            HighSpdRestartCnt = HIHG_SPD_RESTARTS;
            HighSpdInhibitLv = 0;
            AlarmHighSpeed = 1;
            HiSpdLv = 3;
            ShutPerform();
          }
        }
      }
    }
  }
  else if(HighSpdInhibitLv == 1)
  {
    if(++HighSpdInhibitX10ms >= 500/10)   // 高速抑制持续时长，此时段内低功率运行
    {
      HighSpdInhibitX10ms = 0;
      HighSpdInhibitLv = 0;
    }
  }
  
//  // 高速标志
//  if(HighSpdInhibitLv == 1)
//    FlagHighSpeed = 1;
//  else
//    FlagHighSpeed = 0;

  // 超高速关闭
  if(FOC__EOME > ULTRA_SHUT_SPD)
  {
    if(RunningX1ms >= 500)              // 启动后运行一段时长，此操作有效
    {
      if(++UltraSpdX10ms > 1000/10)     // 超高速时长限制
      {
        UltraSpdX10ms = 0;

        AlarmHighSpeed = 1;
        HiSpdLv = 2;
        ShutPerform();
        SetFaultResatrt();
      }
    }
  }
  else
    UltraSpdX10ms = 0;
}
//----------------------------------------------------------------------------//
u16 VspIBusAlt;
s32 VspRPMAlt;
s32 VspRPMCmd;
//----------------------------------------------------------------------------//
// 说明：计算VSP参考给定
// 根据当前实际情况和给定量VSP，计算参考输出量
void CalcVSPRef(void)
{
  //----------给定功率控制----------//
  // 当电池掉电到最小值之前，线性减小功率最大值
//  if(UBusX10 >= (U_BUS_LOW_X10+4))
//    MaxPower = MAX_PWR_HI;
//  else if(UBusX10 >= U_BUS_LOW_X10)
//    MaxPower = MAX_PWR_LO+ ((UBusX10-U_BUS_LOW_X10)*(MAX_PWR_HI-MAX_PWR_LO)>>2);
//  else 
//    MaxPower = MAX_PWR_LO;

////  if(UBusX10 >= (216))
////    MaxPower = MAX_PWR_HI;
////  else if(UBusX10 >= U_BUS_LOW_X10)
////    MaxPower = MAX_PWR_LO+
////               ((UBusX10-U_BUS_LOW_X10)*(MAX_PWR_HI-MAX_PWR_LO)/(216-U_BUS_LOW_X10));
////  else 
////    MaxPower = MAX_PWR_LO;
//  
//  if(Vsp_Q12 >= VSP_ON_Q12)
//  {
//    if(Vsp_Q12 >= VSP_MAX_Q12)
//      VspPowerAlt = (u32)MaxPower*100;
//    else if(Vsp_Q12 > VSP_MIN_Q12)
//      VspPowerAlt = (u32)MinPower*100 + ((u32)100*(MaxPower - MinPower)*(Vsp_Q12-VSP_MIN_Q12))/(VSP_MAX_Q12-VSP_MIN_Q12);
////      VspPowerAlt = (u32)MinPower*100 +
////        (((u32)((100*4096)/(VSP_MAX_Q12-VSP_MIN_Q12))*(MaxPower - MinPower))*(Vsp_Q12-VSP_MIN_Q12)>>12);
//    else
//      VspPowerAlt = (u32)MinPower*100;
//  }
//  else if(Vsp_Q12 >= VSP_OFF_Q12)
//    VspPowerAlt = (u32)MinPower*100;
//  else
//    VspPowerAlt = 0;
//  
//  if(UBusX10 <= 50)
//    IBusCommand_mA = VspPowerAlt*(100/50);
//  else
//    IBusCommand_mA = ((u32)VspPowerAlt*100)/UBusX10;
//    
//  if(IBusCommand_mA > I_BUS_MAX_X1000)
//    IBusCommand_mA = I_BUS_MAX_X1000;
  //----------BLOCK END----------//
    
  //----------给定电流控制----------//
//  #define I_BUS_CMD_MAX    6000
//  #define I_BUS_CMD_MIN    1000

//  if(Vsp_Q12 >= VSP_ON_Q12)
//  {
//    if(Vsp_Q12 >= VSP_MAX_Q12)
//      VspIBusAlt = I_BUS_CMD_MAX;
//    else if(Vsp_Q12 > VSP_MIN_Q12)
//      VspIBusAlt = I_BUS_CMD_MIN + 
//        ((u32)(I_BUS_CMD_MAX - I_BUS_CMD_MIN)*(Vsp_Q12-VSP_MIN_Q12))/(VSP_MAX_Q12-VSP_MIN_Q12);
//    else
//      VspIBusAlt = I_BUS_CMD_MIN;
//  }
//  else if(Vsp_Q12 >= VSP_OFF_Q12)
//    VspIBusAlt = I_BUS_CMD_MIN;
//  else
//    VspIBusAlt = 0;

//  IBusCommand_mA = VspIBusAlt;
  //----------BLOCK END----------//
  
  //----------给定转速控制----------//
  //VspRPMAlt VspRPMCmd
//  #define RPM_CMD_MAX    80000
//  #define RPM_CMD_MIN    10000

//  if(Vsp_Q12 >= VSP_ON_Q12)
//  {
//    if(Vsp_Q12 >= VSP_MAX_Q12)
//      VspRPMAlt = RPM_CMD_MAX;
//    else if(Vsp_Q12 > VSP_MIN_Q12)
//      VspRPMAlt = RPM_CMD_MIN +
//        ((u32)(RPM_CMD_MAX - RPM_CMD_MIN)*(Vsp_Q12-VSP_MIN_Q12))/(VSP_MAX_Q12-VSP_MIN_Q12);
//    else
//      VspRPMAlt = RPM_CMD_MIN;
//  }
//  else if(Vsp_Q12 >= VSP_OFF_Q12)
//    VspRPMAlt = RPM_CMD_MIN;
//  else
//    VspRPMAlt = 0;
//    
//  VspRPMCmd = VspRPMAlt;
  //----------BLOCK END----------//
}
