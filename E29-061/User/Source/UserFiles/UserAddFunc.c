/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : AddFunction.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 2017-12-26
* Description        : This file contains all the add function used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/
// #include <AddFunction.h>
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

#include <UserGlobal.h>
#include <UserDefine.h>

/* Private variables ---------------------------------------------------------*/

extern uint16  POWER_VSP;
extern u8  DrvEnFlag;
extern u8  KeyOnOffCmd;
extern u8  ReStartDlyCmd;
extern u16 ReStartDlyX10ms;
extern u16 RunningX1ms;
extern u8  ReStartCnt;
extern u16 IBusCommandAlt;
extern u16 IBusCommandAlt;
extern u16 IBusFilt_mA;
extern u8  StuckLv;                   // 堵转等级
extern u8  StartFailLv;               // 启动异常等级
extern s32 VspRPMCmd;

extern s16 idata Vsp_Q12;
extern s16 idata VspSpdAlt;
extern u8  idata AlarmStuck;          // 6 卡住
extern u8  idata AlarmStartLose;      // 7 启动计算异常
extern u32 idata MotorRPM;
extern u32 idata VspPwrCmd;
extern u32 idata UxISmp;
//----------------------------------------------------------------------------//
// 说明：
void ShutPerform(void)
{
  MOE     = 0;
  ClrBit(DRV_CR, FOCEN);  //关闭FOC
  mcState = mcReady;
  
  DrvEnFlag = 0;
  Vsp_Q12 = 0;
  //KeyGoCmdOn = 0;
  KeyOnOffCmd = 0;
}

//----------------------------------------------------------------------------//
// 说明：
void CancleResatrt(void)
{
  ReStartDlyCmd = 0;
  ReStartDlyX10ms = 0;
}

//----------------------------------------------------------------------------//
// 说明：
void SetFaultResatrt(void)
{
  if(ReStartDlyCmd == 0)
  {
    ReStartDlyCmd = 1;
  
    if((RunningX1ms < 2500)&&(ReStartCnt < FLT_START_TIMES))
    {
      if(ReStartDlyX10ms == 0)
        ReStartDlyX10ms = 10;
    }
    else
      CancleResatrt();
  }
}

////----------------------------------------------------------------------------//
//// 说明：
//u8  BrkStage = 0;
//u16 MtBrakeCnt = 0;
//void MotorBrake(void)
//{
//  if(++MtBrakeCnt >= 500)
//  {
//    MtBrakeCnt = 0;
//    mcState = mcStop;
//  }
//  
//  if(BrkStage == 0)
//  {
//    if((mcFocCtrl.mcSysSpeed < BRAKE_SPEED)||                 //_Q15(60000.0/MOTOR_SPEED_BASE)
//       (MtBrakeCnt > 300))
//      BrkStage = 1;
//  }
//  else
//  {
//    FOC_SWDUTY = BRAKE_INIT_DUTY;
//    FOC_CR1 = 0x06;                   // FOC计数器使能，软件写PWM占空比
//    FOC_CMR = 0x15;                   // U相输出  0x15
//    MOE = 1;
//  }
//}

//----------------------------------------------------------------------------//
// 说明：母线电流PI调节
void IBusPI(int16 err)
{
  //PI_UKMAX = 2000;
  PI_EK = err;
  PI_LPF_CR |= 0x02;                  // Start PI
  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
  //PI_UK+=(SKP/4096 +1);
  FOC_IQREF = PI_UK;
  
  //PI_LPF_CR &= 0xFD;                  // stop PI
  
//    PI_EK =  Xn1;         //填入EK
//    PI_LPF_CR |= 0x02;    // Start PI
//    _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
//    PI_UK+=(SKP/4096 +1);
//    return PI_UK;
}

void EK_deal(void)
{
  if(FOC_EK2 < OBS_K2T_Actual-10)
  {
    mcFocCtrl.Smo_EK2=FOC_EK2;
    mcFocCtrl.Smo_EK2+=10;
    FOC_EK2=mcFocCtrl.Smo_EK2;
  }
  else if(FOC_EK2 > OBS_K2T_Actual+10)
  {
    mcFocCtrl.Smo_EK2=FOC_EK2;
    mcFocCtrl.Smo_EK2-=10;
    FOC_EK2=mcFocCtrl.Smo_EK2;
  }
  else
  {
    FOC_EK2=OBS_K2T_Actual;
  }
}
//----------------------------------------------------------------------------//
// 说明：功率调节
// 将当前母线电压电流检测值相乘，乘积与给定对比，根据比较所得调节电流给定量，使功率达到要求
s16 IBusError;
s16 IBusPIErr;
s32 SpdErr;
s16 SpdPIErr;
s32 UIErr;
s16 UIPIErr;
u16 focek2;

void Regulation(void)
{
  if((mcState ==mcRun)||(mcState ==mcStop))
  {
    switch(mcFocCtrl.CtrlMode)
    {
      case 0:
      {
        if(FOC__EOME > Motor_Loop_Speed)
        {
          FOC_DQKP = DQKP;
          FOC_DQKI = DQKI;
          
          PI_UKMAX   = SOUTMAX;
          PI_UKMIN   = SOUTMIN;
          
          
          #if (CONTROL_TARGET==CONTROL_TARGET_POWER)
          {
            mcFocCtrl.CtrlMode = 1;
            
            mcFocCtrl.mcIqref = FOC_IQREF;
            PI_UK = mcFocCtrl.mcIqref;
          
            PI_KP = _Q12(0.3);
            PI_KI = _Q12(0.01);
          }
          #elif (CONTROL_TARGET==CONTROL_TARGET_SPEED)
          {
            mcFocCtrl.CtrlMode = 2;
              
            mcFocCtrl.mcIqref = FOC_IQREF;
            PI_UK = mcFocCtrl.mcIqref;
            
            PI_KP = _Q12(0.4);
            PI_KI = _Q12(0.01);
            
            //VspSpdAlt = 10000;//mcFocCtrl.SpeedFlt;//
          }
          #endif
          
          mcFocCtrl.SpeedRamp = 2;
          mcFocCtrl.SpeedLoop = 5;
          //OutLoopParameterSet();       //环路基本参数设置
          
        }
      }
        break;
      case 1:
      {
        #if (OUTLoop_Mode== OUTLoop_Disable)
        {
          mcFocCtrl.TorqueLoopTime++;
          if(mcFocCtrl.TorqueLoopTime>SPEED_LOOP_TIME)
          {
            mcFocCtrl.TorqueLoopTime=0;
            mcFocCtrl.mcIqref = FOC_IQREF;
            if (FOC_IQREF < QOUTVALUE)
            {
              mcFocCtrl.mcIqref += QOUTINC;
              if (mcFocCtrl.mcIqref > QOUTVALUE) mcFocCtrl.mcIqref = QOUTVALUE;
              FOC_IQREF = mcFocCtrl.mcIqref;
            }
            else if (FOC_IQREF > QOUTVALUE)
            {
              mcFocCtrl.mcIqref -= QOUTINC;
              if (mcFocCtrl.mcIqref < QOUTVALUE) mcFocCtrl.mcIqref = QOUTVALUE;
              FOC_IQREF = mcFocCtrl.mcIqref;
            }
          }
        }
        #elif (OUTLoop_Mode== OUTLoop_Enable)
        {
         
          //PI_UKMAX = SOUTMAX;
          //PI_UKMIN = SOUTMIN;
          
          //PI_KP    = _Q12(0.9);
          //PI_KI    = _Q12(0.010);

          //功率调节方法1：----------------------------------------------------//
//          //IBusPIErr = 150 - IBusFilt_mA;         //调试中，直接对母线电流参考值赋值
//          IBusPIErr = IBusCommandAlt - IBusFilt_mA;
//          IBusError = IBusPIErr;

//          #define PWR_PI_ERR_MAX       200  // 为防止PI调节溢出，对输入做限制
//          #define PWR_PI_ERR_MIN       10   // 为防止PI调节截断，对输入做限制
//          //if(RunningX1ms > 1000)             // 为获得大转矩，启动不限制；为平稳，运行时限制
//          {
//            if(IBusPIErr > 0)
//            {
//              if(IBusPIErr > PWR_PI_ERR_MAX)
//                IBusError = PWR_PI_ERR_MAX;
//              else if(IBusPIErr < PWR_PI_ERR_MIN)
//                IBusError = PWR_PI_ERR_MIN;
//            }
//            else if(IBusPIErr < 0)
//            {
//              if(IBusPIErr < -PWR_PI_ERR_MAX)
//                IBusError = -PWR_PI_ERR_MAX;
//              else if(IBusPIErr > -PWR_PI_ERR_MIN)
//                IBusError = -PWR_PI_ERR_MIN;
//            }
//          }
//          IBusError = IBusError*12;
//          IBusPI(IBusError);
                    
          //功率调节方法2：----------------------------------------------------//
          // UxISmp是电压电流的采样结果直接相乘，
          // VspPwrCmd是功率参考值，对应电压电流的采样值格式
          
          //VspPwrCmd = MAX_POWER_UXI_SMP;//调试中，直接对功率参考值赋值
          UIErr = VspPwrCmd - UxISmp;
          
          #define PWR_PI_ERR_MAX       10000  // 为防止PI调节溢出，对输入做限制
          
          if(UIErr >= 0)
          {
            if(UIErr >= PWR_PI_ERR_MAX)
              UIErr = PWR_PI_ERR_MAX;
          }
          else
          {
            if(UIErr <= -PWR_PI_ERR_MAX)
              UIErr = -PWR_PI_ERR_MAX;
          }
          
          UIPIErr = UIErr;
          IBusPI(UIPIErr*3);

          //功率调节方法结束----------------------------------------------------//
        }
        #endif //END OUTLoop_Mode

        EK_deal();
      }
      break;
      case 2:
      {
        if(++mcFocCtrl.SpeedLoopTime >= 5)
        {
          mcFocCtrl.SpeedLoopTime = 0;
          ////
          //PI_UKMAX = SOUTMAX;
          //PI_UKMIN = SOUTMIN;
          //PI_KP    = _Q12(0.2);
          //PI_KI    = _Q12(0.002);
          
//          VspRPMCmd = 90000;
//          SpdErr = VspRPMCmd - MotorRPM;//
          SpdErr =  VspSpdAlt - mcFocCtrl.SpeedFlt;//FOC__EOME;(s32)(s32) 

          #define SPD_ERR_MAX 15000
          if(SpdErr > SPD_ERR_MAX)
            SpdPIErr = SPD_ERR_MAX;
          else if(SpdErr < -SPD_ERR_MAX)
            SpdPIErr = -SPD_ERR_MAX;
          else
            SpdPIErr = (s16)SpdErr;
          
          IBusPI(SpdPIErr);
          
          EK_deal();
        }
      }
    }
  }
}

//----------------------------------------------------------------------------//
// 启动失败检测
void CheckStartFault(void)
{
  /*******启动保护恢复*********/
//  h_Fault->mcEsValue = FOC__ESQU;

  if(mcState == mcRun)
  {
    //方法一，5s内速度大于最大速度，同时反电动势值低于一定值
    if(mcFaultDect.StartSpeedCnt <= 2000/2)
    {
      mcFaultDect.StartSpeedCnt++;
      if((mcFocCtrl.SpeedFlt > Motor_Max_Speed)&&(mcFocCtrl.EsValue < 20))
      {
        mcFaultDect.StartSpeedCnt = 0;
        mcProtectTime.StartFlag = 1;
        
        AlarmStartLose = 1;
        StartFailLv = 1;
        ShutPerform();
        SetFaultResatrt();
      }
    }
      
   //方法二
    if(mcFaultDect.StartEsCnt <= 2000/2)//前6s，等待1.5s后，开始判断ES，如果超过一定次数，则失败
    {
      mcFaultDect.StartEsCnt++;
      mcFaultDect.StartDelay++;
      if(mcFaultDect.StartDelay >= 1500)        // 1.5s
      {
        mcFaultDect.StartDelay = 1500;
        if((mcFocCtrl.EsValue < 5))//&&(mcFocCtrl.CtrlMode==0))
        {
            mcFaultDect.StartESCount++;
            if(mcFaultDect.StartESCount >= 20)
            {
              mcFaultDect.StartDelay = 0;
              mcFaultDect.StartESCount = 0;
              mcProtectTime.StartFlag = 2;
              
              AlarmStartLose = 1;
              StartFailLv = 2;
              ShutPerform();
              SetFaultResatrt();
            }
        }
        else
        {
          if(mcFaultDect.StartESCount > 0)
            mcFaultDect.StartESCount--;
        }
     }
   }
   else
   {
     mcFaultDect.StartESCount = 0;
   }
    //方法三，长时间在CtrlMode=0状态
    if(mcFocCtrl.CtrlMode == 0)         //
    {
      mcFaultDect.StartFocmode++;
      if(mcFaultDect.StartFocmode >= 800/2)
      {
        mcFaultDect.StartFocmode = 0;
        mcProtectTime.StartFlag = 3;
        
        AlarmStartLose = 1;
        StartFailLv = 3;
        ShutPerform();
        SetFaultResatrt();
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
//堵转保护
void CheckStallFault(void)             //void Fault_Stall(FaultVarible *h_Fault)
{
  // 只在运行中检测
  if(mcState == mcRun)
  {
    if(mcFaultDect.StallDelayCnt <= 2000/2)//5s
    {
      mcFaultDect.StallDelayCnt ++;
    }
    else
    {
      //method 1，判断反电动势太小或当反电动势太小，转速太大
      if((mcFocCtrl.EsValue < 10)||
         ((FOC__EOME > _Q15(800.0/MOTOR_SPEED_BASE))&&(mcFocCtrl.EsValue < 40)))
      {
        mcFaultDect.StallDectEs++;
        if(mcFaultDect.StallDectEs >= 10)
        {
          mcFaultDect.StallDectEs = 0;
          mcProtectTime.StallFlag = 1;

          StuckLv = 1;
          AlarmStuck = 1;
          ShutPerform();
          SetFaultResatrt();
          
        }
      }
      else
      {
        if(mcFaultDect.StallDectEs > 0)
          mcFaultDect.StallDectEs--;
      }
      //method 2，判断速度低于堵转最小值或者超过堵转最大值
      if((mcFocCtrl.SpeedFlt < Motor_Stall_Min_Speed)||(mcFocCtrl.SpeedFlt > Motor_Stall_Max_Speed))
      {
        mcFaultDect.StallDectSpeed++;
        if(mcFaultDect.StallDectSpeed >= 8)
        {
          mcFaultDect.StallDectSpeed = 0;
          mcProtectTime.StallFlag = 2;
          
          if(mcFocCtrl.SpeedFlt < Motor_Stall_Min_Speed)
            StuckLv = 2;
          else
            StuckLv = 3;
          AlarmStuck = 1;
          ShutPerform();
          SetFaultResatrt();
        }
      }
      else
      {
        if(mcFaultDect.StallDectSpeed > 0)
          mcFaultDect.StallDectSpeed--;
      }
    }
  }
}
