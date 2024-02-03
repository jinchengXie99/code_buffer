/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : FocControl.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 2017-12-26
* Description        : This file contains all the foc control framework used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>

/* Private variables ----------------------------------------------------------------------------*/
MotStaType mcState;
MotStaM    McStaSet;

uint16 TimeCnt;

/*---------------------------------------------------------------------------*/
/* Name     :   void MC_Control(void)
/* Input    :   NO
/* Output   :   NO
/* Description: 电机状态机函数，包括初始化、预充电、顺风逆风判断、预定位、启动、运行、故障等
/*---------------------------------------------------------------------------*/
void MC_Control(void)
{
  switch(mcState)
  {
    case mcReady:    // 关闭输出,上电会对电流进行采集校准,当采样校准结束标志置1且启动指令置1后，才跳转到mcInit
      Motor_Ready();
      if((mcCurOffset.OffsetFlag == 1) && (mcSpeedRamp.FlagONOFF == 1) && (mcFocCtrl.State_Count == 0))
      {
          mcState       = mcInit;
          FOC_EFREQACC  = 0;
          FOC_EFREQMIN  = 0;
          FOC_EFREQHOLD = 0;
      }
    break;

    case mcInit:                                       // 初始化状态，进入mcCharge状态
      Motor_Init();
      mcState               =  mcCharge;               // 跳入mcCharge状态
      mcFocCtrl.State_Count = Charge_Time;
    break;

    case mcCharge:                                     // 预充电状态，MCU输出固定频率占空比，预充电结束后，跳入mcTailWind
      Motor_Charge();
      #if (IPMState == NormalRun)                      // 正常按电机状态机运行
        if( mcFocCtrl.State_Count == 0)
        {
            MOE                   = 0;                 // 关闭输出
            mcState               = mcTailWind;
            mcFocCtrl.State_Count = 0;
        }
      #endif
    break;

    case mcTailWind:
      #if (TailWind_Mode == NoTailWind)               // 无顺逆风处理的，直接跳入下一个状态
        mcState                           = mcPosiCheck;
        McStaSet.SetFlag.PosiCheckSetFlag = 0;
        mcFocCtrl.mcPosCheckAngle         = 0xffff;   // 角度赋初值

      #elif (TailWind_Mode == TailWind)
        Motor_TailWind();

      #endif
    break;

    case mcPosiCheck:
      #if (PosCheckEnable==0)                         //初始位置检测不使能时初始角度为预定位角度
        mcFocCtrl.mcPosCheckAngle = Align_Angle;
        mcState = mcAlign;
        mcFocCtrl.State_Count = Align_Time;
    
//      FOC_Init();
//       /*使能输出*/
//      DRV_CMR |= 0x3F;                         // U、V、W相输出
//      MOE = 1;
//      mcState = mcStart;                       //不加预定位开这里

      #else
        RPD();

      #endif
    break;

    case mcAlign:       // 预定位时间结束后，直接启动; AlignTestMode=1用于初始位置检测调试用
      Motor_Align();

      #if (AlignTestMode==1)
          while(1);

      #else
          if(mcFocCtrl.State_Count == 0)
          {
            mcState = mcStart;
          }
      #endif
    break;

    case mcStart:                           // 配置电机启动参数，进入mcRun状态。
      Motor_Open();
    break;

    case mcPllTect:                           // 配置电机启动参数，进入mcRun状态。
      #if (EstimateAlgorithm == PLL)
          Motor_PllStart();
      #endif
    break;

    case mcRun:                             // 运行状态，若运行状态的给定变为0，进入mcStop状态。
      if(mcSpeedRamp.FlagONOFF == 0)
      {
          mcState   = mcStop;
          mcFocCtrl.State_Count = 200;
          FOC_IQREF = 0;
        
          MOE       = 0;
          ClrBit(DRV_CR, FOCEN);  //关闭FOC
          FOC_CR1   = 0x00;
      }
    break;

    case mcStop:
        #if (StopBrakeFlag == 0)
        {
            FOC_CR1 = 0x00;
            /*关闭FOC*/
            ClrBit(DRV_CR, FOCEN);
  
            mcState = mcReady;
        }
        #else
        {
            if(mcFocCtrl.State_Count==0) //延时刹车，这里更改用来跑高速电机启停
            {
                MOE      = 0;
                FOC_CR1  = 0x00;
                ClrBit(DRV_CR, FOCEN);
                DRV_DR   = DRV_ARR*1;
                DRV_CMR &= 0xFFC0;
                DRV_CMR |= 0x015;                                                // 三相下桥臂通，刹车
                ClrBit(DRV_CR, OCS);                                             // OCS = 0, DRV_COMR;OCS = 1, FOC/SVPWM/SPWM
                MOE      = 1;
                mcState  = mcBrake;
                mcFocCtrl.State_Count = 400;//StopWaitTime;
            }
        }
        #endif
    break;

    case mcBrake:
      if(mcFocCtrl.State_Count == 0)
      {
        MOE=0;
        ClrBit(DRV_CR, FOCEN);
        mcState = mcReady;
      }
    break;

    case mcFault:
    break;
  }
}
