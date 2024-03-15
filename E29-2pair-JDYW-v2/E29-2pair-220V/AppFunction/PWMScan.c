/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： PWMScan.c
 * 文件标识：
 * 内容摘要： PWMScan
 * 其它说明： PWM chenk , motor direction
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2019年12月27日
 *
 *******************************************************************************/
#include "PWMScan.h"
#include "lks32mc08x.h"
#include "lks32mc08x_tim.h"
#include "parameter.h"
#include "state_machine.h"
#include "MC_type.h"
#include "power.h"
#include "fault_detection.h"

stru_PWMTime_t  PWMTime;

extern u16 g_nMotorRunDir;
/*****************************************************************************
 * 函数名   : PWMScan(void)
 * 说明     : 检测输入PWM的占空比和周期，可以用作占空比调速和频率调速
 * 设计思路 ：1.将Timer CH0和CH1连接到一起，CH0用作捕捉高电平计数，CH1用作捕捉周期计数; \
 *          : 2.针对不接PWM的情况，外围电路要接上拉或者下拉
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PWMScan(void)
{
  if(UTIMER_IF & BIT5)//T1_CH1_IF Timer1 CH1下降沿中断置1
  {
    PWMTime.nFlagPeriod = 1;
    PWMTime.nHighLevelTime = (s16)UTIMER_UNT1_CMP1 - (INT16)UTIMER_UNT1_CMP0; //捕捉高电平计数值
    PWMTime.nPeriod = (s16)UTIMER_UNT1_CMP1 - PWMTime.nPreTime;                //捕捉周期计数值
    PWMTime.nPreTime = (s16)UTIMER_UNT1_CMP1;

    PWMTime.nFlagReset = 0;
  }

  if(UTIMER_IF & BIT3)                     //Timer1过零中断置1
  {
    if(!PWMTime.nFlagPeriod)              //如果检测不到CH1下降沿变化，则为PWM不接的情况
    {
      if(PWM_HIGH)                     //输入IO口电平为高，占空比为100%
      {
        PWMTime.nDuty  = DUTY_FULL;//100%
        PWMTime.nPeriod = PWM_TIME_PERIOD; //10k

        PWMTime.nFlagReset = 0;
      }
      else                             //输入IO口电平为低，占空比为0%
      {
        PWMTime.nDuty  = DUTY_ZERO;//0%
        PWMTime.nPeriod = PWM_TIME_PERIOD;
//                PWMTime.FlagReset = 1;
      }
    }

    PWMTime.nFlagPeriod = 0;

  }
}

/*****************************************************************************
 * 函数名   : PWMnDutyScan(void)
 * 说明     : PWM占空比计算
 * 设计思路 ：1.根据高电平和周期的计数来计算占空比，采用DSP计算，缩短计算时间
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PWMnDutyScan(void)
{
  if(PWMTime.nFlagPeriod)
  {
    __disable_irq();/*关闭中断，中断总开关*/

    DSP_DID = ((INT32)PWMTime.nHighLevelTime * 1000 + ((INT32)PWMTime.nPeriod >> 1));  //调用DSP计算占空比
    DSP_DIS =  (INT32)PWMTime.nPeriod;
    delay(15);
    PWMTime.nDuty = DSP_QUO;

    __enable_irq();/*开启中断*/
  }

  /***********芯片软复位操作*******************/
//    if(PWMTime.FlagReset)
//    {
//        PWMTime.ResetCntr ++;
//        if(PWMTime.ResetCntr > RESET_TIME)
//        {
//            PWMOutputs(DISABLE);
//            __disable_irq();/*关闭中断，中断总开关*/
//            FG_LOW;
//            PWMTime.ResetCntr = 0;
//            REG32(0xE000ED0C) = 0x05FA0004; //MCU 软复位
//        }
//    }
//    else
//    {
//        PWMTime.ResetCntr = 0;
//    }


}

/*****************************************************************************
 * 函数名   : PWMScanInit(void)
 * 说明     : PWM占空比检测初始化
 * 设计思路 ：1.变量初始化
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PWMScanInit(void)
{
  PWMTime.nDuty = 0;
  PWMTime.nFlagPeriod = 0;
  PWMTime.nHighLevelTime = 0;
  PWMTime.nLowLevelTime = 0;
  PWMTime.nPeriod = 0;

  PWMTime.nFlagReset = 0;
  PWMTime.nResetCntr = 0;
}

/*****************************************************************************
 * 函数名   : SpeedScan(void)
 * 说明     : 给定速度计算
 * 设计思路 ：1.根据检测到的占空比计算对应转速
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void SpeedScan(void)
{
  if(PWMTime.nDuty < PWM_MIN)
  {
    AppCommData.wSpeedRef = SPEED_MIN;
  }
  else if(PWMTime.nDuty < PWM_MAX)
  {
    AppCommData.wSpeedRef = (SPEED_SCALE * (PWMTime.nDuty - PWM_MIN)) + SPEED_MIN;
  }
  else
  {
    AppCommData.wSpeedRef = SPEED_MAX;
  }
}

/*****************************************************************************
 * 函数名   : MotorDirScan(void)
 * 说明     : 电机运行方向转换
 * 设计思路 ：1.根据方向检测IO的电平来确定电机转向
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void MotorDirScan(void)
{
  static s16 s16DirectionCnt;

  if(DIR_CW)                              //IO电平为高时为CW
  {
    s16DirectionCnt ++;
    if(s16DirectionCnt >= DIR_FLITER)   //消抖
    {
      s16DirectionCnt = DIR_FLITER;
      g_nMotorRunDir = CW;
    }
  }
  else                                     //IO电平为低时为CCW
  {
    s16DirectionCnt --;
    if(s16DirectionCnt <= -DIR_FLITER)
    {
      s16DirectionCnt = -DIR_FLITER;
      g_nMotorRunDir = CCW;
    }
  }

  if(AppCommData.nMotorRunDirPre != g_nMotorRunDir)   //检测到的转向和当前转向不一致，则切换到ilde状态处理。
  {
    ControlState = idle;
  }

  AppCommData.nMotorRunDirPre = g_nMotorRunDir;
}

/*****************************************************************************
 * Function:     void FGScan(void)
 * Description:  FG  机械一圈6个脉冲，2对极 == 一个电周期3个脉冲
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
/*****************************************************************************
 * 函数名   : FGScan(void)
 * 说明     : 电机转速输出信号
 * 设计思路 ：1.机械一圈6个脉冲，2对极 == 一个电周期3个脉冲，需要根据实际需求处理。     \
 *          : 2.对于FG频率和电机电频率有整数倍关系的采用现有的方法，即通过角度来翻转IO; \
 *          : 3.对于特殊频率的FG，建议采用Timer处理。通过转速来计算Timer的周期值。
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void FGScan(u16 nTheta)
{
  if((nTheta > 0) && (nTheta <= 10922))          //0<Theta<60
  {
    FG_HIGH;
  }
  else if((nTheta > 10922) && (nTheta <= 21844)) //60<Theta<120
  {
    FG_LOW;
  }
  else if((nTheta > 21844) && (nTheta <= 32766)) //120<Theta<180
  {
    FG_HIGH;
  }
  else if((nTheta > 32766) && (nTheta <= 43688)) //180<Theta<240
  {
    FG_LOW;
  }
  else if((nTheta > 43688) && (nTheta <= 54610)) //240<Theta<300
  {
    FG_HIGH;
  }
  else if((nTheta > 54610) && (nTheta <= 65535)) //300<Theta<360
  {
    FG_LOW;
  }

}

/*****************************************************************************
 * 函数名   : PowerScan(void)
 * 说明     : 给定功率计算
 * 设计思路 ：1.根据检测到的电位器电压计算对应转速
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PowerScan(void)
{
  if(PWMTime.nVSPValue < VSP_MIN)
  {
    if(Faults.R == 0)
    {
      g_bMC_RunFlg = 0;
    }
  }
  else if(PWMTime.nVSPValue < VSP_MAX)
  {
    PWMTime.wPowerValue = (POWER_SCALE * (PWMTime.nVSPValue - VSP_MIN)) + POWER_MIN;
    if((g_bMC_RunFlg == 0) && (Faults.R == 0))
    {
      g_bMC_RunFlg = 1;
    }
  }
  else
  {
    PWMTime.wPowerValue = POWER_MAX;
    if((g_bMC_RunFlg == 0) && (Faults.R == 0))
    {
      g_bMC_RunFlg = 1;
    }
  }

}

