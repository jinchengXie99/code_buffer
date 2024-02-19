/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： RFControl.c
 * 文件标识：
 * 内容摘要： RFControl
 * 其它说明：
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2020年2月21日
 *
 *******************************************************************************/
#include "RFControl.h"
#include "lks32mc08x_flash.h"
#include "lks32mc08x_tim.h"
#include "mc_declaration.h"
#include "parameter.h"
#include "fault_detection.h"
#include "lks32mc08x_tim.h"

RFSystem_t RFState;
RFProcess_t RFHandle;

/*****************************************************************************
 * Function:     void RFReceive(void)
 * Description:  RFReceive
 * Parameter:    data receive  period 62.5us
 * Return:       no
 *****************************************************************************/
//INT16  DataTestCntr;
//UINT32 DataTest[6];
void RFReceive(void)
{
  if(RFLEVEL_HIGH)//level 1
  {
    if(RFState.PeriodFlag)
    {
      RFState.PeriodFlag = 0;
      RFState.Period = RFState.HighLevelCntr + RFState.LowLevelCntr;
      if((RFState.Period > CODE_PERIOD_TIME_MIN)&&(RFState.Period < CODE_PERIOD_TIME_MAX))
      {
        if((RFState.HighLevelCntr > CODE0_HIGH_TIME_MIN)&&(RFState.HighLevelCntr < CODE0_HIGH_TIME_MAX))//data 0
        {
          if(RFState.DataCntr < DATA_LENGTH)
          {
            RFState.DataCntr++;
          }
          else
          {
//              DataTest[DataTestCntr] = RFState.DataBuffer.R;
//              DataTestCntr++;
//              if(DataTestCntr > 6)
//              {
//                DataTestCntr = 0;
//              }

            if(RFState.DataBufferTime == 0)
            {
              RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              RFState.DataBufferTime = 1;
            }
            else  //第二次数据
            {
              if(RFState.DataBufferCheck.R == RFState.DataBuffer.R)
              {
                RFState.DataBufferTime = 0;
//                RFState.RXSuccessFlag  = 1;
                RFState.DataBufferCheck.R = 0;
//                RFState.DataBufferCntr = 0;
                if(RFState.Data.R != RFState.DataBuffer.R)
                {
                  RFState.Data.R = RFState.DataBuffer.R;
                  if(RFState.DataDelayCntr == 0)
                  {
                    RFState.RXSuccessFlag  = 1;
                    RFState.DataDelayCntr = DATA_DELAY_TIME;
                  }
//                  RFState.RXSuccessFlag  = 1;
                }
              }
              else
              {
                RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              }
            }

            RFState.DataCntr = 0;
            RFState.DataBuffer.R = 0;
          }
        }
        else if((RFState.HighLevelCntr > CODE1_HIGH_TIME_MIN)&&(RFState.HighLevelCntr < CODE1_HIGH_TIME_MAX))//data 1
        {
          if(RFState.DataCntr < DATA_LENGTH)
          {
            RFState.DataBuffer.R += (1 << (DATA_LENGTH-RFState.DataCntr));//MSB
//            RFState.DataBuffer.R += (1 << RFState.DataCntr);  //LSB
            RFState.DataCntr++;
          }
          else
          {
            RFState.DataBuffer.R += 1;//MSB
//            RFState.DataBuffer.R += (1<< RFState.DataCntr); //LSB

//              DataTest[DataTestCntr] = RFState.DataBuffer.R;
//              DataTestCntr++;
//              if(DataTestCntr > 6)
//              {
//                DataTestCntr = 0;
//              }
//
            if(RFState.DataBufferTime == 0) //第一次数据
            {
              RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              RFState.DataBufferTime = 1;
            }
            else     ////读取数据 & 校验 2次数据相同
            {
              if(RFState.DataBufferCheck.R == RFState.DataBuffer.R)
              {
                RFState.DataBufferTime = 0;
//                RFState.RXSuccessFlag  = 1;
                RFState.DataBufferCheck.R = 0;
//                RFState.DataBufferCntr = 0;
                if(RFState.Data.R != RFState.DataBuffer.R)
                {
                  RFState.Data.R = RFState.DataBuffer.R;

                  if(RFState.DataDelayCntr == 0)
                  {
                    RFState.RXSuccessFlag = 1;
                    RFState.DataDelayCntr = DATA_DELAY_TIME;
                  }
//                  RFState.RXSuccessFlag  = 1;
                }
              }
              else
              {
                RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              }

            }

            RFState.DataCntr = 0;
            RFState.DataBuffer.R = 0;
          }
        }
        else //noise
        {
          RFState.DataCntr = 0;
          RFState.DataBuffer.R = 0;
        }
      }
      else //noise
      {
        RFState.DataCntr = 0;
        RFState.DataBuffer.R = 0;
      }

      RFState.HighLevelCntr = 0;
      RFState.LowLevelCntr  = 0;
    }

    if(RFState.HighLevelCntr < CODE1_HIGH_TIME_MAX)
    {
      RFState.HighLevelCntr ++;
    }
    else //noise
    {
      RFState.DataCntr = 0;
      RFState.DataBuffer.R = 0;
//      RFState.DataBufferCheck.R = 0;
    }

  }
  else //level 0
  {

    if(RFState.LowLevelCntr < CODE_LOW_TIME_MAX)
    {
      RFState.LowLevelCntr++;
      RFState.PeriodFlag = 1;
    }
    else  //noise
    {
      RFState.DataCntr = 0;
      RFState.DataBuffer.R = 0;
      RFState.HighLevelCntr = 0;
    }

  }

  if(RFState.DataBufferTime) //300ms 不能接收到2帧正确数据则重新接收
  {
    if(RFState.DataBufferCntr < DATA_CHECK_TIME)
    {
      RFState.DataBufferCntr ++;
    }
    else
    {
      RFState.DataBufferCntr = 0;
      RFState.DataBufferTime = 0;
      RFState.DataBufferCheck.R = 0;
    }
  }
  else
  {
    RFState.DataBufferCntr = 0;
  }

}


/*****************************************************************************
 * Function:     void RFCommand(void)
 * Description:  RF Command
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void RFCommand(void)
{
  if(RFState.RXSuccessFlag)
  {
    RFState.RXSuccessFlag = 0;
    RFState.DataBufferCntr = 0;
//    RFState.DataDelayCntr = DATA_DELAY_TIME;
    //校验
    RFState.CheckData = (RFState.Data.B.FunctionCode0<<3) + RFState.Data.B.PID;
    RFState.CheckData = RFState.CheckData ^ RFState.Data.B.CustomerCode0 ^ RFState.Data.B.CustomerCode1 ^ RFState.Data.B.CustomerCode2 ^
                        RFState.Data.B.CustomerCode3 ^ RFState.Data.B.CustomerCode4 ^ RFState.Data.B.FunctionCode1 ^ 0xA;
    if(RFState.CheckData == RFState.Data.B.Check) //校验正确
    {
      RFState.KeyCode = ((UINT16)RFState.Data.B.FunctionCode1 << 1) + (UINT16)RFState.Data.B.FunctionCode0;//取按键码

      RFState.CustomerIDBuffer = (RFState.Data.R & 0xFFFFF000); //读取ID

      if(RFState.IDMatchCntr < ID_MATCH_TIME) //5s内对码
      {
        if((RFState.KeyCode == ID_MATCH)&&(RFState.IDMatchFlag == 0)) //对码
        {
          if((RFState.CustomerIDBuffer != RFState.CustomerID[0])&&(RFState.CustomerIDBuffer != RFState.CustomerID[1]))//存储ID
          {
            RFState.CustomerID[0] = RFState.CustomerID[1];
            RFState.CustomerID[1] = RFState.CustomerIDBuffer;

            __disable_irq();

            EraseSector(ID_ADDRESS);
            ProgramPage(ID_ADDRESS,8,(unsigned char*)RFState.CustomerID);

            __enable_irq();
            RFState.IDMatchFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
        }
      }

      if((RFState.CustomerIDBuffer == RFState.CustomerID[0])||(RFState.CustomerIDBuffer == RFState.CustomerID[1])) //ID match
      {
//        RFState.BZTime = BZ_ON_TIME;

        switch(RFState.KeyCode)
        {
        case IR_LED: //P0.11    灯控
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            if(RFHandle.LedFlag == 0)
            {
              UTIMER_UNT3_CMP0 = 48000;
              RFHandle.LedFlag = 1;
            }
            else if(RFHandle.LedFlag == 1)
            {
              UTIMER_UNT3_CMP0 = 96001;
              RFHandle.LedFlag = 0;
            }

            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_ON_OFF:         //开关
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.OnOffFlag = FAN_OFF;
            TIM_TimerCmd(TIMER3, DISABLE);//关灯
            RFInit();
            g_bMC_RunFlg = 0;
            ControlState = idle;
          }
          else if(RFHandle.OnOffFlag == FAN_OFF)
          {
            if(Faults.R == 0)
            {
              RFHandle.OnOffFlag = FAN_ON;
            }
//              g_bMC_RunFlg = 1;
          }

          RFState.BZTime = BZ_ON_TIME;

          break;
        }
        case IR_SPEED1:   //1档
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;
            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED1);

            RFState.BZTime = BZ_ON_TIME;
            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED2:    //2档
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED2);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED3:   //3档
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED3);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED4:     //4档
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED4);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED5:     //5档
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED5);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_NATUREWIND:   //自然风
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {

            RFHandle.NatureWindFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_1H:       ////定时1h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_1H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_2H: ////定时2h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_2H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_3H://定时3h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_3H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_8H:    //定时8h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_8H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_DIR: //方向切换
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            if(RFHandle.FanDirection == CW)
            {
              RFHandle.FanDirection = CCW;
              g_nMotorRunDir = RFHandle.FanDirection;
            }
            else if(RFHandle.FanDirection == CCW)
            {
              RFHandle.FanDirection = CW;
              g_nMotorRunDir = RFHandle.FanDirection;
            }

            RFHandle.DirectionCntr = DIR_DELAY_TIME;
            RFHandle.DirectionFlag = 1;
            ControlState = idle;//进入idle状态，延时DIR_DELAY_TIME 重启
            g_bMC_RunFlg = 0;

            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case ID_MATCH:
        {
//            if(RFState.IDMatchFlag == 0)
//            {
//              RFState.BZTime = BZ_ON_TIME;
//            }
          break;
        }
        default:
          break;
        }
      }
    }
  }

  if(RFState.IDMatchCntr < ID_MATCH_TIME)//5s内不能对码，重新上电。
  {
    RFState.IDMatchCntr ++;
  }


  if(RFState.BZTime > 0) //蜂鸣器时间
  {
    RFState.BZTime --;
  }

  if(RFHandle.DirectionFlag)
  {
    if(RFHandle.DirectionCntr > 0) //方向切换时间
    {
      RFHandle.DirectionCntr--;
    }
    else
    {
      if(AppCommData.wSpeedRef != 0)
      {
        g_bMC_RunFlg = 1;
      }
      RFHandle.DirectionFlag = 0;
    }
  }

  if(RFState.DataDelayCntr > 0)//接收到正确数据后的延时
  {
    RFState.DataDelayCntr--;
  }

  NatureWindControl(); //自然风

  FanRunTime();//定时

}

/*****************************************************************************
 * Function:     void NatureWindContor(void)
 * Description:  Nature Wind 1 3 5 3 5 4档各20s
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void NatureWindControl(void)
{
  if(RFHandle.NatureWindFlag)
  {
    if(RFHandle.NatureWindCntr < NATUREWIND_TIME)
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED1);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*2))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED3);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*3))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED5);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*4))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED2);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*5))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED5);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*6))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED4);

    }
    else
    {
      RFHandle.NatureWindCntr = 0;
    }

    RFHandle.NatureWindCntr ++;
  }
  else
  {
    RFHandle.NatureWindCntr = 0;
  }
}

/*****************************************************************************
 * Function:     void FanRunTime(void)
 * Description:  风扇运行定时  1H 2H 3H 8H
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void FanRunTime(void)
{
  if(RFHandle.PowerDelayFlag)
  {
    if(RFHandle.PowerOnCntr > 0)
    {
      RFHandle.PowerOnCntr--;
    }
    else
    {
      RFHandle.OnOffFlag = FAN_OFF;
      RFHandle.PowerDelayFlag = 0;
      RFInit();
      g_bMC_RunFlg = 0;
    }
  }
}
/*****************************************************************************
 * Function:     void BZContor(void)
 * Description:   Contor BZ  3k hz
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
UINT8 BZCntr = 0;
void BZControl(void)
{
  if(RFState.BZTime > 0)
  {
    if(BZCntr < 2)
    {
      BZ_ON;
    }
    else if(BZCntr < 4)
    {
      BZ_OFF;
    }
    else
    {
      BZCntr = 0;
    }

    BZCntr++;
  }
  else
  {
    BZ_ON;
    BZCntr = 0;
  }
}

/*****************************************************************************
 * Function:     void RFInit(void)
 * Description:  RF  Init
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void RFInit(void)
{
  RFState.CheckData = 0;
  RFState.Data.R = 0;
  RFState.DataBuffer.R = 0;
  RFState.DataBufferCheck.R = 0;
  RFState.DataBufferCntr = 0;
  RFState.DataBufferTime = 0;
  RFState.DataCntr = 0;
  RFState.PeriodFlag = 0;
  RFState.RXSuccessFlag = 0;
  RFState.HighLevelCntr = 0;
  RFState.LowLevelCntr = 0;
  RFState.Period = 0;
  RFState.KeyCode = 0;

//RF Handle
  RFHandle.PowerOnCntr = 0;
  RFHandle.NatureWindCntr = 0;
  RFHandle.NatureWindFlag = 0;
  RFHandle.PowerDelayFlag = 0;
  RFHandle.DirectionCntr = 0;
  RFHandle.LedFlag = 0;
}
