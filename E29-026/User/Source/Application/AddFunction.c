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

/* Private variables ---------------------------------------------------------*/
FaultStateType             mcFaultSource;
PWMINPUTCAL        xdata   mcPwmInput;
FaultVarible       xdata   mcFaultDect;
MotorRSDTypeDef    idata   RSDDetect;
CurrentVarible     idata   mcCurVarible;
ProtectVarible     idata   mcProtectTime;

OUTLOOP            xdata   SpeedPICtrl;
FOCCTRL            xdata   mcFocCtrl;
ADCSample          xdata   AdcSampleValue;
ONVarible          xdata   ONOFFTest;
MCLedDisplay       xdata   mcLedDisplay;
MCRAMP             xdata   mcSpeedRamp;
SLEEPMODE          xdata   SleepSet;
MotorFRTypeDef     xdata   mcFRState;
int16              xdata   VSP;

float brakeduty = Brakeduty_F;
uint16  POWER_VSP;
/*---------------------------------------------------------------------------*/
/* Name     :   void OutLoopParameterSet(void)
/* Input    :   NO
/* Output   :   NO
/* Description: 
/*---------------------------------------------------------------------------*/
void OutLoopParameterSet(void)
{
  memset(&SpeedPICtrl,0, sizeof(OUTLOOP));                                // SpeedControl clear
  #if (Motor_Speed_Control_Mode == SPEED_LOOP_CONTROL)
  {              
    mcSpeedRamp.IncValue    = SPEEDRAMPSTARTINC;
    mcSpeedRamp.DecValue    = SPEEDRAMPSTARTDEC;
  }
  #elif (Motor_Speed_Control_Mode == POWER_LOOP_CONTROL)
  {
    mcSpeedRamp.IncValue    = POWRAMPSTARTINC;
    mcSpeedRamp.DecValue    = POWRAMPSTARTDEC;
  }
  #endif

//  SpeedPICtrl.ExtKP       = SKP;
//  SpeedPICtrl.ExtKI       = SKI;
//  SpeedPICtrl.ExtOutMax   = SOUTMAX;
//  SpeedPICtrl.ExtOutMin   = SOUTMIN;
//  
  PI_KP     = SKP;
  PI_KI     = SKI;
  PI_UKMAX   = SOUTMAX;
  PI_UKMIN   = SOUTMIN;
  
  mcFocCtrl.mcIqref = FOC_IQREF;
  PI_UK = mcFocCtrl.mcIqref;
//  PI_UK      = I_Value(0.7);
}
/*---------------------------------------------------------------------------*/
/* Name    :  int16 KLPF_VALUE(int16 INVlaue, int16 OutLastValue)
/* Input  :  INVlaue��OutLastValue
/* Output  :  int16�ı���
/* Description:  �˲�����,�ó˷�������
/*---------------------------------------------------------------------------*/
int16 KLPF_VALUE(int16 INVlaue, int16 OutLastValue)
{
  int16 Result = 0;
  MDU_MA = (INVlaue-OutLastValue);
  MDU_MB = (int16)480;                     /*д�������ͳ���*/

  Result = MDU_MB;
  Result += OutLastValue;
  return(Result);
}

/*---------------------------------------------------------------------------*/
/* Name    :  void FaultProcess(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �������������ر�FOC�����ͬʱ��״̬��ΪmcFault
/*---------------------------------------------------------------------------*/
void FaultProcess(void)
{
  MOE     = 0;
  ClrBit(DRV_CR, FOCEN);  //�ر�FOC
  mcState = mcFault;                
  HW1 = HWOFF;
  HW2 = HWOFF;
}

/*---------------------------------------------------------------------------*/
/* Name    :  int16 Abs_F16(int16 value)
/* Input  :  value
/* Output  :  int16
/* Description:  �Ա���ȡ16λ�ľ���ֵ
/*---------------------------------------------------------------------------*/
uint16 Abs_F16(int16 value)
{
  if(value < 0)
  {
    return (- value);
  }
  else
  {
    return (value);
  }
}
/*---------------------------------------------------------------------------*/
/* Name    :  int32 Abs_F32(int32 value)
/* Input  :  value
/* Output  :  int16
/* Description:  �Ա���ȡ16λ�ľ���ֵ
/*---------------------------------------------------------------------------*/
uint32 Abs_F32(int32 value)
{
  if(value < 0)
  {
    return (- value);
  }
  else
  {
    return (value);
  }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void APP_DIV(void)
/* Input  :  void
/* Output  :  void
/* Description:  �������õ������ĵط�������ͬһ���жϣ��Ա����жϴ���
/*---------------------------------------------------------------------------*/
void APP_DIV(void)
{
    if( mcPwmInput.PWMDivFlag==1)  //���������������������ֵ�еĳ�����ͻ
    {
       mcPwmInput.PWMDuty = MDU_DIV_IDATA_U32(&mcPwmInput.pwm.PWMCompareAMP, &mcPwmInput.PWMARRUpdate);
       mcPwmInput.PWMDivFlag=0;
    }
    if( mcFocCtrl.ESDIVFlag==1)  //���������������������ֵ�еĳ�����ͻ
    {
       mcFocCtrl.SQUSpeedDIVEs = MDU_DIV_XDATA_U32(&mcFocCtrl.SQUSysSpeed,&mcFocCtrl.EsValue);
       mcFocCtrl.ESDIVFlag=0;
    }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void PWMInputCapture(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  ����PWM����
/*---------------------------------------------------------------------------*/
void PWMInputCapture(void)
{
  uint16 MotorControlVSP;

   if(mcPwmInput.PWMUpdateFlag==1)  // ���µ�duty����
   {
      if((Abs_F32(mcPwmInput.PWMCompare-mcPwmInput.PWMCompareOld)<50)// ���αȽ�ֵ��������ٶ�����
        &&(Abs_F32(mcPwmInput.PWMARROld-mcPwmInput.PWMARR)<50)// ��������ֵ��������ٶ�����
//        &&((100<mcPwmInput.PWMARR)&&(mcPwmInput.PWMARR<6000))// ����ֵ��һ����Χ�ڲ���Ϊ��Ч����һ��Ƶ�ʷ�Χ
        &&(mcPwmInput.PWMDivFlag==0))
        {
          mcPwmInput.PWMFlag=1;                               // PWMFlag��1�ڼ䣬����ȡTIM3__DR��TIM3__ARR;����ֹ����
          mcPwmInput.pwm.PWMCompareUpdate[0]=(mcPwmInput.PWMCompare>>1);// �������32768
          mcPwmInput.PWMARRUpdate=mcPwmInput.PWMARR;
          mcPwmInput.PWMDivFlag=1;                            // ��������
          mcPwmInput.PWMFlag=0;
        }
       if(mcPwmInput.PWMDivFlag==0)
        {
          if(mcPwmInput.PWMcnt<3)//2����ƽ��ֵ
          {
            mcPwmInput.PWMcnt++;
            mcPwmInput.PWMVSum +=mcPwmInput.PWMDuty;
          }
          else
          {
            MotorControlVSP= (mcPwmInput.PWMVSum >>1);//ע������һ��������ǰ��ıȽ�ֵ���ۻ��й�
            mcPwmInput.PWMVSum=0;
            mcPwmInput.PWMcnt =0;
          }
           MotorControlVSP=mcPwmInput.PWMDuty;
         if((MotorControlVSP > ONPWMDuty)&&(MotorControlVSP <=(OFFPWMDutyHigh+1)))
          {
            mcSpeedRamp.FlagONOFF = 1;
          }
         else if(MotorControlVSP < OFFPWMDuty)//||(MotorControlVSP >= OFFPWMDutyHigh))
          {
            mcSpeedRamp.FlagONOFF = 0;
          }

          //ת�����߼���
          if(mcSpeedRamp.FlagONOFF==1)
          {
            if(MotorControlVSP <= MINPWMDuty)
            {
              mcSpeedRamp.TargetValue = Motor_Min_Speed;
            }
            else if(MotorControlVSP < MAXPWMDuty)
            {
              mcSpeedRamp.TargetValue = Motor_Min_Speed + SPEED_K*(MotorControlVSP-MINPWMDuty);
            }
            else
            {
              mcSpeedRamp.TargetValue  =  Motor_Max_Speed;
            }
          }
          else
          {
            mcSpeedRamp.TargetValue =0;
          }
        }

      mcPwmInput.PWMUpdateFlag =0;
      mcPwmInput.PWMCompareOld=mcPwmInput.PWMCompare;//���˴αȽ�ֵ��ֵ���ϴαȽ�ֵ
      mcPwmInput.PWMARROld=mcPwmInput.PWMARR;//���˴�����ֵ��ֵ���ϴ�����ֵ
    }
}
/*****************************************************************************
 * Function:     void  Fault_OverVoltage(mcFaultVarible *h_Fault)
 * Description:   ��ѹǷѹ��������������ÿ5ms�ж�һ�Σ�ĸ�ߵ�ѹ���ڹ�ѹ����ֵʱ����������һ��������ֵ����20�Σ��ж�Ϊ��ѹ�������ر����;��֮��������������
                 ͬ��Ƿѹ������
                 �����Ƿѹ����״̬�£�ĸ�ߵ�ѹ�ָ���Ƿѹ�ָ�ֵ���ϣ���ѹ�ָ�ֵ����ʱ����������һ������200�κ󣬻ָ������ݵ�λ��Ϣ�������ָ����ĸ�״̬��
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
void Fault_OverUnderVoltage(FaultVarible *h_Fault)
{
  //��ѹ����
    if(mcFaultSource == FaultNoSource)//����������������
    {
        if(mcFocCtrl.mcDcbusFlt > OVER_PROTECT_VALUE)   //ĸ�ߵ�ѹ���ڹ�ѹ����ֵʱ������������20�Σ��ж�Ϊ��ѹ�������ر����;��֮��������������
        {
          h_Fault->OverVoltDetecCnt++;
          if(h_Fault->OverVoltDetecCnt > 20)//���100ms
          {
            h_Fault->OverVoltDetecCnt = 0;
            mcFaultSource=FaultOverVoltage;
            FaultProcess();
          }
        }
        else
        {
          if(h_Fault->OverVoltDetecCnt>0)
          {
            h_Fault->OverVoltDetecCnt--;
          }
        }

      //Ƿѹ����
        if((mcFocCtrl.mcDcbusFlt< UNDER_PROTECT_VALUE)&&(mcFaultDect.VoltDetecExternCnt < 30))
        {
          h_Fault->UnderVoltDetecCnt++;

          if(h_Fault->UnderVoltDetecCnt > 20)//���100ms
          {
            h_Fault->UnderVoltDetecCnt = 0;
            h_Fault->VoltDetecBraketCount = 150;
            mcFaultSource=FaultUnderVoltage;
            FaultProcess();
            
            if(KS.FlashWriteOnetimes == 0)
            {
              Rom.WriteValue = (KS.KeyValuetotal&0x7f);//ȥ��һ�������Ϣ
              Rom.ReadValue = Get1ByteFromFlash(STARTPAGEROMADDRESS);
              if(Rom.ReadValue != Rom.WriteValue)
              {
                Write1Byte2Flash(STARTPAGEROMADDRESS,Rom.WriteValue);
              }
              KS.FlashWriteOnetimes = 1;
            }
          }
        }
        else
        {
          if(h_Fault->UnderVoltDetecCnt>0)
          {
            h_Fault->UnderVoltDetecCnt--;
          }
        }
    }
    if(((mcFaultSource==FaultOverVoltage)||(mcFaultSource==FaultUnderVoltage))&&(h_Fault->VoltDetecBraketCount >0))
    {  
      if(h_Fault->VoltDetecBraketCount > 0)
      {h_Fault->VoltDetecBraketCount--;}
      
      if((h_Fault->VoltDetecBraketCount >= 20)&&(h_Fault->VoltDetecBraketCount <= 110))
      {
          DRV_DR  = DRV_ARR*brakeduty;
          DRV_CMR &= 0xFFC0;
          DRV_CMR |= 0x015;                                                // �������ű�ͨ��ɲ��
          ClrBit(DRV_CR, OCS);                                             // OCS = 0, DRV_COMR;OCS = 1, FOC/SVPWM/SPWM
          MOE = 1;
        
          if(brakeduty < 1.0)
          brakeduty += 0.02;
      }
      else if((h_Fault->VoltDetecBraketCount <= 10))
      {
        MOE = 0;
        DRV_OUT = 0x00;
        FaultProcess();
        D1 = LEDOFF;
        D2 = LEDOFF;
        D3 = LEDOFF;
        D4 = LEDOFF;                               
        D5 = LEDOFF;
        D6 = LEDOFF;      
      }
    }

    /*******��ѹǷѹ�����ָ�*********/
    if((mcState == mcFault) &&((mcFaultSource==FaultUnderVoltage)||(mcFaultSource==FaultOverVoltage))
       &&(h_Fault->VoltDetecBraketCount == 0))
    {
      if((mcFocCtrl.mcDcbusFlt< OVER_RECOVER_VALUE)&&(mcFocCtrl.mcDcbusFlt> UNDER_RECOVER_VALUE))
      {
        h_Fault->VoltRecoverCnt++;
        if(h_Fault->VoltRecoverCnt>100)//�������200ms����������ָ�
        {
          mcState = mcReady;
          mcFaultSource=FaultNoSource;
          h_Fault->VoltRecoverCnt = 0;
        }
      }
      else
      {
        h_Fault->VoltRecoverCnt = 0;
      }
   }
}


/*****************************************************************************
 * Function:     void  Fault_Power(mcFaultVarible *h_Fault)
 * Description:   ���ʱ�������
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
void Fault_Power(FaultVarible *h_Fault)
{

    if(mcFaultSource == FaultNoSource)//����������������
    {
        if(mcFocCtrl.Powerlpf > PowerLimit)   //���ʴ��ڱ���ֵʱ����������20�Σ��ж�Ϊ���ر������ر����;��֮��������������
        {
          h_Fault->OverPowerDetecCnt++;
          if(h_Fault->OverPowerDetecCnt > 20)
          {
            h_Fault->OverPowerDetecCnt = 0;
            mcFaultSource=FaultOverPower;
            FaultProcess();
          }
        }
        else
        {
          if(h_Fault->OverPowerDetecCnt>0)
          {
            h_Fault->OverPowerDetecCnt--;
          }
        }
    }

  }
  
void Fault_Temperature(FaultVarible *h_Fault)
{
  /*******����˿�¶ȱ���********/
  if(mcFaultSource == FaultNoSource)//����������������
  {
    if(User.Temperature < TemperatureProtectvalue)
    {
      h_Fault->Temperaturecnt++;
      if(h_Fault->Temperaturecnt >= 200)  //1S
      {
        h_Fault->Temperaturecnt = 0;
        mcFaultSource=FaultNtcTemperature;
        FaultProcess();
      } 
    }
    else
    {
      if(h_Fault->Temperaturecnt > 0)
      {h_Fault->Temperaturecnt--;}
    }
    
  }
  
  //����˿�¶ȱ����ָ�
   if((mcFaultSource==FaultNtcTemperature)&&(mcState == mcFault))
   {
      if(User.Temperature > TemperatureRecovervalue)
      {
        mcFaultDect.TemperatureRecCount++;
        if(mcFaultDect.TemperatureRecCount >= 200)
        {
          mcFaultDect.TemperatureRecCount = 0;
          mcFaultSource=FaultNoSource;
          mcState = mcReady;
        }
      }
      else
      {
        if(mcFaultDect.TemperatureRecCount > 0)
        {mcFaultDect.TemperatureRecCount--;}
      }   
   }
   
   /**���ư��¶ȱ���**/
  if(mcFocCtrl.mcADCTemperature < OVER_Temperature)
  {
    h_Fault->TemperCnt++;
    if(h_Fault->TemperCnt > 500)
    {  
      h_Fault->TemperCnt = 0;
      h_Fault->TemperRecover = 0;
      mcFaultSource = FaultOverTemperature;
      FaultProcess();
    }
  }
  else
  {
    h_Fault->TemperCnt = 0;
  }
  /***���ư��¶ȱ����ָ�**/
  if((mcState == mcFault)&&(mcFaultSource == FaultOverTemperature))
  {              
     if(h_Fault->TemperRecover < OverTemperRecoverTime)                
     {
       h_Fault->TemperRecover++; 
     }
     else
     {
      if(mcFocCtrl.mcADCTemperature > UNDER_Temperature)  
      {  
        h_Fault->TemperRecover = 0;          
        mcState = mcReady;
        mcFaultSource=FaultNoSource;
      }         
     }     
   }
}  

/*****************************************************************************
 * Function:     void Fault_Overcurrent(CurrentVarible *h_Cur)
 * Description:   ������л�������ʱ����������ĳһ�����ֵ����OverCurrentValue����OverCurCnt��1��
                 �����ۼ�3�Σ��ж�Ϊ�������������ִ��ʱ��Լ30.4us��
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
void Fault_Overcurrent(CurrentVarible *h_Cur)
{
  if((mcState == mcRun)||(mcState == mcStart))            // check over current in rum and open mode
  {
    h_Cur->Abs_ia = Abs_F16(FOC__IA);
    h_Cur->Abs_ib = Abs_F16(FOC__IB);
    h_Cur->Abs_ic = Abs_F16(FOC__IC);
    if(h_Cur->Abs_ia> h_Cur->Max_ia)                      // �˲��ּ������������������������ȱ�ౣ��
    {
       h_Cur->Max_ia = h_Cur->Abs_ia;
    }
    if(h_Cur->Abs_ib > h_Cur->Max_ib)
    {
       h_Cur->Max_ib = h_Cur->Abs_ib;
    }
    if(h_Cur->Abs_ic > h_Cur->Max_ic)
    {
       h_Cur->Max_ic = h_Cur->Abs_ic;
    }

    if((h_Cur->Max_ia>=OverSoftCurrentValue)||(h_Cur->Max_ib>=OverSoftCurrentValue)||(h_Cur->Max_ic>=OverSoftCurrentValue))
    {
      h_Cur->OverCurCnt++;
      if(h_Cur->OverCurCnt>=3)
      {
        h_Cur->Max_ia=0;
        h_Cur->Max_ib=0;
        h_Cur->Max_ic=0;
        h_Cur->OverCurCnt=0;
        mcFaultSource=FaultSoftOVCurrent;
        FaultProcess();
      }
    }
    else
    {
      if(h_Cur->OverCurCnt>0)
      {
        h_Cur->OverCurCnt--;
      }
    }
  }
}

/*****************************************************************************
 * Function:     void  Fault_OverCurrentRecover(mcFaultVarible *h_Fault)
 * Description:   ��Ӳ�����������ָ�
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
void Fault_OverCurrentRecover(FaultVarible *h_Fault)
{
  if((mcState == mcFault)&&((mcFaultSource==FaultSoftOVCurrent)||(mcFaultSource==FaultHardOVCurrent))&&(mcProtectTime.CurrentPretectTimes<5))
  {
    h_Fault->CurrentRecoverCnt++;
    if(h_Fault->CurrentRecoverCnt>=OverCurrentRecoverTime)//1000*5=5s
    {
      h_Fault->CurrentRecoverCnt=0;
      mcProtectTime.CurrentPretectTimes++;
      mcState = mcReady;
      mcFaultSource=FaultNoSource;
    }
  }
}

/*****************************************************************************
 * Function:     void  Fault_OverPowerRecover(mcFaultVarible *h_Fault)
 * Description:   ���ʱ����ָ�����
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
void Fault_OverPowerRecover(FaultVarible *h_Fault)
{
  if((mcState == mcFault)&&(mcFaultSource==FaultOverPower)&&(mcProtectTime.PowerPretectTimes<5))
  {
    h_Fault->OverPowerDetecCnt++;
    if(h_Fault->OverPowerDetecCnt>=OverPowerRecoverTime)
    {
      h_Fault->OverPowerDetecCnt=0;
      mcProtectTime.PowerPretectTimes++;
      mcState = mcReady;
      mcFaultSource=FaultNoSource;
    }
  }
}

/*****************************************************************************
 * Function:     void  Fault_Start(mcFaultVarible *h_Fault)
 * Description:   ���������������������״̬�£������ǰ5s����ת�ٴﵽ��ת����ֵ����5s�󷴵綯��ֵ̫��(�˷���δ��֤)
                  ��4s�ڻ���CtrlMode״̬�����ٶȵ���MOTOR_LOOP_RPM�������ж�Ϊ����ʧ�ܣ����ͣ����
                  �������ж�Ϊ����ʧ�ܺ��������������ڻ����5�Σ�������������У׼״̬���ȴ�������
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
  void Fault_Start(FaultVarible *h_Fault)
  {
    /*******���������ָ�*********/
    if(mcState == mcRun)
    {
      //����һ��5s���ٶȴ�������ٶȣ�ͬʱ���綯��ֵ����һ��ֵ
//      if(h_Fault->StartSpeedCnt<=1000)
//      {
//        h_Fault->StartSpeedCnt++;
//        if((mcFocCtrl.SpeedFlt > Motor_Max_Speed)&&(mcFocCtrl.EsValue<20))
//        {
//          h_Fault->StartSpeedCnt = 0;
//          mcFaultSource=FaultStart;
//          FaultProcess();
//          mcProtectTime.SecondStartTimes++;
//          mcProtectTime.StartFlag  =  1;
//        }
//      }
     //������
      if(h_Fault->StartEsCnt<=1200)//ǰ6s���ȴ�1s�󣬿�ʼ�ж�ES���������һ����������ʧ��
      {
        h_Fault->StartEsCnt++;
        h_Fault->StartDelay++;  
        if(h_Fault->StartDelay>=75)        // 0.15S
        {
           h_Fault->StartDelay=75;
           if((mcFocCtrl.EsValue <100))//&&(mcFocCtrl.CtrlMode==0))
            {
              h_Fault->StartESCount++;
              if(h_Fault->StartESCount>=15)
              {
                mcFaultSource=FaultStart;
                FaultProcess();
                mcProtectTime.SecondStartTimes++;
                h_Fault->StartDelay=0;
                h_Fault->StartESCount=0;
                mcProtectTime.StartFlag  =  2;
              }
            }
            else
            {
              if(h_Fault->StartESCount>0)
                h_Fault->StartESCount--;
            }
       }
     }
     else
     {
       h_Fault->StartESCount=0;
     }
       //����������ʱ����CtrlMode=0״̬
      if(mcFocCtrl.CtrlMode==0)         //
      {
        h_Fault->StartFocmode++;
        if(h_Fault->StartFocmode>=40)
        {
          h_Fault->StartFocmode=0;
          mcFaultSource=FaultStart;
          FaultProcess();
          mcProtectTime.SecondStartTimes++;
          mcProtectTime.StartFlag  =  3;
        }
      }
    }
    #if (!StartONOFF_Enable)
    {
     if((mcFaultSource==FaultStart)&&(mcState == mcFault)&&(mcProtectTime.SecondStartTimes<=StartProtectRestartTimes))
     {
       mcFaultSource=FaultNoSource;
       mcState = mcReady;
     }
    }
   #endif
  }
 
 /*****************************************************************************
 * Function:     void  Fault_Stall(mcFaultVarible *h_Fault)
 * Description:   ��ת���������������ֱ�����ʽ��
                 ��һ�֣�
                 �ڶ��֣��������״̬�£��ӳ�4s�жϣ������ٶȾ���ֵ������ת�ٶ�����5�Σ�
                 �����֣��������״̬�£���U,V�����������ֵ���ڶ�ת��������ֵ����6�Σ�
                 ���������ֵ��κ�һ�ֱ�������ʱ�����ͣ���������ж�Ϊ��ת������
                 ����ת����״̬�£�U��ɼ�ֵ���ڶ�ת�ָ�ֵʱ������ת����С�ڻ���ڶ�ת��������8�Σ�
                 �����ӳ�mcStallRecover��������������У׼״̬��
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
//��ת����
void Fault_Stall(FaultVarible *h_Fault,CurrentVarible *h_Cur)
{
//  h_Fault->mcEsValue = FOC__ESQU;
  if(mcState == mcRun)
  {
    if(h_Fault->StallDelayCnt <=1000)// ��������ʱ5s�ж��Ƿ��ת����
    {
      h_Fault->StallDelayCnt ++;
    }
    else
    {
      //method 1�������綯��̫С 
      if(mcFocCtrl.EsValue< 70)
      {
        h_Fault->StallDectEs++;
        if(h_Fault->StallDectEs >= 30)   //�ж����������ﵽ���ô�������������
        {
          h_Fault->StallDectEs=0;
          mcFaultSource=FaultStall;
          mcProtectTime.StallTimes++;        //��ת����+1
          FaultProcess();
          h_Fault->StallDelayCnt = 0;        //������������ʼ���¼���������5S�ӳ�
          mcProtectTime.StallFlag  =  1;
        }
      }
      else
      {
        if(  h_Fault->StallDectEs>0)
          h_Fault->StallDectEs--;
      }
      
      //method 2���ж��ٶȵ��ڶ�ת��Сֵ���߳�����ת���ֵ
      if((mcFocCtrl.SpeedFlt<Motor_Stall_Min_Speed)||(mcFocCtrl.SpeedFlt > Motor_Stall_Max_Speed))
      {
        h_Fault->StallDectSpeed++;
        if(h_Fault->StallDectSpeed >= 100)   //�ж����������ﵽ���ô�������������
        {
          h_Fault->StallDectSpeed=0;
          mcFaultSource=FaultStall;
          mcProtectTime.StallTimes++;        //��ת����+1
          FaultProcess();
//          h_Fault->StallDelayCnt = 0;       //������������ʼ���¼���������5S�ӳ�
           mcProtectTime.StallFlag =2;
        }
      }
      else
      {
        if(h_Fault->StallDectSpeed>0)
        h_Fault->StallDectSpeed--;
      }
      
      //method 3      A/B/C��������һ����������ڶ�ת��������ת����
//    if((h_Cur->Max_ia >= StallCurrentValue)||
//       (h_Cur->Max_ib >= StallCurrentValue)||
//       (h_Cur->Max_ic >= StallCurrentValue))
//    {       
//      h_Fault->mcStallDeCurrent++;              
//      if(h_Fault->mcStallDeCurrent >= 20)          //�ж����������ﵽ���ô�������������
//      {
//        h_Fault->mcStallDeCurrent=0;
//        mcFaultSource=FaultStall;  
//        mcProtectTime.StallTimes++;
//        FaultProcess();    
//        h_Fault->StallDelayCnt = 0;       //������������ʼ���¼���������5S�ӳ�
//        mcProtectTime.StallFlag  =  3;
//      }             
//    }  
//    else
//      {
//        if(h_Fault->mcStallDeCurrent>0)
//        h_Fault->mcStallDeCurrent--;
//      }
  }
}

 #if (!StartONOFF_Enable)
{
     /*******��ת�����ָ�*********/
//        if((mcFaultSource==FaultStall)&&(mcState == mcFault)&&(mcProtectTime.StallTimes<=4))  //��ת��������
//        {
//          h_Fault->StallReCount++;
//          if(h_Fault->StallReCount>=StallRecoverTime)
//          {
//            h_Fault->StallReCount=0;        //16000
//            mcFaultSource=FaultNoSource;
//            mcState =   mcReady;
//          }
//        }
//        else
//        {
//          h_Fault->StallReCount=0;
//        }
}
#endif
}
    
 /*****************************************************************************
 * Function:     void  Fault_phaseloss(mcFaultVarible *h_Fault)
 * Description:   ȱ�ౣ�����������������״̬�£�10msȡ������������ֵ��
                 1.5s�жϸ���������ֵ���������������ֵ����һ��ֵ�������������ֵȴ�ǳ�С�����ж�Ϊȱ�ౣ�������ͣ����
 * Parameter:     mcFaultVarible *h_Fault
 * Return:       no
 *****************************************************************************/
void Fault_phaseloss(FaultVarible *h_Fault)
{
  if(mcState == mcRun)
  {
    h_Fault->Lphasecnt++;
    if(h_Fault->Lphasecnt>8)
    {
       h_Fault->Lphasecnt=0;
       if(((mcCurVarible.Max_ia>(mcCurVarible.Max_ib*2))||(mcCurVarible.Max_ia>(mcCurVarible.Max_ic*2)))&&(mcCurVarible.Max_ia>PhaseLossCurrentValue))
       {
          h_Fault->AOpencnt++;
       }
       else
       {
        if(h_Fault->AOpencnt>0)
          h_Fault->AOpencnt --;
       }
       if(((mcCurVarible.Max_ib >(mcCurVarible.Max_ia*2))||(mcCurVarible.Max_ib >(mcCurVarible.Max_ic*2)))&&(mcCurVarible.Max_ib >PhaseLossCurrentValue))
       {
         h_Fault->BOpencnt++;
       }
      else
       {
         if(h_Fault->BOpencnt>0)
          h_Fault->BOpencnt --;
       }
       if(((mcCurVarible.Max_ic >(mcCurVarible.Max_ia*3))||(mcCurVarible.Max_ic >(mcCurVarible.Max_ib*3)))&&(mcCurVarible.Max_ic >PhaseLossCurrentValue))
       {
         h_Fault->COpencnt++;
       }
      else
       {
         if(h_Fault->COpencnt>0)
          h_Fault->COpencnt --;
       }
        mcCurVarible.Max_ia = 0;
        mcCurVarible.Max_ib = 0;
        mcCurVarible.Max_ic = 0;
      if(h_Fault->AOpencnt > 10|| h_Fault->BOpencnt > 10 || h_Fault->COpencnt > 10)
       {
          mcProtectTime.LossPHTimes++;
          mcFaultSource=FaultLossPhase;
          FaultProcess();
       }
    }
  }

  /*******ȱ�ౣ���ָ�*********/
  #if (!StartONOFF_Enable)           //��ͣ����ʱ����ȱ�ౣ���ָ�
  {
    if((mcFaultSource==FaultLossPhase)&&(mcState == mcFault)&&(mcProtectTime.LossPHTimes<5))//������5��
    {
      h_Fault->mcLossPHRecCount++;
      if(h_Fault->mcLossPHRecCount>=PhaseLossRecoverTime)
      {
        h_Fault->AOpencnt=0;
        h_Fault->BOpencnt=0;
        h_Fault->COpencnt=0;
        mcState = mcReady;
        mcFaultSource=FaultNoSource;
      }
    }
    else
    {
      h_Fault->mcLossPHRecCount=0;
    }
  }
  #endif
}

/*---------------------------------------------------------------------------*/
/* Name    :  void Fault_Detection(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �����������򱣻���ʱ����Ӧ����ܸߣ����÷ֶδ���ÿ5����ʱ���ж�ִ��һ�ζ�Ӧ�ı���
                ���������й�Ƿѹ�����¡���ת��������ȱ��ȱ���������ʱ���ɸ�������һ�����ĵ��Լ��롣
/*---------------------------------------------------------------------------*/
void Fault_Detection(void)
{
  mcFaultDect.segment++;
  if(mcFaultDect.segment>=5)
  {
    mcFaultDect.segment=0;
  }
  if(mcFaultDect.segment==0)
  {
    if(CurrentRecoverEnable)//���������ָ�ʹ��
    {
      Fault_OverCurrentRecover(&mcFaultDect);
    }
    if(PowerRecoverEnable) //���ʱ����ָ�ʹ��
    {
      Fault_OverPowerRecover(&mcFaultDect);
    }
    
    if(TemperatureProtectEnable) //�¶ȱ���
    {
      Fault_Temperature(&mcFaultDect);
    }    
  }
  else if(mcFaultDect.segment==1)
  {
    if(VoltageProtectEnable==1)//��ѹ����ʹ��
    {
      Fault_OverUnderVoltage(&mcFaultDect);
    }
    
    if(OverPowerProtectEnable==1); //���ʱ���ʹ��
    {
      Fault_Power(&mcFaultDect);
    }        
  }
  else if(mcFaultDect.segment==2)
  {
    if(StartProtectEnable==1)//��������ʹ��
    {
      Fault_Start(&mcFaultDect);
    }
  }
  else if(mcFaultDect.segment==3)
  {
    if(StallProtectEnable==1)//��ת����ʹ��
    {
      Fault_Stall(&mcFaultDect,&mcCurVarible);
    }
  }
  else if(mcFaultDect.segment==4)
  {
    if(PhaseLossProtectEnable==1)//ȱ�ౣ��ʹ��
    {
      Fault_phaseloss(&mcFaultDect);
    }
  }
  else
  {

  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void ONOFF_Starttest(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  ��������
/*---------------------------------------------------------------------------*/
void ONOFF_Starttest(ONVarible  *h_test)
{
  if(h_test->ONOFF_Flag==1)
  {
     h_test->ON_Count++;
    if(h_test->ON_Count>StartON_Time)
    {
      h_test->ON_Count=0;
      h_test->ONOFF_Times++;
      h_test->ONOFF_Flag=0;
      mcSpeedRamp.FlagONOFF = 0;
//      mcSpeedRamp.TargetValue = 0;
    }
  }
  else
  {
    if(mcState!=mcFault)
    {
      h_test->OFF_Count++;
      if(h_test->OFF_Count>StartOFF_Time)
      {
        h_test->OFF_Count=0;
        h_test->ONOFF_Flag=1;
        mcSpeedRamp.FlagONOFF = 1;
//        mcSpeedRamp.TargetValue = Motor_Test_Speed;
      }
    }
  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void Speed_response(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �ٶ���Ӧ�������ɸ������������ƻ������ת�ؿ��ơ���ת�ٿ��ơ��㹦�ʿ���
/*---------------------------------------------------------------------------*/

void Speed_response(void)
{
//    uint16  UqMinTemp = 0;     //UQ�޷���Сֵ�������
  if((mcState ==mcRun)||(mcState ==mcStop))
  {      
    switch(mcFocCtrl.CtrlMode)
    {
      case 0:
      {
          if(FOC__EOME > Motor_Loop_Speed)
          {
            mcFocCtrl.CtrlMode = 1;
            FOC_DQKP = DQKP;
            FOC_DQKI = DQKI;
//              #if (Motor_Speed_Control_Mode == SPEED_LOOP_CONTROL)
//              {
//                mcSpeedRamp.ActualValue = _Q15(75000 / MOTOR_SPEED_BASE);
//              }
            
//              #endif
              
            OutLoopParameterSet();       //��·������������
            
            mcFocCtrl.SpeedRamp = 2;
            mcFocCtrl.SpeedLoop = 5;
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
            mcFocCtrl.SpeedRampTime++;
            if(mcFocCtrl.SpeedRampTime > mcFocCtrl.SpeedRamp)
            {
                mcFocCtrl.SpeedRampTime = 0;
                mc_ramp(&mcSpeedRamp);       //�⻷�ٶ����»���١�ÿSpeedRampTimeִ��һ��
            }
            
            mcFocCtrl.SpeedLoopTime++;
            if(mcFocCtrl.SpeedLoopTime > mcFocCtrl.SpeedLoop)  //�⻷��������
            {
                mcFocCtrl.SpeedLoopTime=0;
                #if (Motor_Speed_Control_Mode == SPEED_LOOP_CONTROL)
                {
//                      SpeedPICtrl.ExtRef = mcSpeedRamp.ActualValue;
//                      SpeedPICtrl.ExtFed = mcFocCtrl.SpeedFlt;
//
//                      if(mcFocCtrl.SpeedFlt > (_Q15(80000 / MOTOR_SPEED_BASE)))
//                      {
//                        mcFocCtrl.loopchdelay++;
//                        if(mcFocCtrl.loopchdelay > 500)
//                        {
//                          mcFocCtrl.loopchdelay = 0;
//                          PI_KP    = SKPH;
//                          PI_KI    = SKIH;
////                          FOC_DQKP = DQKPH;
////                          FOC_DQKI = DQKIH;
//                          mcFocCtrl.SpeedLoop = 2;
//                          mcFocCtrl.SpeedRamp = 8;
//                        }
//
//                      }
//                      else if(mcFocCtrl.SpeedFlt < (_Q15(60000 / MOTOR_SPEED_BASE)))
//                      {
//                        PI_KP      = SKP;
//                        PI_KI      = SKI;
//                        FOC_DQKP   = DQKP;
//                        FOC_DQKI   = DQKI;
//                        SetBit(FOC_CR2,F5SEG);
//                      }
//                      
//                      HW_PI_Control(&SpeedPICtrl);
//                      FOC_IQREF = SpeedPICtrl.ExtOut;
                  }
                  #elif (Motor_Speed_Control_Mode == POWER_LOOP_CONTROL)
                  {
                    if(mcFocCtrl.SpeedFlt > (_Q15(100000 / MOTOR_SPEED_BASE)))
                    {
                        FOC_DQKP  = DQKPH;
                        FOC_DQKI  = DQKIH;
                        PI_KP     = SKPH;
                        PI_KI     = SKIH;
                    }
                    else if(mcFocCtrl.SpeedFlt < (_Q15(80000 / MOTOR_SPEED_BASE)))
                    { 
                        FOC_DQKP  = DQKP;
                        FOC_DQKI  = DQKI;
                        PI_KP     = SKP;
                        PI_KI     = SKI;
                    }
                    
                    FOC_IQREF = HW_One_PI(POWER_VSP- mcFocCtrl.mcSysPower);
                  }
                  #endif
              }
//           // �ж�FOC_QMINֵ�ָ�������ֵ
//           UqMinTemp = FOC_QMIN;
//           if(UqMinTemp > QOUTMIN)
//           {
//             UqMinTemp -- ;
//             FOC_QMIN = UqMinTemp;
//           }
           }
         #endif //END OUTLoop_Mode

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
      break;
    }
  }
}

/*---------------------------------------------------------------------------*/
/* Name       :  void FGOutput(void)
/* Input      :  NO
/* Output     :  NO
/* Description:  FG�ź����
/*---------------------------------------------------------------------------*/
void FGOutput(void)
{//FG�����ж�FGIF�����ж�
  if(mcState==mcRun)
  {
    if((FOC__THETA>=0)&&(FOC__THETA<32768))//0-180
    {
      ResetFGPin;
    }
    else if((FOC__THETA>=32768)&&(FOC__THETA<65536))//180-360
    {
      SetFGPin;
    }
  }
  else if(mcState == mcFault)
  {
      SetFGPin;
  }
  else
  {
      ResetFGPin;
  }
}
/*---------------------------------------------------------------------------*/
/* Name       :   uint16 SoftLPF(uint16 Xn1, uint16 Xn0, uint16 K)
/* Input      :  uint16 Xn1, uint16 Xn0, uint16 K
/* Output     :  uint16
/* Description:  �����ͨ�˲�
/*---------------------------------------------------------------------------*/
 int16 SoftLPF(int16 Xn1, int16 Xn0, int16 K)
 {
    int16 Temp16 = 0;
    int32 Temp32 = 0;

    Temp32 = (((int32)Xn1 - (int32)Xn0) * (int32)K) >> 15;
    Temp16 = Xn0 + (int16)Temp32;
    return Temp16;
 }

/*---------------------------------------------------------------------------*/
/* Name    :  void LED_Display(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  LED����ʾ
/*---------------------------------------------------------------------------*/
void LED_Display(void)
{
  switch(mcFaultSource)
  {
    case FaultNoSource:
    SetLEDPin;                          //�͵�ƽ�������ߵ�ƽ��
      break;
    case FaultHardOVCurrent:             //Ӳ������������˸10��           
        Led_OnOff(&mcLedDisplay,10);
      break;
    case FaultSoftOVCurrent:            //�������������˸4��           
        Led_OnOff(&mcLedDisplay,4);
      break;
    case FaultUnderVoltage:              //Ƿѹ������˸2��
        Led_OnOff(&mcLedDisplay,2);
      break;
    case FaultOverVoltage:                //��ѹ������˸3��
        Led_OnOff(&mcLedDisplay,3);
      break;
    case FaultLossPhase:                  //ȱ�ౣ����˸8��
        Led_OnOff(&mcLedDisplay,5);
      break;
    case FaultStall:                     //��ת������˸6��
        Led_OnOff(&mcLedDisplay,6);
      break;
    case  FaultOverPower:               //���ʱ�����˸7��
        Led_OnOff(&mcLedDisplay,7);
      break;

    default:
      break;
  }
}

//LED�Ƶ���˸
void Led_OnOff(MCLedDisplay *hLedDisplay,uint8 htime)
{
  hLedDisplay->LedCount++;
  if(hLedDisplay->LedCount<hLedDisplay->Counttime)
   {
      if(hLedDisplay->Count<200)
      {
       hLedDisplay->Count++;

      }
      else if((hLedDisplay->Count>=200)&&(hLedDisplay->Count<201))
      {
        hLedDisplay->Count=0;
        LEDPinONOFF;
        hLedDisplay->LedTimCot++;
      }
      else
        ;
      if(hLedDisplay->LedTimCot>=2*htime)
      {
        hLedDisplay->Count=202;
        SetLEDPin;
      }
    }
    else if(hLedDisplay->LedCount>=hLedDisplay->Counttime)
    {
     hLedDisplay->LedCount=0;
     hLedDisplay->LedTimCot=0;
     hLedDisplay->Count=0;
    }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void mc_ramp(void)
/* Input  :  hTarget,MC_RAMP *hSpeedramp
/* Output  :  NO
/* Description:
/*---------------------------------------------------------------------------*/
void mc_ramp(MCRAMP *hSpeedramp)
{
  if (hSpeedramp->ActualValue < hSpeedramp->TargetValue)
  {
      if(hSpeedramp->ActualValue + hSpeedramp->IncValue < hSpeedramp->TargetValue)
      {
          hSpeedramp->ActualValue += hSpeedramp->IncValue;
      }
      else
      {
          hSpeedramp->ActualValue = hSpeedramp->TargetValue;
      }
  }
  else
  {
      if(hSpeedramp->ActualValue - hSpeedramp->DecValue > hSpeedramp->TargetValue)
      {

          hSpeedramp->ActualValue -= hSpeedramp->DecValue;
      }
      else
      {
          hSpeedramp->ActualValue = hSpeedramp->TargetValue;
      }
  }
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  int16 HW_One_PI(int16 Xn1, int16 Yn0, int16 Xn2)
  Description   :  PI����
  Input         :  Xn1--E(K)
  Output      :  PI_UK--��ǰPI���ֵ,ִ��ʱ��us
-------------------------------------------------------------------------------------------------*/
int16 HW_One_PI(int16 Xn1)
{
     PI_EK =  Xn1;        //����EK
    PI_LPF_CR |= 0x02;    // Start PI
    _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
    PI_UK+=(SKP/4096 +1);
    return PI_UK;
}

void HW_PI_Control(OUTLOOP *PIPara)
{    
    PI_EK =  PIPara->ExtErr;                                                    //���뱾��EK ���ֵ
    SetBit(PI_LPF_CR,PISTA);                                                    // Start PI
    _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
  
    PI_UK      = PIPara->ExtOutL;
    PI_UKMAX   = SOUTMAX;
    PI_UKMIN   = SOUTMIN;
    PIPara->ExtErr = PIPara->ExtRef - PIPara->ExtFed;
    PI_EK =  PIPara->ExtErr;
  
    SetBit(PI_LPF_CR,PISTA);                                                    // Start PI
    _nop_();  _nop_();  _nop_();  _nop_();  _nop_();
    PIPara->ExtOut = PI_UK + 1;                                                  // ����PI������
    PIPara->ExtOutL= PIPara->ExtOut;
    
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  int16 LPF(int16 Xn1, int16 Xn0, int8 K)
  Description   :  LFP����  Y(k) = Y(k-1) + LPF_K*(X(k) �C Y(k-1))
  Input         :  Xn1--��ǰ����ֵ
                   Xn0--��һ���˲����ֵ
                   K--LPF�˲�ϵ��
  Output        :  LPF_Y--��ǰ�˲����ֵ��ִ��ʱ��Ϊ4us��
-------------------------------------------------------------------------------------------------*/
int16 LPFFunction(int16 Xn1, int16 Xn0, int8 K)
{
  LPF_K = K;
  LPF_X = Xn1;
  LPF_Y = Xn0;
  SetBit(PI_LPF_CR, LPFSTA);
  _nop_();_nop_();_nop_();_nop_();_nop_();
  return LPF_Y;
}

/*---------------------------------------------------------------------------*/
/* Name    :  void VSPSample(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  VSP����
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Name    :  void Sleepmode(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  ˯��ģʽ����
/*---------------------------------------------------------------------------*/
void Sleepmode(void)
{
   SleepSet.SleepDelayCout++;
  if(SleepSet.SleepDelayCout>=20000)//���65530����Ҫ�ٴ������������
  {
//    FOC_EFREQMIN  = -Motor_Omega_Ramp_Min;
//    FOC_EFREQHOLD = -Motor_Omega_Ramp_End;
    mcSpeedRamp.TargetValue = 0;
    MOE = 0;
    ClrBit(DRV_CR, FOCEN);  //�ر�FOC
    SleepSet.SleepDelayCout = 0;
    SleepSet.SleepFlag = 1;
    SetBit(P1_IE, P11);   // config P11 as the source of EXTI1
    SetBit(PCON, STOP);
  }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void StarRampDealwith(void)
/* Input  :  NO
/* Output  :  NO
/* Description:
/*---------------------------------------------------------------------------*/
void StarRampDealwith(void)
{
  if((mcState == mcRun))
  {
    if(mcFocCtrl.State_Count == 300)//2300
    {
      FOC_EKP = OBSW_KP_GAIN_RUN;                            // ���������PI��KP
      FOC_EKI = OBSW_KI_GAIN_RUN;                            // ���������PI��KI
    }
    else if(mcFocCtrl.State_Count == 200)//2000
    {
      FOC_EKP = OBSW_KP_GAIN_RUN1;                          // ���������PI��KP
      FOC_EKI = OBSW_KI_GAIN_RUN1;                          // ���������PI��KI
    }
    else if(mcFocCtrl.State_Count == 150)//1600
    {
      FOC_EKP = OBSW_KP_GAIN_RUN2;                          // ���������PI��KP
      FOC_EKI = OBSW_KI_GAIN_RUN2;                          // ���������PI��KI
    }
    else if(mcFocCtrl.State_Count == 100)//1200
    {
      FOC_EKP = OBSW_KP_GAIN_RUN3;                          // ���������PI��KP
      FOC_EKI = OBSW_KI_GAIN_RUN3;                          // ���������PI��KI
    }
    else if(mcFocCtrl.State_Count == 50)
    {
      FOC_EKP = OBSW_KP_GAIN_RUN4;                          // ���������PI��KP
      FOC_EKI = OBSW_KI_GAIN_RUN4;                          // ���������PI��KI
    }
    else;
  }
}
