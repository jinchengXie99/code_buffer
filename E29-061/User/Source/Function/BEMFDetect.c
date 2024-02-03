/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : BEMFDetect.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 10-Apr-2017
* Description        : This file contains BEMF detection used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>

/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/

/* Private function prototypes ------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------*/

BEMFDetect_TypeDef xdata BEMFDetect;
extern uint16  POWER_VSP;
/*---------------------------------------------------------------------------*/
/* Name    :  void BEMFDetectInit(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  BEMF�ĳ�ʼ��������������ʼ����ʹ�ܱȽ���
/*---------------------------------------------------------------------------*/
void BEMFDetectInit(void)
{
  
    //BEMF���ǰ�ر�mos���
    BEMFDetect.BEMFSpeed =0;
    BEMFDetect.BEMFSpeedBase =0;
    BEMFDetect.BEMFStatus = 0;
    BEMFDetect.FRStatus      = mcFRState.TargetFR;
    BEMFDetect.BEMFTimeCount = BEMF_START_DETECT_TIME;//��ʼ���綯�Ƽ��ʱ��

    BEMFDetect.BEMFSpeedInitStatus =0;
    BEMFDetect.FlagSpeedCal =0;
    BEMFDetect.BEMFStartStatus =0;

    //ʹ�ܶ�ʱ��2���ڼ��ʱ��
    Time2_BMEF_Init();
    //ʹ�ܱȽ���
    CMP_BMEF_Init();

}
/*---------------------------------------------------------------------------*/
/* Name    :  void CMP_BMEF_Init(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  BMF��ӦTime2�ĳ�ʼ��
/*---------------------------------------------------------------------------*/
void CMP_BMEF_Init(void)
{
  /*-------------------------------------------------------------------------------------------------
  CMP Input Pin Mode
  0: GPIO Mode, P1.4--CMP0_IN+, P1.6--CMP1_IN+, P2.1--CMP2_IN+
                P1.5--CMP0_IN-, P1.7--CMP1_IN-, P2.2--CMP2_IN-
  1: BEMF Mode, �Ƚ����������ӵ��ڲ��������ӵ���U��V��W��BMEF�����㣬
                �Ƚ����������ӵ��ڲ��������ӵ�����������Ե�
                �Ƚ���������P1.5/P1.7/P2.2�Ͽ���������GPIO����������;
-------------------------------------------------------------------------------------------------*/
  SetBit(P1_AN, P14 | P16);                                   // CMP0��CMP1
  SetBit(P2_AN, P21);                                         // CMP2
/*-------------------------------------------------------------------------------------------------
   CMP0_MOD��
   00��  �������������ĵ�����BEMFģʽ
   01��  �����������ĵ�����BEMFģʽ
   10��  3��ֱȽ���ģʽ
   11��  2�Ƚ���ģʽ
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR2, CMP0MOD0 | CMP0MOD1, CMP0MOD0);
/*-------------------------------------------------------------------------------------------------
  �Ƚ������ѡ�����ã���CMP0_MOD���ʹ��
  CMP0_SEL[1:0]=00���Ƚ���0������3�Ƚ�����ѯģʽ��������CMP0P��CMP1P��CMP2P֮���Զ�����ѡ��
                  ���˹̶�������BEMF��������ĵ㣬���������ֱ�����CMP0_OUT��CMP1_OUT��CMP2_OUT
  CMP0_SEL[1:0]=01���Ƚ���0ѡ��CMP0��Ӧ�Ķ˿���ϣ����˽�CMP0P�����˽�����BEMF��������ĵ㣬�����CMP0_OUT
  CMP0_SEL[1:0]=10���Ƚ���0ѡ��CMP1��Ӧ�Ķ˿���ϣ����˽�CMP1P�����˽�����BEMF��������ĵ㣬�����CMP1_OUT
  CMP0_SEL[1:0]=11���Ƚ���0ѡ��CMP2��Ӧ�Ķ˿���ϣ����˽�CMP2P�����˽�����BEMF��������ĵ㣬�����CMP2_OUT

-----------------------------------------------------------------------------*/
  SetReg(CMP_CR2, CMP0SEL0 | CMP0SEL1, 0x00);

/*-------------------------------------------------------------------------------------------------
  �Ƚ������͵�ѹѡ��
  000: �޳���   001: ��2.5mV   010: -5mV   011: +5mV
  100: +-5mV   101: -10mV   110: +10mV   111: +-10mV
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR1, CMP0HYS0 | CMP0HYS1 | CMP0HYS2, CMP0HYS2  );
/*-------------------------------------------------------------------------------------------------
  CMP0����ѯʱ������
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR2, CMP0CSEL1 | CMP0CSEL0, 0x00);

/*-------------------------------------------------------------------------------------------------
  �Ƚ����ж�ģʽ����
  00: �������ж�  01: �����ز����ж�  10: �½��ز����ж�  11: ����/�½��ز����ж�
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR0, CMP2IM0 | CMP2IM1, CMP2IM0 | CMP2IM1);
  SetReg(CMP_CR0, CMP1IM0 | CMP1IM1, CMP1IM0 | CMP1IM1);
  SetReg(CMP_CR0, CMP0IM0 | CMP0IM1, CMP0IM0 | CMP0IM1);
  SetBit(CMP_CR2, CMP0EN);//�������Ƚ���
}
/*---------------------------------------------------------------------------*/
/* Name    :  void Time2_RSD_Init(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  BMF��ӦTime2�ĳ�ʼ��
/*---------------------------------------------------------------------------*/
void Time2_BMEF_Init(void)
{
  /*-------------------------------------------------------------------------------------------------
  ��ֹͣ������������Ĵ����������������
-------------------------------------------------------------------------------------------------*/
  ClrBit(TIM2_CR1, T2EN);                                 // 0��ֹͣ������1,ʹ�ܼ���

/*-------------------------------------------------------------------------------------------------
  ʱ�ӷ�Ƶ����(T2PSC)
  000:cpuclk(24MHz)      001:cpuclk/2^1(12MHz)  010:cpuclk/2^2(6MHz)  011:cpuclk/2^3(3MHz)
  100:cpuclk/2^4(1.5MHz)  101:cpuclk/2^5(750KHz)  110:cpuclk/2^6(375KHz)  111:cpuclk/2^7(187.5KHz)
-------------------------------------------------------------------------------------------------*/
  SetReg(TIM2_CR0, T2PSC0 | T2PSC1 | T2PSC2, T2PSC0 | T2PSC1 | T2PSC2);
  /*-------------------------------------------------------------------------------------------------
  /ģʽѡ��
  T2MODE1��T2MODE0
  00--����Timerģʽ��01--���ģʽ
  10--����Countģʽ��11--QEP����RSDģʽ
-------------------------------------------------------------------------------------------------*/
  SetReg(TIM2_CR0, T2MOD0 | T2MOD1, T2MOD0);
  /*-------------------------------------------------------------------------------------------------
  ����жϱ�־λ
  ��ֹPWM���ڼ���ж�ʹ��
-------------------------------------------------------------------------------------------------*/
  ClrBit(TIM2_CR0, T2CES);                                   // ���������������ʹ��
  ClrBit(TIM2_CR1, T2IR | T2IF | T2IP);               // �����жϱ�־λ

/*-------------------------------------------------------------------------------------------------
  ��������ֵ���Ƚ�ֵ������ֵ
  ��ֹPWM���ڼ���ж�ʹ��
  ʹ�ܼ����������ж�ʹ��
-------------------------------------------------------------------------------------------------*/
  TIM2__ARR = 60000;                                         // TIM2 Period = 0.32s
  TIM2__DR = TIM2__ARR;
  TIM2__CNTR = 0;
/*-----------��������------------------------------------------------*/
  SetBit(TIM2_CR1, T2EN);                               //��������

}
/*---------------------------------------------------------------------------*/
/* Name    :  void GetBEMFStatus(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �����綯��״̬
/*---------------------------------------------------------------------------*/
uint8  GetBEMFStatus(void)
{
  uint8 BEMFStatus = 0;

  if(ReadBit(CMP_SR, CMP2OUT))
  {
    BEMFStatus += 4;
  }

  if(ReadBit(CMP_SR, CMP1OUT))
  {
    BEMFStatus += 2;
  }

  if(ReadBit(CMP_SR, CMP0OUT))
  {
    BEMFStatus += 1;
  }

  return BEMFStatus;

}

/*-------------------------------------------------------------------------------------------------
  Function Name :  uint8 CWCCWDetect(void)
  Description   :  ���ܺ����������ת�򣬸�����������Hall״̬˳�����жϵ��ת��
  Input         :  ��
  Output        :  MC_FR--���ת��ȡֵΪCW��CCW LastHallStatus
-------------------------------------------------------------------------------------------------*/
uint8 CWCCWDetect(uint8 HallStatus)
{
  static uint8 MC_FR = 0;
   static uint8 MC_HallStatus = 0;

  if(MC_HallStatus == 0)//��һ�ν����ж�
  {
    MC_HallStatus = HallStatus;
    MC_FR = CW;
    return MC_FR;
  }

  if(MC_HallStatus != HallStatus)
  {
    switch(MC_HallStatus)
    {
      case 1:
        if(HallStatus == 5)
        {
          MC_FR = CCW;
        }
        if(HallStatus == 3)
        {
          MC_FR = CW;
        }
        break;
      case 2:
        if(HallStatus == 3)
        {
          MC_FR = CCW;
        }
        if(HallStatus == 6)
        {
          MC_FR = CW;
        }
        break;
      case 3:
        if(HallStatus == 1)
        {
          MC_FR = CCW;
        }
        if(HallStatus == 2)
        {
          MC_FR = CW;
        }
        break;
      case 4:
        if(HallStatus == 6)
        {
          MC_FR = CCW;
        }
        if(HallStatus == 5)
        {
          MC_FR = CW;
        }
        break;
      case 5:
        if(HallStatus == 4)
        {
          MC_FR = CCW;
        }
        if(HallStatus == 1)
        {
          MC_FR = CW;
        }
        break;
      case 6:
        if(HallStatus == 2)
        {
          MC_FR = CCW;
        }
        if(HallStatus == 4)
        {
          MC_FR = CW;
        }
        break;
      default:
        break;
    }
    MC_HallStatus = HallStatus;
  }
  return MC_FR;
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  void BEMFSpeedDetect(void)
  Description   :  ����ٶȵļ�ʱ
  Input         :  ��
  Output        :  ��
-------------------------------------------------------------------------------------------------*/
void BEMFSpeedDetect(void)
{
  if(BEMFDetect.BEMFSpeedInitStatus == 0)
  {
    BEMFDetect.BEMFSpeedInitStatus =1;

    BEMFDetect.PeriodTime = 0;
    BEMFDetect.MC_StepTime[0] = 0;
    BEMFDetect.MC_StepTime[1] = 0;
    BEMFDetect.MC_StepTime[2] = 0;
    BEMFDetect.MC_StepTime[3] = 0;
    BEMFDetect.MC_StepTime[4] = 0;
    BEMFDetect.MC_StepTime[5] = 0;
    BEMFDetect.BEMFStep =0;
    BEMFDetect.StepTime =0;
    BEMFDetect.FirstCycle =0;
  }
  else
  {
    BEMFDetect.StepTime = TIM2__CNTR;
    TIM2__CNTR = 0;

    BEMFDetect.MC_StepTime[BEMFDetect.BEMFStep] = BEMFDetect.StepTime;

    BEMFDetect.PeriodTime = (BEMFDetect.MC_StepTime[0] + BEMFDetect.MC_StepTime[1] + BEMFDetect.MC_StepTime[2] +
                            BEMFDetect.MC_StepTime[3] + BEMFDetect.MC_StepTime[4] + BEMFDetect.MC_StepTime[5])>>3;

    BEMFDetect.BEMFStep++;

    if(BEMFDetect.FirstCycle)//360�ȣ���һȦ��360����һ���ٶȣ��ڶ�Ȧ��60�ȼ���һ���ٶ�
    {
      BEMFDetect.FlagSpeedCal = 1;
      BEMFDetect.BEMFSpeedBase = TempBEMFSpeedBase;
    }
    else//60��
    {
      BEMFDetect.FlagSpeedCal = 1;
      BEMFDetect.BEMFSpeedBase = TempBEMFSpeedBase1;
      BEMFDetect.PeriodTime = BEMFDetect.StepTime;
    }

    if(BEMFDetect.BEMFStep == 6)
    {
      BEMFDetect.FirstCycle = 1;
      BEMFDetect.BEMFStep = 0;
    }
  }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void BEMFSpeedCal(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �ٶȼ��㣬�õ�����Q��ʽ������
/*---------------------------------------------------------------------------*/
void BEMFSpeedCal(void)
{
  if(BEMFDetect.FlagSpeedCal)//�˴�ע��XDATA�ͳ���ֻ����16λ������
  {
    BEMFDetect.BEMFSpeed  = MDU_DIV_XDATA_U32(&BEMFDetect.BEMFSpeedBase, &BEMFDetect.PeriodTime);    //�õ�����Q��ʽ���ٶ�,�˴������������ֵ����ͻ
    BEMFDetect.FlagSpeedCal = 0;
  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void BEMFDetectFunc(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  BEMF��⣬�жϷ����ٶȣ��Լ�˳���бջ�
/*---------------------------------------------------------------------------*/
void BEMFDetectFunc(void)
{
  if(ReadBit(CMP_SR, CMP0IF)||ReadBit(CMP_SR, CMP1IF)||ReadBit(CMP_SR, CMP2IF))//����⵽�Ƚ����ж�ʱ
  {

    //��⵱ǰBEMF״̬
    BEMFDetect.BEMFStatus = GetBEMFStatus();

    //����BEMF״̬�ж�FR״̬
    BEMFDetect.FRStatus = CWCCWDetect(BEMFDetect.BEMFStatus);

    //�ٶȼ��
    BEMFSpeedDetect();

    //�ٶȼ���
    BEMFSpeedCal();

    //ǿ��������־ʹ��ʱ
    if(BEMFDetect.BEMFStartStatus)
    {
      //CWʱU��BEMF������������CCWʱV��BEMF����������
      if(((mcFRState.TargetFR == CW)&&(BEMFDetect.BEMFStatus == 5))||((mcFRState.TargetFR == CCW)&&(BEMFDetect.BEMFStatus == 3)))
      {
        //ִ��ֱ�ӱջ���������
        BEMFFOCCloseLoopStart();
        ClrBit(CMP_CR0, CMP2IM1 | CMP2IM0 | CMP1IM1 | CMP1IM0 | CMP0IM1 | CMP0IM0);
        ClrBit(CMP_CR2, CMP0EN);

        BEMFDetect.BEMFStartStatus =0;
      }
    }
    ClrBit(CMP_SR, CMP0IF | CMP1IF | CMP2IF);
  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void BEMFDealwith(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  BEMF����ʽ
/*---------------------------------------------------------------------------*/
void BEMFDealwith(void)
{
    if((BEMFDetect.BEMFTimeCount>=0)&&(BEMFDetect.BEMFTimeCount<(BEMF_START_DETECT_TIME-BEMF_START_DELAY_TIME)))
    {
        if(BEMFDetect.FRStatus == mcFRState.TargetFR)
        {
          //�����趨ת��ʱֱ������
          if((BEMFDetect.BEMFSpeed > BEMFMotorStartSpeed)&&(BEMFDetect.BEMFSpeed < BEMFMotorStartSpeedHigh))
          {
            BEMFDetect.BEMFStartStatus = 1;

          }
        }
        else//��ת��ɲ��
        {
            McStaSet.SetFlag.TailWindSetFlag=0;

            MOE = 0;
            DRV_DR = DRV_ARR+1;
            DRV_CMR &= 0xFFC0;
            DRV_CMR |= 0x015;                         // �������ű�ͨ��ɲ��
            ClrBit(DRV_CR,OCS);//OCS = 0, DRV_COMR;OCS = 1, FOC/SVPWM/SPWM
//            SetBit(DRV_CR, DRVEN);
            MOE = 1;

            ClrBit(CMP_CR0, CMP2IM1 | CMP2IM0 | CMP1IM1 | CMP1IM0 | CMP0IM1 | CMP0IM0);
            ClrBit(CMP_CR2, CMP0EN);

            if(BEMFDetect.BEMFSpeed > _Q15(300.0/MOTOR_SPEED_BASE))
            {
              mcFocCtrl.State_Count=2000;
              BEMFDetect.BEMFCCWFlag=1;
            }
            else
            {
              mcFocCtrl.State_Count=1000;
              if(BEMFDetect.BEMFCCWFlag==0)
              BEMFDetect.BEMFCCWFlag=2;
            }

        }
    }
    if((BEMFDetect.BEMFTimeCount == 0)&&(BEMFDetect.BEMFSpeed < BEMFMotorStartSpeed))//��ֹ�����
    {
      ClrBit(CMP_CR0, CMP2IM1 | CMP2IM0 | CMP1IM1 | CMP1IM0 | CMP0IM1 | CMP0IM0);
      ClrBit(CMP_CR2, CMP0EN);
      mcState = mcPosiCheck;
      McStaSet.SetFlag.PosiCheckSetFlag  = 0;
      mcFocCtrl.mcPosCheckAngle          = 0xffff;      // �Ƕȸ���ֵ
    }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void BEMFFOCCloseLoopStart(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �ջ�����
/*---------------------------------------------------------------------------*/
void BEMFFOCCloseLoopStart(void)
{
      /*FOC��ʼ��*/
    FOC_Init();

    /*����������KP��KI*/
    FOC_IDREF = ID_Start_CURRENT;                         // D����������
    mcFocCtrl.mcIqref= IQ_Start_CURRENT;                  // Q����������
    FOC_IQREF = mcFocCtrl.mcIqref;                        // Q����������

    FOC_DQKP = DQKP;
    FOC_DQKI = DQKI;

    FOC_EFREQACC   = Motor_Omega_Ramp_ACC;
    FOC_EFREQMIN   = Motor_Omega_Ramp_Min;
    FOC_EFREQHOLD = Motor_Omega_Ramp_End;

    SetBit(FOC_CR1,EFAE);                              // ������ǿ�����
    ClrBit(FOC_CR1,RFAE);                              // ��ֹǿ��
    SetBit(FOC_CR1,ANGM);                              // ����ģʽ

    FOC__EOME=BEMFDetect.BEMFSpeed;
//    FOC__UQ=(QOUTMAX>>1);

    #if (EstimateAlgorithm == SMO)
    {
      //���ݲ�ͬת��ȷ������ATO_BWֵ
      if(BEMFDetect.BEMFSpeed >_Q15(360.0/MOTOR_SPEED_BASE))
      {
        FOC_EKP                       = OBSW_KP_GAIN_RUN4;
        FOC_EKI                       = OBSW_KI_GAIN_RUN4;
        mcFocCtrl.mcIqref              = IQ_RUN_CURRENT;
        mcFocCtrl.State_Count         = 100;
      }
      else if(BEMFDetect.BEMFSpeed >_Q15(160.0/MOTOR_SPEED_BASE))
      {

        FOC_EKP                        = OBSW_KP_GAIN_RUN3;
        FOC_EKI                       = OBSW_KI_GAIN_RUN3;
        mcFocCtrl.mcIqref              = IQ_RUN_CURRENT;
        mcFocCtrl.State_Count         = 1000;
      }
      else
      {
        FOC_EKP                        = OBSW_KP_GAIN_RUN1;
        FOC_EKI                       = OBSW_KI_GAIN_RUN1;
        mcFocCtrl.mcIqref              = IQ_Start_CURRENT;
        mcFocCtrl.State_Count         = 2000;
      }
    }
    #elif (EstimateAlgorithm == PLL)
    {
       FOC_EKP                          = OBSW_KP_GAIN_RUN4;
       FOC_EKI                         = OBSW_KI_GAIN_RUN4;
       mcFocCtrl.mcIqref              = IQ_RUN_CURRENT;
    }
    #endif //end   EstimateAlgorithm
    FOC_OMEKLPF                     = SPEED_KLPF;
    mcState                         = mcRun;
    mcFocCtrl.CtrlMode              = 0;

    /*ʹ�����*/
    DRV_CMR |= 0x3F;                         // U��V��W�����
    MOE = 1;
    EA=1;
//    POWER_VSP = POWER_gear;

}

//static idata uint32 TempPower;
//void RealPowerCal(void)
//{
//  TempPower = mcFocCtrl.mcDcbusFlt;
//  TempPower = TempPower*mcFocCtrl.mcADCCurrentbus;
//  mcFocCtrl.mcSysPower = (uint16)(TempPower>>10);
//}