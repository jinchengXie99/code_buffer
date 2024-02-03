/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : RSDDetect.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 10-Apr-2017
* Description        : This file contains init speed detection used for Motor Control.
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

/*---------------------------------------------------------------------------*/
/* Name    :  void RSDDetectInit(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  RSD��ʼ��
/*---------------------------------------------------------------------------*/
void RSDDetectInit(void)
{
  MOE = 0;

  RSDDetect.RSDStepTime[0]= 0;
  RSDDetect.RSDStepTime[1]= 0;
  RSDDetect.RSDStepTime[2]= 0;
  RSDDetect.RSDStepTime[3]= 0;
  RSDDetect.RSDTimes   = 0;
  RSDDetect.RSDPeriod  = 0;
  RSDDetect.RSDCount   = 0;
  RSDDetect.RSDState   = Static;
  RSDDetect.RSDSpeed   = 0;
  RSDDetect.RSDDIR     = 0;
  RSDDetect.RSDFlag    = 0;
  RSDDetect.RSDStep    = 0;
  RSDDetect.RSDBRFlag  = 0;
    
  ClrBit(DRV_CR, FOCEN);  // �ر�FOC
  
  CMP_RSD_Init();
  Time2_RSD_Init();       // RSD�õ���Time2
}
/*---------------------------------------------------------------------------*/
/* Name    :  void CMP_RSD_Init(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  RSD��CMP�ĳ�ʼ��
/*---------------------------------------------------------------------------*/
void CMP_RSD_Init(void)
{
/*-------------------------------------------------------------------------------------------------
  CMP Input Pin Mode
   P1.4--CMP0_IN+, P1.6--CMP1_IN+, P2.1--CMP2_IN+
   P1.5--CMP0_IN-, P1.7--CMP1_IN-, P2.2--CMP2_IN-
   P1.3--CMP1P2
-------------------------------------------------------------------------------------------------*/
  SetBit(P1_AN, P14 | P15 | HBMOD);                                   // 
  ClrBit(P1_OE,P13);
/*-------------------------------------------------------------------------------------------------
  CMP0_MOD��
  00��  �������������ĵ�����BEMFģʽ
  01��  �����������ĵ�����BEMFģʽ
  10��  3��ֱȽ���ģʽ
  11��  2�Ƚ���ģʽRSD
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR2, CMP0MOD0 | CMP0MOD1, CMP0MOD0 | CMP0MOD1);

/*-------------------------------------------------------------------------------------------------
  �Ƚ������ѡ�����ã���CMP0_MOD���ʹ��
  CMP0_SEL[1:0]=00���Ƚ���0������2�Ƚ�����ѯģʽ��������CMP0P��CMP1P2֮���Զ�����ѡ�񣬸��˹̶���CMP0M��
                    ���������ֱ�����CMP0_OUT��CMP1_OUT
  CMP0_SEL[1:0]=01���Ƚ���0ѡ��CMP0��Ӧ�Ķ˿���ϣ������˽�CMP0P�����˽�CMP0M�������CMP0_OUT
  CMP0_SEL[1:0]=10���Ƚ���0ѡ��CMP1��Ӧ�Ķ˿���ϣ������˽�CMP1P2�����˽�CMP0M�������CMP1_OUT
-----------------------------------------------------------------------------*/
  SetReg(CMP_CR2, CMP0SEL0 | CMP0SEL1, 0x00);

/*-------------------------------------------------------------------------------------------------
  �Ƚ������͵�ѹѡ��
  000: �޳���   001: ��2.5mV   010: -5mV   011: +5mV
  100: +-5mV   101: -10mV   110: +10mV   111: +-10mV
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR1, CMP0HYS0 | CMP0HYS1 | CMP0HYS2, CMP0HYS0 | CMP0HYS1 | CMP0HYS2 );

/*-------------------------------------------------------------------------------------------------
  CMP0����ѯʱ������
-------------------------------------------------------------------------------------------------*/
  SetReg(CMP_CR1, CMP0CSEL0 | CMP0CSEL1, 0x00);

  EA = 0;

/**------------------------------------------------------
ʹ�ܱȽ���CMP0,CMP1,CMP2��ADC��pwm on/off��������

00����on��off��������û���ӳٲ�������
01��ֻ��off����������CMP_SAMR�ӳٲ�������
10��ֻ��on����������CMP_SAMR�ӳٲ�������
11����on��off������������CMP_SAMR�ӳٲ�������
---------------------------------------------------------**/
  SetReg(CMP_CR3, SAMSEL0 | SAMSEL1, 0);

  /**�����ӳ�����**/
  CMP_SAMR = 0x10;

  SetBit(CMP_CR2, CMP0EN);//ʹ�ܱȽ���
}
/*---------------------------------------------------------------------------*/
/* Name    :  void Time2_RSD_Init(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  RSD��ӦTime2�ĳ�ʼ��
/*---------------------------------------------------------------------------*/
void Time2_RSD_Init(void)
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
  SetReg(TIM2_CR0, T2MOD0 | T2MOD1, T2MOD0 | T2MOD1);

  SetBit(TIM2_CR1, T2FE);                               // �˲�ʹ��
/*-------------------------------------------------------------------------------------------------
  ����жϱ�־λ
  ��ֹPWM���ڼ���ж�ʹ��
  ʹ�ܼ����������ж�ʹ��
-------------------------------------------------------------------------------------------------*/
  ClrBit(TIM2_CR1, T2IR | T2IF | T2IP);                       // ����жϱ�־λ
  ClrBit(TIM2_CR0, T2CES | T2IRE);                               // ���������������ʹ��
  SetBit(TIM2_CR1, T2IPE | T2IFE);                               // ������Ч���ر仯�ж�ʹ�ܺͻ�������������ʹ��
  /*-------------------------------------------------------------------------------------------------
  ��ʱ��2�ж����ȼ����ü�оƬ�ж���ʹ��
  PTIM231-PTIM230���ж����ȼ�����ֵ��0-3���α�ʾ���ȼ�����͵���ߣ���4���Ż�������
  EA,оƬ�ж���ʹ��
-------------------------------------------------------------------------------------------------*/
  PTIM21 = 1;
  PTIM20 = 0;                                             // TIM2/2�ж����ȼ���Ϊ2
  EA = 1;
/*-------------------------------------------------------------------------------------------------
  ��������ֵ���Ƚ�ֵ������ֵ
-------------------------------------------------------------------------------------------------*/
  TIM2__CNTR = 0;
  
/*-----------��������------------------------------------------------*/
  SetBit(TIM2_CR1, T2EN);                               //��������
}
/*---------------------------------------------------------------------------*/
/* Name    :  void RSDDetect(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  RSD��⣬�ж���ת���Ƿ�ת
/*---------------------------------------------------------------------------*/
void RSDFRDetect(void)
{
  RSDDetect.RSDCount=TIM2__CNTR;//ע��RSDCount��ΪIDATA��������
   if(RSDDetect.RSDCount < -7)//��ת
   {
     RSDDetect.RSDPeriod=(RSDDetect.RSDStepTime[0]+RSDDetect.RSDStepTime[1]+RSDDetect.RSDStepTime[2]+RSDDetect.RSDStepTime[3])>>2;
     RSDDetect.RSDDIR=ReadBit(TIM2_CR1,T2DIR);
     ClrBit(TIM2_CR1, T2EN);  //�ص���ʱ��2
     RSDDetect.RSDState=Forward;
     RSDDetect.RSDFlag=1;
   }
   else if(RSDDetect.RSDCount > 7)//��ת
   {
     RSDDetect.RSDPeriod=(RSDDetect.RSDStepTime[0]+RSDDetect.RSDStepTime[1]+RSDDetect.RSDStepTime[2]+RSDDetect.RSDStepTime[3])>>2;
     RSDDetect.RSDDIR=ReadBit(TIM2_CR1,T2DIR);
     ClrBit(TIM2_CR1, T2EN);  //�ص���ʱ��2        
     RSDDetect.RSDState=Reverse;
     RSDDetect.RSDFlag=1;
   }
   else//����ж�
   {
      RSDDetect.RSDStepTime[RSDDetect.RSDStep]=TIM2__ARR;//���Ƚ����Ķ�ֵ��RSDStep
      RSDDetect.RSDStep++;
      if(RSDDetect.RSDStep>3)
      {
        RSDDetect.RSDStep=0;
      }
   }
   RSDDetect.RSDTimes++;//����жϣ�����û����������ת
   if(RSDDetect.RSDTimes>15)
   {
      RSDDetect.RSDState=Static;
      RSDDetect.RSDFlag=1;
   }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void RSDDealwith(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  RSD����ʽ
/*---------------------------------------------------------------------------*/
void RSDDealwith(void)
{
  if(RSDDetect.RSDFlag==1)
  {
    if(RSDDetect.RSDState != Static)//�����ٶ�
    {
       RSDDetect.RSDSpeedBase=RSDSpeedBaseStep;
       RSDDetect.RSDPeriod+=1;
       RSDDetect.RSDSpeed  = MDU_DIV_IDATA_U32(&RSDDetect.RSDSpeedBase,&RSDDetect.RSDPeriod);//Q��ʽ���ٶ�,�˴������������ֵ����ͻ
    }
    if (((RSDDetect.RSDState==Reverse)&&(RSDDetect.RSDDIR==0x00))&&(RSDDetect.RSDSpeed>_Q15(80.0/MOTOR_SPEED_BASE))&&(RSDDetect.RSDCCWTimes<4))//��ת����һ���ٶȣ���ɲ������С��4��ʱ������
    {
        mcFocCtrl.State_Count=1000;
        McStaSet.SetFlag.TailWindSetFlag=0;
        MOE = 0;
        DRV_DR = DRV_ARR+1;
        DRV_CMR &= 0xFFC0;
        DRV_CMR |= 0x015;                         // �������ű�ͨ��ɲ��
        ClrBit(DRV_CR, OCS);//OCS = 0, DRV_COMR;OCS = 1, FOC/SVPWM/SPWM
        MOE = 1;
        
        if(RSDDetect.RSDSpeed > _Q15(200.0/MOTOR_SPEED_BASE))//�ٶȳ���һ��ֵʱ��ɲ�������ñ�־λ��������ʱ�����ݲ�ͬ�ٶ����ò�ͬ��������
        {         
          mcFocCtrl.State_Count=1500;
          RSDDetect.RSDCCWFlag=1;
        }
        else
        {          
          mcFocCtrl.State_Count=800;
          if(RSDDetect.RSDCCWFlag==0)
          RSDDetect.RSDCCWFlag=2;
        }
       RSDDetect.RSDCCWTimes++;
       ClrBit(CMP_CR2, CMP0EN);                                //�رձȽ���
       ClrBit(TIM2_CR1, T2EN);                                 // 0��ֹͣ������1,ʹ�ܼ���
    }
    else if ((RSDDetect.RSDState==Forward)&&(RSDDetect.RSDDIR==0x01))//��ת
    {      
      RSDFOCCloseLoopStart();                              
      RSDCloseDeal();

      ClrBit(CMP_CR2, CMP0EN);                                 // �رձȽ���
      ClrBit(TIM2_CR1, T2EN);                                   // 0��ֹͣ������1,ʹ�ܼ���
    }
    else//�������羲ֹ
    {
      ClrBit(CMP_CR2, CMP0EN);                               // �رձȽ���
      ClrBit(TIM2_CR1, T2EN);                                 // 0��ֹͣ������1,ʹ�ܼ���
      if(RSDDetect.RSDCCWFlag!=0)
      {
         if(RSDDetect.RSDBRFlag==0)                          // ɲ��400ms����������������µĳɹ���
         {
          RSDDetect.RSDCCWSBRCnt=400;
          MOE = 0;
          DRV_DR = DRV_ARR+1;
          DRV_CMR &= 0xFFC0;;
          DRV_CMR |= 0x015;                                  // �������ű�ͨ��ɲ��
          ClrBit(DRV_CR, OCS);                               // OCS = 0, DRV_COMR;OCS = 1, FOC/SVPWM/SPWM
          MOE = 1;
          RSDDetect.RSDBRFlag=1;
        }
      }
      if((RSDDetect.RSDCCWFlag==0)||(RSDDetect.RSDCCWSBRCnt==0))
      {        
        MOE = 0;
        mcState = mcPosiCheck;
        McStaSet.SetFlag.PosiCheckSetFlag  = 0;
        mcFocCtrl.mcPosCheckAngle          = 0xffff;      // �Ƕȸ���ֵ
      }    
    }
  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void RSDFOCCloseLoopStart(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �ջ�����
/*---------------------------------------------------------------------------*/
void RSDFOCCloseLoopStart(void)
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

    SetBit(FOC_CR1,ANGM);                              // ����ģʽ
    ClrBit(FOC_CR1,RFAE);                              // ��ֹǿ��
    SetBit(FOC_CR1,EFAE);                              // ������ǿ�����

    FOC__EOME=RSDDetect.RSDSpeed;

    if(RSDDetect.RSDSpeed>_Q15(2400.0/MOTOR_SPEED_BASE))
    {
      FOC__UQ=16000;//
    }  
    else if(RSDDetect.RSDSpeed>_Q15(1800.0/MOTOR_SPEED_BASE))
    {
      FOC__UQ=10000;//
    }  
    else if(RSDDetect.RSDSpeed>_Q15(1200.0/MOTOR_SPEED_BASE))
    {
      FOC__UQ=8000;//
    } 
    else if(RSDDetect.RSDSpeed>_Q15(200.0/MOTOR_SPEED_BASE))
    {
      FOC__UQ=4000;//
    }      
    FOC__UD=0;
    #if (EstimateAlgorithm == SMO)
    {
      //���ݲ�ͬת��ȷ������ATO_BWֵ
      if(RSDDetect.RSDSpeed>_Q15(360.0/MOTOR_SPEED_BASE))
      {
        FOC_EKP                       = OBSW_KP_GAIN_RUN4;
        FOC_EKI                       = OBSW_KI_GAIN_RUN4;
        mcFocCtrl.mcIqref              = IQ_RUN_CURRENT;
        mcFocCtrl.State_Count         = 100;
      }
      else if(RSDDetect.RSDSpeed>_Q15(160.0/MOTOR_SPEED_BASE))
      {
        FOC_EKP                       = OBSW_KP_GAIN_RUN3;
        FOC_EKI                       = OBSW_KI_GAIN_RUN3;
        mcFocCtrl.mcIqref              = IQ_RUN_CURRENT;
        mcFocCtrl.State_Count         = 100;
      }
      else
      {
        FOC_EKP                        = OBSW_KP_GAIN_RUN2;
        FOC_EKI                       = OBSW_KI_GAIN_RUN2;
        mcFocCtrl.mcIqref              = IQ_RUN_CURRENT;
        mcFocCtrl.State_Count         = 100;
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
}

