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
/* Description: ���״̬��������������ʼ����Ԥ��硢˳������жϡ�Ԥ��λ�����������С����ϵ�
/*---------------------------------------------------------------------------*/
void MC_Control(void)
{
  switch(mcState)
  {
    case mcReady:    // �ر����,�ϵ��Ե������вɼ�У׼,������У׼������־��1������ָ����1�󣬲���ת��mcInit
      Motor_Ready();
      if((mcCurOffset.OffsetFlag == 1) && (mcSpeedRamp.FlagONOFF == 1) && (mcFocCtrl.State_Count == 0))
      {
          mcState       = mcInit;
          FOC_EFREQACC  = 0;
          FOC_EFREQMIN  = 0;
          FOC_EFREQHOLD = 0;
      }
    break;

    case mcInit:                                       // ��ʼ��״̬������mcCharge״̬
      Motor_Init();
      mcState               =  mcCharge;               // ����mcCharge״̬
      mcFocCtrl.State_Count = Charge_Time;
    break;

    case mcCharge:                                     // Ԥ���״̬��MCU����̶�Ƶ��ռ�ձȣ�Ԥ������������mcTailWind
      Motor_Charge();
      #if (IPMState == NormalRun)                      // ���������״̬������
        if( mcFocCtrl.State_Count == 0)
        {
            MOE                   = 0;                 // �ر����
            mcState               = mcTailWind;
            mcFocCtrl.State_Count = 0;
        }
      #endif
    break;

    case mcTailWind:
      #if (TailWind_Mode == NoTailWind)               // ��˳��紦��ģ�ֱ��������һ��״̬
        mcState                           = mcPosiCheck;
        McStaSet.SetFlag.PosiCheckSetFlag = 0;
        mcFocCtrl.mcPosCheckAngle         = 0xffff;   // �Ƕȸ���ֵ

      #elif (TailWind_Mode == TailWind)
        Motor_TailWind();

      #endif
    break;

    case mcPosiCheck:
      #if (PosCheckEnable==0)                         //��ʼλ�ü�ⲻʹ��ʱ��ʼ�Ƕ�ΪԤ��λ�Ƕ�
        mcFocCtrl.mcPosCheckAngle = Align_Angle;
        mcState = mcAlign;
        mcFocCtrl.State_Count = Align_Time;
    
//      FOC_Init();
//       /*ʹ�����*/
//      DRV_CMR |= 0x3F;                         // U��V��W�����
//      MOE = 1;
//      mcState = mcStart;                       //����Ԥ��λ������

      #else
        RPD();

      #endif
    break;

    case mcAlign:       // Ԥ��λʱ�������ֱ������; AlignTestMode=1���ڳ�ʼλ�ü�������
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

    case mcStart:                           // ���õ����������������mcRun״̬��
      Motor_Open();
    break;

    case mcPllTect:                           // ���õ����������������mcRun״̬��
      #if (EstimateAlgorithm == PLL)
          Motor_PllStart();
      #endif
    break;

    case mcRun:                             // ����״̬��������״̬�ĸ�����Ϊ0������mcStop״̬��
      if(mcSpeedRamp.FlagONOFF == 0)
      {
          mcState   = mcStop;
          mcFocCtrl.State_Count = 200;
          FOC_IQREF = 0;
        
          MOE       = 0;
          ClrBit(DRV_CR, FOCEN);  //�ر�FOC
          FOC_CR1   = 0x00;
      }
    break;

    case mcStop:
        #if (StopBrakeFlag == 0)
        {
            FOC_CR1 = 0x00;
            /*�ر�FOC*/
            ClrBit(DRV_CR, FOCEN);
  
            mcState = mcReady;
        }
        #else
        {
            if(mcFocCtrl.State_Count==0) //��ʱɲ����������������ܸ��ٵ����ͣ
            {
                MOE      = 0;
                FOC_CR1  = 0x00;
                ClrBit(DRV_CR, FOCEN);
                DRV_DR   = DRV_ARR*1;
                DRV_CMR &= 0xFFC0;
                DRV_CMR |= 0x015;                                                // �������ű�ͨ��ɲ��
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
