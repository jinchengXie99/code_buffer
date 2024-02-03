/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : FocControl.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 10-Apr-2017
* Description        : This file contains all the foc control function used for motor control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <FU68xx_2_DMA.h>

CurrentOffset xdata mcCurOffset;

extern float brakeduty;
extern uint8  KeyLongPressFlag;
extern uint16 POWER_VSP;
extern bit ReverseCleanCmd;
/*---------------------------------------------------------------------------*/
/* Name    :  void FOC_Init(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  mcInit״̬�£���FOC����ؼĴ�����������,������Ĵ����������ã����ʹ��
/*---------------------------------------------------------------------------*/
void FOC_Init(void)
{
  /*ʹ��FOC*/
  ClrBit(DRV_CR, FOCEN);
  SetBit(DRV_CR, FOCEN);
  /*����FOC�Ĵ���*/
  FOC_CR1       = 0;                                      // ���� FOC_CR1
  FOC_CR2       = 0;                                      // ���� FOC_CR2
  FOC_IDREF     = 0;                                      // ���� Id
  FOC_IQREF     = 0;                                      // ���� Iq

  FOC__THETA     = 0;                                      // ���� �Ƕ�
  FOC_RTHEACC   = 0;                                      // ���� ���º����ĳ�ʼ���ٶ�
  FOC__RTHESTEP   = 0;                                    //
  FOC__RTHECNT   = 0;                                      //
  FOC__THECOMP   = _Q15(2.0/180.0);                        // SMO ���㲹����
  FOC__THECOR     = 0x02;                                  // ���ǶȲ���

  /*��������������*/
  FOC_DMAX       = DOUTMAX;
  FOC_DMIN       = DOUTMIN;

  FOC_QMAX       = QOUTMAX;
  FOC_QMIN       = QOUTMIN;

  /*λ�ù����������*/
  FOC_EK1       = OBS_K1T;
  FOC_EK2       = OBS_K2T;
  FOC_EK3       = OBS_K3T * 1.2;
  FOC_EK4       = OBS_K4T;
  FOC_FBASE     = OBS_FBASE;
  FOC_OMEKLPF   = SPEED_KLPF;
  FOC_EBMFK     = OBS_KLPF;

//  FOC_TGLI      = PWM_TGLI_LOAD;
  FOC_TGLI      = 0;

  /*********PLL��SMO**********/
  #if (EstimateAlgorithm == SMO)
  {
      ClrBit(FOC_CR2, ESEL);
      FOC_KSLIDE    = OBS_KSLIDE;
      FOC_EKLPFMIN  = OBS_EA_KS;
  }
  #elif (EstimateAlgorithm == PLL)
  {
      SetBit(FOC_CR2, ESEL);
      FOC_KSLIDE    = OBSE_PLLKP_GAIN;
      FOC_EKLPFMIN  = OBSE_PLLKI_GAIN;
  }
  #endif //end SVPMW_Mode

  SetBit(FOC_CR1, SVPWMEN);                              // SVPWMģʽ

  /*����ת*/
  if(mcFRState.FR==CCW)                                 // ����F/R
  {
    SetBit(DRV_CR,DDIR);
  }
  else
  {
    ClrBit(DRV_CR,DDIR);
  }
  
//  if(KS.KeyLongPressFlag == 0)
//  {
//    ClrBit(DRV_CR,DDIR);                               // ��ת��־λ
//  }
//  else
//  {
//    SetBit(DRV_CR,DDIR); 
//  }
  // ��ת��־λ
  #if (IRMODE==1)
  {
    SetBit(DRV_CR,DDIR);                               // ��ת��־λ
    if (ReverseCleanCmd == 1)
      ClrBit(DRV_CR,DDIR); 
  }
  #else
  {
    ClrBit(DRV_CR,DDIR);                               // ��ת��־λ
    if (ReverseCleanCmd == 1)
      SetBit(DRV_CR,DDIR); 
  }
  #endif //end IRMODE
    

  /**������**/
  #if (OverModulation == 1)
  {
      SetBit(FOC_CR1,OVMDL);                          // ������
  }
  #endif //end OverModulation

  /*�������������Ҫ��С������,FOC_TRGDLYΪ0���߶�ʽSVPWM��ʽ*/
  #if (Shunt_Resistor_Mode == Single_Resistor)
  {
    SetReg(FOC_CR1, CSM0 | CSM1, 0x00);
    FOC_TSMIN = PWM_TS_LOAD;                              // ��С��������
    FOC_TRGDLY = 0x9;                                    // ����ʱ�����е㣬һ�㿼�ǿ�������Ӱ�죬�������ӳ٣�
                                                          // 0x0c��ʾ�ӳ�12��clock����ǰ�÷�����ʽ����0x84��ʾ��ǰ12��clock��
    ClrBit(FOC_CR2,F5SEG);                                // 7��ʽ
    SetReg(CMP_CR1, CMP3MOD0 | CMP3MOD1, 0x00);
  }
  /*˫�����������������������ֵ�����½��ؽ���ǰ��ʼ����Ia������81*/
  #elif (Shunt_Resistor_Mode == Double_Resistor)          // double resistor sample
  {
    SetReg(FOC_CR1, CSM0 | CSM1, CSM0);

    FOC_TSMIN = PWM_DT_LOAD;                              // ��������ֵ
    FOC_TRGDLY = 0x83;                                    // ADC������ʱ�̣�����ʱ���ڼ�������㸽����83Ϊ�½��ؽ���ǰ3��clock����Ia���뵥���費ͬ
                                                          // 01Ϊ�����ؿ�ʼ���һ��clock��ʼ����������ʵ�����������
    FOC_TBLO=PWM_DLOWL_TIME;                              //���ű���С���壬��֤����
    SetReg(CMP_CR1, CMP3MOD0 | CMP3MOD1, 0x00);

    /*���ʽ���߶�ʽѡ��*/
    #if (SVPMW_Mode == SVPWM_7_Segment)
    {
      ClrBit(FOC_CR2,F5SEG);                              // 7��ʽ
    }
    #elif (SVPMW_Mode == SVPWM_5_Segment)
    {
      SetBit(FOC_CR2,F5SEG);                              // 7��ʽ
    }
    #endif

    #if (DouRes_Sample_Mode == DouRes_1_Cycle)
    {
      ClrBit(FOC_CR2,DSS);                                // 7��ʽ
    }
    #elif (DouRes_Sample_Mode == DouRes_2_Cycle)
    {
      SetBit(FOC_CR2,FOC_DSS);                            // 7��ʽ
    }
    #endif //end DouRes_Sample_Mode
  }
  /*���������*/
  #elif (Shunt_Resistor_Mode == Three_Resistor)            // signel resistor sample
  {

    SetReg(FOC_CR1, CSM0 | CSM1, CSM0 | CSM1);// ������

    FOC_TSMIN  = PWM_DT_LOAD;                              // ��������ֵ
    FOC_TRGDLY =0F;                                        // ADC������ʱ�̣�����ʱ���ڼ�������㸽����83Ϊ�½��ؽ���ǰ3��clock����Ia���뵥���費ͬ��
                                                          // 01Ϊ�����ؿ�ʼ���һ��clock��ʼ����������ʵ�����������

    SetReg(CMP_CR1, CMP3MOD0 | CMP3MOD1, CMP3MOD0 | CMP3MOD1);
    FOC_TBLO= PWM_OVERMODULE_TIME;                        // �����Ƶ������������TB����

    /*���ʽ���߶�ʽѡ��*/
    #if (SVPMW_Mode == SVPWM_7_Segment)
    {
      ClrBit(FOC_CR2,F5SEG);                              // 7��ʽ
    }
    #elif (SVPMW_Mode == SVPWM_5_Segment)
    {
      SetBit(FOC_CR2,F5SEG);                              // 7��ʽ
    }
    #endif //end SVPMW_Mode

    #if (DouRes_Sample_Mode == DouRes_1_Cycle)
    {
      ClrBit(FOC_CR2,DSS);                              // 7��ʽ
    }
    #elif (DouRes_Sample_Mode == DouRes_2_Cycle)
    {
      SetBit(FOC_CR2,DSS);                              // 7��ʽ
    }
    #endif //end DouRes_Sample_Mode
  }
  #endif  //end Shunt_Resistor_Mode


  /* ʹ�ܵ�����׼У��  */
  #if (CalibENDIS == Enable)
  {
    if(mcCurOffset.OffsetFlag==1)
    {
      #if (Shunt_Resistor_Mode == Single_Resistor)        // ������У��
      {
        /*set ibus current sample offset*/
        SetReg(FOC_CR2, CSOC0 | CSOC1, 0x00);
        FOC_CSO = mcCurOffset.Iw_busOffset;                // д��Ibus��ƫ��

      }
      #elif (Shunt_Resistor_Mode == Double_Resistor)      // ˫����У��
      {
        /*set ia, ib current sample offset*/
        SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC0);
        FOC_CSO  = mcCurOffset.IuOffset;                  // д��IA��ƫ��

        SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC1);
        FOC_CSO  = mcCurOffset.IvOffset;                  // д��IB��ƫ��

      }
      #elif (Shunt_Resistor_Mode == Three_Resistor)        // ������У��
      {
        /*set ibus current sample offset*/
        SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC0);
        FOC_CSO = mcCurOffset.IuOffset;                   // д��IA��ƫ��

        SetReg(FOC_CR2, CSOC0 | CSOC1, CSOC1);
        FOC_CSO = mcCurOffset.IvOffset;                    // д��IB��ƫ��

        SetReg(FOC_CR2, CSOC0 | CSOC1, 0x00);
        FOC_CSO = mcCurOffset.Iw_busOffset;                // д��IC��ƫ��
      }
      #endif  //end Shunt_Resistor_Mode
    }
  }
  #endif  //end CalibENDIS
  /*-------------------------------------------------------------------------------------------------
  DRV_CTL��PWM��Դѡ��
  OCS = 0, DRV_COMR
  OCS = 1, FOC/SVPWM/SPWM
  -------------------------------------------------------------------------------------------------*/
  /*�������Ƚ�ֵ��ԴFOC*/
  SetBit(DRV_CR, OCS);
}
/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Charge(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  Ԥ��磬��һֱ����Ԥ���״̬�£����ӵ������������֤IPM����Mos��
Ԥ������������һ���Ƕ�U�����Ԥ��磬�ڶ����Ƕ�U,V�������Ԥ���;�������Ƕ�U��V��W�������Ԥ��硣
/*---------------------------------------------------------------------------*/
void Motor_Charge(void)
{
     if(McStaSet.SetFlag.ChargeSetFlag==0)
     {
        McStaSet.SetFlag.ChargeSetFlag = 1;
         #if (IPMState == IPMtest)
         {
           DRV_DR = 0.7*DRV_ARR;                // IPM 70% duty
         }
         #elif (IPMState == NormalRun)          // ���������״̬������
         {
           DRV_DR = 0.1*DRV_ARR;                //���ű�10% duty
         }
        #endif
         /*-------------------------------------------------------------------------------------------------
        DRV_CTL��PWM��Դѡ��
        OCS = 0, DRV_COMR
        OCS = 1, FOC/SVPWM/SPWM
        -------------------------------------------------------------------------------------------------*/
        ClrBit(DRV_CR, OCS);
        mcFocCtrl.ChargeStep = 0;
     }
     if((mcFocCtrl.State_Count < Charge_Time)&&(mcFocCtrl.ChargeStep == 0))
     {
       mcFocCtrl.ChargeStep = 1;
       #if (IPMState == IPMtest)
       {
         DRV_CMR |= 0x03;                         // U�����
       }
       #elif (IPMState == NormalRun)              // ���������״̬������
       {
         DRV_CMR |= 0x01;                         // U�����ű�ͨ
       }
       #endif
       MOE = 1;
     }
     if(( mcFocCtrl.State_Count <= (Charge_Time<<1)/3)&&(mcFocCtrl.ChargeStep== 1))
     {
       mcFocCtrl.ChargeStep = 2;
       #if (IPMState == IPMtest)
       {
         DRV_CMR |= 0x0F;                         // U��V�����
       }
       #elif (IPMState == NormalRun)              // ���������״̬������
       {
         DRV_CMR |= 0x04;                         // V�����ű۵�ͨ
       }
       #endif
     }
     if((mcFocCtrl.State_Count <= Charge_Time/3)&&(mcFocCtrl.ChargeStep== 2))
     {
        mcFocCtrl.ChargeStep = 3;
       #if (IPMState == IPMtest)
       {
         DRV_CMR |= 0x3F;                         // U��V��W�����
       }
       #elif (IPMState == NormalRun)              // ���������״̬������
       {
         DRV_CMR |= 0x10;                         // W�����ű۵�ͨ
       }
       #endif
     }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Align(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  Ԥ��λ��������������ж�ʱ������Ԥ��λ�̶���ʼλ��;��������ж�ʱ������Ԥ��λɲ��
/*---------------------------------------------------------------------------*/
void Motor_Align(void)
{
  if(McStaSet.SetFlag.AlignSetFlag==0)
  {
      McStaSet.SetFlag.AlignSetFlag=1;

      /*FOC��ʼ��*/
      FOC_Init();

      /*����Ԥ��λ�ĵ�����KP��KI*/
      FOC_IDREF = ID_Align_CURRENT;
      FOC_IQREF = IQ_Align_CURRENT;

      FOC_DQKP = DQKP_Alignment;
      FOC_DQKI = DQKI_Alignment;

      FOC_EKP   = OBSW_KP_GAIN;
      FOC_EKI   = OBSW_KI_GAIN;

      /*����Ԥ��λ�Ƕ�*/
      #if (AlignTestMode==1)
      {
        FOC__THETA    = Align_Theta;
      }
      #else
      {
        #if (PosCheckEnable==1)
        {
          FOC__THETA   = mcFocCtrl.mcPosCheckAngle + _Q15((float)60.0/180.0);;
        }
        #else
        {
          FOC__THETA    =Align_Theta;
        }
        #endif  //end PosCheckEnable
      }
      #endif  //end AlignTestMode
            /*********PLL��SMO**********/
      #if (EstimateAlgorithm == SMO)
      {
        FOC__ETHETA   = FOC__THETA-4096;
      }
      #elif (EstimateAlgorithm == PLL)
      {
        FOC__ETHETA   = FOC__THETA;
      }
      #endif //end   EstimateAlgorithm
      TimeCnt=FOC__THETA;

      /*ʹ�����*/
      DRV_CMR |= 0x3F;                         // U��V��W�����
      MOE = 1;

  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Open(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  ���������Ĳ�������
/*---------------------------------------------------------------------------*/
void Motor_Open(void)
{
    static uint8 OpenRampCycles;
    if(McStaSet.SetFlag.StartSetFlag==0)
    {
      McStaSet.SetFlag.StartSetFlag=1;
//      POWER_VSP = POWER_gear;
      /****������ʼ�Ƕȸ�ֵ**/
      #if (PosCheckEnable)
      {
        FOC__THETA   = mcFocCtrl.mcPosCheckAngle;// �г�ʼλ�ü�⣬���ó�ʼλ�ý�
      }
      #else
      {
        FOC__THETA    = Align_Theta;            // �޳�ʼλ�ü�⣬����Ԥ��λ��
      }
      #endif
      /*********PLL��SMO**********/
      #if (EstimateAlgorithm == SMO)
      {
        FOC__ETHETA   = FOC__THETA-4915;
      }
      #elif (EstimateAlgorithm == PLL)
      {
        FOC__ETHETA   = FOC__THETA;
      }
      #endif //end   EstimateAlgorithm

      TimeCnt=FOC__ETHETA;
      /*����������KP��KI��FOC_EKP��FOC_EKI*/
      FOC_IDREF = ID_Start_CURRENT;                         // D����������
      mcFocCtrl.mcIqref= IQ_Start_CURRENT;                  // Q����������
      
      FOC_DQKP = DQKPStart;
      FOC_DQKI = DQKIStart;

      //����ʱ����Q��PI������ʼֵ
//      FOC_QMAX       = QOUTMAX;
//      FOC_QMIN       = QOUTMINST;
      
      FOC_EKP   = OBSW_KP_GAIN;
      FOC_EKI   = OBSW_KI_GAIN;
      
      /*������ʽѡ��*/
      #if (Open_Start_Mode == Omega_Start)                  // Omega ����
      {
        FOC_EFREQACC   = Motor_Omega_Ramp_ACC;
        FOC_EFREQMIN   = Motor_Omega_Ramp_Min;
        FOC_EFREQHOLD = Motor_Omega_Ramp_End;

        SetBit(FOC_CR1,EFAE);                              // ������ǿ�����
        ClrBit(FOC_CR1,RFAE);                              // ��ֹǿ��
        SetBit(FOC_CR1,ANGM);                              // ����ģʽ
      }
      #elif (Open_Start_Mode == Open_Start)
      {
        FOC_RTHEACC   = Motor_Open_Ramp_ACC;                // ���º����ĳ�ʼ���ٶ�
        FOC__RTHESTEP   = Motor_Open_Ramp_Min;              // 0.62 degree acce speed
        RPDPara.ThetaGet=Motor_Open_Ramp_Min;
        FOC__RTHECNT   = MOTOR_OPEN_ACC_CNT;                // acce time

        ClrBit(FOC_CR1,EFAE);                              // ������ǿ�����
        SetBit(FOC_CR1,RFAE);                              // ��ֹǿ��
        ClrBit(FOC_CR1,ANGM);                              // ����ģʽ
      }
      #elif (Open_Start_Mode == Open_Omega_Start)
      {
        FOC_RTHEACC   = Motor_Open_Ramp_ACC;                  // ���º����ĳ�ʼ���ٶ�
        FOC__RTHESTEP   = Motor_Open_Ramp_Min;                // 0.62 degree acce speed
        FOC__RTHECNT   = MOTOR_OPEN_ACC_CNT;                  // acce time

        FOC_EFREQACC   = Motor_Omega_Ramp_ACC;
        FOC_EFREQMIN   = Motor_Omega_Ramp_Min;
        FOC_EFREQHOLD = Motor_Omega_Ramp_End;

        SetBit(FOC_CR1,EFAE);                              // ������ǿ�����
        SetBit(FOC_CR1,RFAE);                              // ��ֹǿ��
        SetBit(FOC_CR1,ANGM);                              // ����ģʽ
      }
      #endif //end Open_Start_Mode
      
    }
      /*��ͬ������ʽ�£��л���MCRUN״̬*/
      #if (Open_Start_Mode == Open_Start)      //OPEN״̬����ʱ�϶����
      {
        if(OpenRampCycles<(MOTOR_OPEN_ACC_CYCLE-1))
        {
          if(!ReadBit(FOC_CR1,RFAE))
          {
            SetBit(FOC_CR1,RFAE);
            OpenRampCycles++;
          }
        }
        else
        {
          mcFocCtrl.State_Count = 2;

          mcState = mcRun;
        }
          FOC_EKP = OBSW_KP_GAIN_RUN4;                        // ���������PI��KP
          FOC_EKI  = OBSW_KI_GAIN_RUN4;                       // ���������PI��KI
      }
      #elif (Open_Start_Mode == Open_Omega_Start)
      {
          mcFocCtrl.State_Count = 200;
          mcState = mcRun;
      }
      #elif (Open_Start_Mode == Omega_Start)
      {
        /*********PLL��SMO**********/
        #if (EstimateAlgorithm == SMO)
        {
           mcFocCtrl.State_Count = 120;
        }
        #elif (EstimateAlgorithm == PLL)
        {
          FOC_EKP = OBSW_KP_GAIN_RUN4;                        // ���������PI��KP
          FOC_EKI  = OBSW_KI_GAIN_RUN4;                       // ���������PI��KI
        }
        #endif //end   EstimateAlgorithm
        mcState = mcRun;
      }
      #endif //end Open_Start_Mode
      
      FOC_IQREF = mcFocCtrl.mcIqref;                        // Q����������
}

/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Open(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  ���������Ĳ�������
/*---------------------------------------------------------------------------*/
void Motor_PllStart(void)
{
    static uint8 OpenRampCycles;
    if(TailWindDetect.PLLFlag==1)
    {
      TailWindDetect.PLLSpeed=FOC__EOME;
      if(Abs_F16(TailWindDetect.PLLSpeed)<100)  //���״̬�£��ٶȵ���100ʱ���������ﵽһ��ʱ���PLLFlag���㡣
      {
        TailWindDetect.PLLDetectCnt++;
        if(TailWindDetect.PLLDetectCnt>3000)    //��ֹ�������
        {
          TailWindDetect.PLLFlag=0;
          TailWindDetect.PLLTheta=FOC__ETHETA;
          FOC_IQREF =0;
        }
      }
      else
      {
        if(TailWindDetect.PLLDetectCnt>0)
        {
          TailWindDetect.PLLDetectCnt--;
        }
      }
    }
    if((McStaSet.SetFlag.StartSetFlag==0)&&(TailWindDetect.PLLFlag==0))
    {
      McStaSet.SetFlag.StartSetFlag=1;
      MOE = 0;
      FOC_Init();
      FOC__ETHETA    = TailWindDetect.PLLTheta+19000;//16668;//

      /*����������KP��KI��FOC_EKP��FOC_EKI*/
      FOC_IDREF = ID_Start_CURRENT;                         // D����������
      FOC_IQREF = mcFocCtrl.mcIqref;                        // Q����������

      FOC_DQKP = DQKP;
      FOC_DQKI = DQKI;

      /*������ʽѡ��*/
      #if (Open_Start_Mode == Omega_Start)                  // Omega ����
      {
        FOC_EFREQACC   = Motor_Omega_Ramp_ACC;
        FOC_EFREQMIN   = Motor_Omega_Ramp_Min;
        FOC_EFREQHOLD = Motor_Omega_Ramp_End;

        SetBit(FOC_CR1,EFAE);                              // ��ֹ������ǿ�����
        ClrBit(FOC_CR1,RFAE);                              // ʹ��ǿ��
        SetBit(FOC_CR1,ANGM);                              // ����ģʽ
      }
      #elif (Open_Start_Mode == Open_Start)
      {
        FOC_RTHEACC   = Motor_Open_Ramp_ACC;                      // ���º����ĳ�ʼ���ٶ�
        FOC__RTHESTEP   = Motor_Open_Ramp_Min;                // 0.62 degree acce speed
        FOC__RTHECNT   = MOTOR_OPEN_ACC_CNT;                  // acce time

        ClrBit(FOC_CR1,EFAE);                              // ��ֹ������ǿ�����
        SetBit(FOC_CR1,RFAE);                              // ʹ��ǿ��
        ClrBit(FOC_CR1,ANGM);                              // ����ģʽ
      }
      #elif (Open_Start_Mode == Open_Omega_Start)
      {
        FOC_RTHEACC   = Motor_Open_Ramp_ACC;                      // ���º����ĳ�ʼ���ٶ�
        FOC__RTHESTEP   = Motor_Open_Ramp_Min;                // 0.62 degree acce speed
        FOC__RTHECNT   = MOTOR_OPEN_ACC_CNT;                  // acce time

        FOC_EFREQACC   = Motor_Omega_Ramp_ACC;
        FOC_EFREQMIN   = Motor_Omega_Ramp_Min;
        FOC_EFREQHOLD = Motor_Omega_Ramp_End;

        SetBit(FOC_CR1,EFAE);                              // ��ֹ������ǿ�����
        SetBit(FOC_CR1,RFAE);                              // ʹ��ǿ��
        SetBit(FOC_CR1,ANGM);                              // ����ģʽ
      }
      #endif //end Open_Start_Mode
      DRV_CMR |= 0x3F;                         // U��V��W�����
      MOE = 1;
    }

    if((McStaSet.SetFlag.StartSetFlag==1)&&(TailWindDetect.PLLFlag==0))
    {
      /*��ͬ������ʽ�£��л���MCRUN״̬*/
      #if (Open_Start_Mode == Open_Start)      //OPEN״̬����ʱ�϶����
      {
        if(OpenRampCycles<(MOTOR_OPEN_ACC_CYCLE-1))
        {
          if(!ReadBit(FOC_CR1,RFAE))
          {
            SetBit(FOC_CR1,RFAE);
            OpenRampCycles++;
          }
        }
        else
        {
          mcFocCtrl.State_Count = 2;
          mcState = mcRun;
        }
          FOC_EKP = OBSW_KP_GAIN_RUN4;                          // ���������PI��KP
          FOC_EKI  = OBSW_KI_GAIN_RUN4;                          // ���������PI��KI
      }
      #elif (Open_Start_Mode == Open_Omega_Start)
      {
          mcFocCtrl.State_Count = 2600;
          mcState = mcRun;
      }
      #elif (Open_Start_Mode == Omega_Start)
      {

          FOC_EKP = OBSW_KP_GAIN_RUN4;                          // ���������PI��KP
          FOC_EKI  = OBSW_KI_GAIN_RUN4;                          // ���������PI��KI
          mcState = mcRun;
      }
      #endif //end Open_Start_Mode
    }

}
/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Align(void)
/* Input   :  NO
/* Output  :  NO
/* Description:  ˳�����������ú���
/*---------------------------------------------------------------------------*/
void Motor_TailWind(void)
{
  if(mcFocCtrl.State_Count == 0)
  {
    if(McStaSet.SetFlag.TailWindSetFlag == 0)//��ʼ��
    {
        McStaSet.SetFlag.TailWindSetFlag = 1;

        #if (FRDetectMethod==RSDMethod)
         {
           RSDDetectInit();
         }
        #elif (FRDetectMethod==BEMFMethod)
         {
           BEMFDetectInit();
         }
        #elif (FRDetectMethod==FOCMethod)
         {
           TailWindDetectInit();
         }
        #endif

    }
    #if (FRDetectMethod==RSDMethod)
    {
      RSDDealwith();
    }
    #elif (FRDetectMethod==BEMFMethod)
    {
      BEMFDealwith();
    }
    #elif (FRDetectMethod==FOCMethod)
    {
      FOCTailWindDealwith();
    }
   #endif

  }
}

/*---------------------------------------------------------------------------*/
/* Name    :  void MC_Stop(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  inital motor control parameter
/*---------------------------------------------------------------------------*/
void MC_Stop(void)
{
  MOE     = 0;
  ClrBit(DRV_CR, FOCEN, 0);  //�ر�FOC                                                      // disable FOC output and initial register
  mcState = mcInit;
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  void MotorControlInit(void)
  Description   :  ���Ʊ�����ʼ������,�������������ĳ�ʼ�������״̬��ʼ��
  Input         :  ����˵������ϸ��
  Output        :  ���˵������ϸ��
-------------------------------------------------------------------------------------------------*/
void MotorcontrolInit(void)
{
  /***********����******************/
  memset(&mcFaultDect,0, sizeof(FaultVarible));                                // FaultVarible��������
  
  /******��������*********/
  memset(&mcProtectTime,0, sizeof(ProtectVarible));                            // ProtectVarible������������
  
  /*******��������*****************/
  memset(&mcCurVarible,0, sizeof(CurrentVarible));                            // ���������ı�������

  /*******��ͣ���ԵĲ���***************/
  memset(&ONOFFTest,0, sizeof(ONVarible));
  
   /*****�ⲿ���ƻ�*******/
  memset(&mcFocCtrl,0, sizeof(FOCCTRL));                                // mcFocCtrl��������

  /******ADC�����˲�ֵ*********/
  memset(&AdcSampleValue,0, sizeof(ADCSample));                         // ADCSample��������

  /******����ƫ��У׼����*****/
  memset(&mcCurOffset,0, sizeof(CurrentOffset));                        // mcCurOffset��������
  mcCurOffset.IuOffsetSum            = 16383;
  mcCurOffset.IvOffsetSum            = 16383;
  mcCurOffset.Iw_busOffsetSum        = 16383;

  /*****LED����Ӧ***/
  memset(&mcLedDisplay,0, sizeof(MCLedDisplay));                        // mcLedDisplay��������
  mcLedDisplay.Counttime            = 4999;

  /*****�ٶȻ�����Ӧ***/
  memset(&mcSpeedRamp,0, sizeof(MCRAMP));                               // mcSpeedRamp��������
  mcSpeedRamp.DelayPeriod           = 1;

  /*PWM���ٱ���*/
  memset(&mcPwmInput,0, sizeof(PWMINPUTCAL));                           // ����PWM duty ���ٱ���mcPwmInput

  /*���ڱ���*/
  memset(&Uart,0, sizeof(MCUART));                                      // MCUART��������

  /*˯��ģʽ*/  
  memset(&SleepSet,0, sizeof(SLEEPMODE));                               // ˯��ģʽ����

 /*****���״̬��ʱ�����***********/
  McStaSet.SetMode                   = 0;

  /*****���Ŀ�귽��**********/
  mcFRState.TargetFR                 = 0;
  mcFRState.FR = 0;
  POWER_VSP = POWER_LOW;
}

/*---------------------------------------------------------------------------*/
/* Name    :  void VariablesPreInit(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  ��ʼ���������
/*---------------------------------------------------------------------------*/
void VariablesPreInit(void)
{
  /***********����******************/
  memset(&mcFaultDect,0, sizeof(FaultVarible));                                  // FaultVarible��������

  /*****�ⲿ���ƻ�*******/
  memset(&mcFocCtrl,0, sizeof(FOCCTRL));                                        // mcFocCtrl��������

  /*****LED����Ӧ***/
  memset(&mcLedDisplay,0, sizeof(MCLedDisplay));                                // mcLedDisplay��������
  mcLedDisplay.Counttime            = 4999;
  
  memset(&User,0, sizeof(USER_TYPEDEF));  
  
  /*****���״̬��ʱ�����***********/
  McStaSet.SetMode                   = 0;
  /*****ɲ���׿�ռ�ձ�***/
  brakeduty = Brakeduty_F;
  
  KS.FlashWriteOnetimes = 0; //����Ƿѹ��������¼��λ
  
  mcFocCtrl.TPCtrlDealy = 1000;                    //�����������500ms���ٿ�����˿
}
/*---------------------------------------------------------------------------*/
/* Name    :  void GetCurrentOffset(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �ϵ�ʱ���ȶ�Ӳ����·�ĵ������вɼ���д���Ӧ��У׼�Ĵ����С�
                ����ʱ����۲�mcCurOffset�ṹ���ж�Ӧ�����Ƿ��ڷ�Χ�ڡ��ɼ�������OffsetFlag��1��
/*---------------------------------------------------------------------------*/
void GetCurrentOffset(void)
{
  if(mcCurOffset.OffsetFlag==0)
  {      
    SetBit(ADC_CR, ADCBSY);                               // ʹ��ADC
    while(ReadBit(ADC_CR, ADCBSY));
    #if (Shunt_Resistor_Mode == Single_Resistor)             // 29.2ms ������ģʽ���ϵ���֤Ӳ����·ʱ����۲�mcCurOffset.IbusOffset�Ƿ�Ϊ4096
    {
      mcCurOffset.Iw_busOffsetSum+=((ADC4_DR& 0x0fff) << 3);
      mcCurOffset.Iw_busOffset = mcCurOffset.Iw_busOffsetSum >> 4;
      mcCurOffset.Iw_busOffsetSum -= mcCurOffset.Iw_busOffset;
    }
    #elif (Shunt_Resistor_Mode == Double_Resistor)           //44ms ˫����ģʽ���ϵ���֤Ӳ����·ʱ����۲�mcCurOffset.IaOffset��mcCurOffset.IbOffset�Ƿ�Ϊ4096
    {
      mcCurOffset.IuOffsetSum+=((ADC0_DR& 0x0fff) << 3);
      mcCurOffset.IuOffset = mcCurOffset.IuOffsetSum >> 4;
      mcCurOffset.IuOffsetSum -= mcCurOffset.IuOffset;

      mcCurOffset.IvOffsetSum+=((ADC1_DR& 0x0fff) << 3);
      mcCurOffset.IvOffset = mcCurOffset.IvOffsetSum >> 4;
      mcCurOffset.IvOffsetSum -= mcCurOffset.IvOffset;

    }
    #elif (Shunt_Resistor_Mode == Three_Resistor)            //58.2ms ������ģʽ���ϵ���֤Ӳ����·ʱ����۲�mcCurOffset.IaOffset��mcCurOffset.IbOffset��mcCurOffset.IcOffset�Ƿ�Ϊ4096
    {
      mcCurOffset.IuOffsetSum+=((ADC0_DR& 0x0fff) << 3);
      mcCurOffset.IuOffset = mcCurOffset.IuOffsetSum >> 4;
      mcCurOffset.IuOffsetSum -= mcCurOffset.IuOffset;

      mcCurOffset.IvOffsetSum+=((ADC1_DR& 0x0fff) << 3);
      mcCurOffset.IvOffset = mcCurOffset.IvOffsetSum >> 4;
      mcCurOffset.IvOffsetSum -= mcCurOffset.IvOffset;

      mcCurOffset.Iw_busOffsetSum+=((ADC4_DR& 0x0fff) << 3);
      mcCurOffset.Iw_busOffset = mcCurOffset.Iw_busOffsetSum >> 4;
      mcCurOffset.Iw_busOffsetSum -= mcCurOffset.Iw_busOffset;
    }
    #endif

    mcCurOffset.OffsetCount++;
    if(mcCurOffset.OffsetCount>Calib_Time)
    {
      mcCurOffset.OffsetFlag=1;
    }
  }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Ready(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �ϵ�ʱ���ر�������ȶ�Ӳ����·�ĵ������вɼ�����FOC_Init��д���Ӧ��У׼�Ĵ����С�
                ����ʱ����۲�mcCurOffset�ṹ���ж�Ӧ�����Ƿ��ڷ�Χ�ڡ�
/*---------------------------------------------------------------------------*/
void Motor_Ready(void)
{
   if(McStaSet.SetFlag.CalibFlag==0)
   {
     McStaSet.SetFlag.CalibFlag=1;
     ClrBit(DRV_CR, FOCEN);      // �ر�FOC
     MOE      = 0;               // �ر�MOE
     SetBit(ADC_MASK_SYSC, CH7EN |CH4EN | CH3EN | CH2EN | CH1EN | CH0EN);// ����ADC
     mcCurOffset.OffsetFlag=0;   // ��ʼ�����ɼ�
     mcCurOffset.OffsetCount = 0; //�����ɼ���������
   }
}
/*---------------------------------------------------------------------------*/
/* Name    :  void Motor_Init(void)
/* Input  :  NO
/* Output  :  NO
/* Description:  �Ե����ر�����PI���г�ʼ������
/*---------------------------------------------------------------------------*/
void Motor_Init(void)
{
   ClrBit(ADC_MASK_SYSC, CH4EN | CH0EN);// �ر��������������ADC
   VariablesPreInit();                           // �����ر�����ʼ��
   PI_Init();                                     // PI��ʼ��
  
//   LEDControl();                                    //��Ͳ�ϵ��ȸ�ת�ٺ��¶�ֵ   �Ƶ����� ����ֱ�ӿ�����������û��
}
#endif