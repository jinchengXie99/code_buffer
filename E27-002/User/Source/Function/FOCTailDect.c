/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : FOCTailDect.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 2017-12-26
* Description        : This file contains foc tailwind detection used for Motor Control.
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
MotorTailWindTypeDef xdata  TailWindDetect;
/*---------------------------------------------------------------------------*/
/* Name     :   void TailWindDetectInit(void)
/* Input    :   NO
/* Output   :   NO
/* Description: ˳��������ʼ��
/*---------------------------------------------------------------------------*/
void TailWindDetectInit(void)
{
    TailWindDetect.MotorTailWindState     = NormalTailWind;                     //  ��ʼ״̬Ϊ������˳���״̬
    TailWindDetect.TempThailWindSpeedBase = ThailWindSpeedBase;                 //  ˳�������speedbase

    FOC_Init();                                                                 // FOC�ĳ�ʼ��
    FOC_DQKP    = DQKP_TailWind;                                                // ˳���ĵ�����KP
    FOC_DQKI    = DQKI_TailWind;                                                // ˳���ĵ�����KI
    FOC_EKP     = OBSW_KP_GAIN_WIND;                                            // ˳����ٶȹ����KP
    FOC_EKI     = OBSW_KI_GAIN_WIND;                                            // ˳����ٶȹ����KI
    FOC_OMEKLPF = SPEED_KLPF_WIND;                                              // ˳����µ��ٶ��˲�ϵ��
    SetBit(FOC_CR1, ANGM);                                                      // ����ģʽ
    // SetBit(DRV_CR, DRVEN);                                                   // Driver ���������ʹ�ܣ�0-��ֹ��1-ʹ��
    DRV_CMR   |= 0x3F;                                                          // U��V��W�����
    MOE       = 1;                                                              // ��MOE
    FOC_IDREF = 0;                                                              // D���������
    FOC_IQREF = 0;
}
/*---------------------------------------------------------------------------*/
/* Name     :   void TailWindSpeedDetect(void)
/* Input    :   NO
/* Output   :   NO
/* Description: ˳����ٶȼ�⣬���ݽǶȵı仯�������жϷ���
/*---------------------------------------------------------------------------*/
void TailWindSpeedDetect(void)
{
    static int16 LatestTheta;

    if(mcState == mcTailWind)
    {
        //˳�����������FOC�ڲ����ݵ�������Ƕȣ���ֱ�Ӷ�FOC_ETHETA���д���
        if(TailWindDetect.MotorTailWindState == NormalTailWind)
        {
            LatestTheta = FOC__ETHETA;

            //��ת�����ж��� <-170��   <10 >-10  >170������״̬�л���ʱ��
            if(LatestTheta < -30946)
            {
                //������δ������״̬3����״̬1ʱ����
                if((TailWindDetect.SpeedTimerClearStatus == 0) || (TailWindDetect.AngleState == 3))
                {
                    TailWindDetect.SpeedCountTimer       = 0;
                    TailWindDetect.SpeedTimerClearStatus = 1;
                    if(TailWindDetect.AngleState == 3) TailWindDetect.ShakeTimes++;//����1��3֮�䶶��������������1
                }
                //<-170��  ʱ����״̬Ϊ1��������SpeedCountTimer��TIM5�м�ʱ
                TailWindDetect.AngleState = 1;
            }
            else if((LatestTheta>-1820)&&(LatestTheta<1820)) //<10 >-10
            {
                //״̬1��״̬3�л���״̬2ʱ���浱ǰת��ʱ����TailWindDetect.SpeedCount[SpeedStoreNum]
                if((TailWindDetect.AngleState==1)||(TailWindDetect.AngleState==3))
                {
                    //���㵱ǰת�٣�RPM
                    TailWindDetect.SpeedCountTimer += 1;//��ֹΪ0
                    TailWindDetect.TailWindSpeed   = MDU_DIV_XDATA_U32(&TailWindDetect.TempThailWindSpeedBase, &TailWindDetect.SpeedCountTimer);//����ٶ�
                    TailWindDetect.SpeedStoreNum++;

                    //����SpeedCountTimer����
                    if(TailWindDetect.SpeedTimerClearStatus == 1) TailWindDetect.SpeedTimerClearStatus = 0;

                    //��1״̬�л���2״̬˵���Ƕȵ�����ת����ΪCW��3->2��ΪCCW
                    if(TailWindDetect.AngleState == 1)      TailWindDetect.MotorDir = CW;
                    else if(TailWindDetect.AngleState == 3) TailWindDetect.MotorDir = CCW;
                    TailWindDetect.ShakeTimes = 0;//������ض����ļ���
                }
                TailWindDetect.AngleState = 2;

            }
            //>170��ʱ
            else if(LatestTheta>30946)
            {
                //������δ������״̬1����״̬3ʱ����
                if((TailWindDetect.SpeedTimerClearStatus == 0) || (TailWindDetect.AngleState == 1))
                {
                    TailWindDetect.SpeedCountTimer       = 0;
                    TailWindDetect.SpeedTimerClearStatus = 1;
                    if(TailWindDetect.AngleState==1) TailWindDetect.ShakeTimes++;//����1��3֮�䶶��
                }
                TailWindDetect.AngleState = 3;
            }
        }
    }
}
/*---------------------------------------------------------------------------*/
/* Name     :   void FOCCloseLoopStart(void)
/* Input    :   NO
/* Output   :   NO
/* Description: ˳������
/*---------------------------------------------------------------------------*/
void FOCCloseLoopStart(void)
{
    //���������Ĳ�����Omegaģʽ
    FOC_EFREQACC    = Motor_Omega_Ramp_ACC;
    FOC_EFREQMIN    = Motor_Omega_Ramp_Min;
    FOC_EFREQHOLD = Motor_Omega_Ramp_End;

    SetBit(FOC_CR1,EFAE);                                                       // ������ǿ�����
    ClrBit(FOC_CR1,RFAE);                                                       // ��ֹǿ��
    SetBit(FOC_CR1,ANGM);                                                       // ����ģʽ

    //��������PI������޸�ֵ
    FOC_DQKP = DQKP;
    FOC_DQKI = DQKI;
    FOC_DMAX = DOUTMAX;
    FOC_DMIN = DOUTMIN;
    FOC_QMAX = QOUTMAX;
    FOC_QMIN = QOUTMIN;

    /*********PLL��SMO**********/
    #if (EstimateAlgorithm == SMO)
        //���ݲ�ͬת��ȷ������ATO_BWֵ
        if(TailWindDetect.TailWindSpeed > 300)//300rpm/min
        {
            FOC_EKP               = OBSW_KP_GAIN_RUN4;
            FOC_EKI               = OBSW_KI_GAIN_RUN4;
            mcFocCtrl.mcIqref     = IQ_RUN_CURRENT;
            mcFocCtrl.State_Count = 10;
        }
        else if(TailWindDetect.TailWindSpeed > 100)//300rpm/min
        {
            FOC_EKP               = OBSW_KP_GAIN_RUN3;
            FOC_EKI               = OBSW_KI_GAIN_RUN3;
            mcFocCtrl.mcIqref     = IQ_RUN_CURRENT;
            mcFocCtrl.State_Count = 120;
        }
        else
        {
            FOC_EKP               = OBSW_KP_GAIN_RUN;
            FOC_EKI               = OBSW_KI_GAIN_RUN;
            mcFocCtrl.mcIqref     = IQ_RUN_CURRENT;
            mcFocCtrl.State_Count = 120;
        }

    #elif (EstimateAlgorithm == PLL)
         mcFocCtrl.mcIqref = IQ_RUN_CURRENT;

    #endif //end    EstimateAlgorithm


    /*estimate parameter set*/
    FOC_OMEKLPF = SPEED_KLPF;
    FOC_IDREF   = ID_RUN_CURRENT;     // D����������
    FOC_IQREF   = mcFocCtrl.mcIqref ;         // Q����������
    PI_UK       =   mcFocCtrl.mcIqref   ;


    mcState            = mcRun;
    mcFocCtrl.CtrlMode = 0;
}

/*---------------------------------------------------------------------------*/
/* Name     :   void FOCTailWindTimeLimit(void)
/* Input    :   NO
/* Output   :   NO
/* Description: FOC˳������ʱ�䴦��
/*---------------------------------------------------------------------------*/
void FOCTailWindTimeLimit(void)
{
    if(mcState == mcTailWind)
    {
        //1Hz Min
        if(TailWindDetect.SpeedCountTimer < TailWind_Time)                      // SpeedCountTimer-����ʱ����
        {
            TailWindDetect.SpeedCountTimer++;
        }
        else
        {
            TailWindDetect.TailWindSpeed       = 0;                             //�ٶ�Ϊ0
            TailWindDetect.SpeedCountTimer     = 0;
            TailWindDetect.SpeedOverFlowStatus = 1;                             // ת�ټ��������־
        }
    }
}
 /*---------------------------------------------------------------------------*/
/* Name     :   void TailWindDealwith(void)
/* Input    :   NO
/* Output   :   NO
/* Description: ˳�����������ú���
/*---------------------------------------------------------------------------*/
void FOCTailWindDealwith(void)
{
    /*********PLL��SMO**********/
    #if (EstimateAlgorithm == SMO)
         SMO_TailWindDealwith();

    #elif (EstimateAlgorithm == PLL)
         PLL_TailWindDealwith();

    #endif //end    EstimateAlgorithm
}

void SMO_TailWindDealwith(void)
{
    //���״̬Ϊ��ת��ת�ټ������ʱ����1-3״̬������������2��ʱ����Ϊ��ֹ
    if(((TailWindDetect.MotorDir == CW) && (TailWindDetect.SpeedOverFlowStatus))||(TailWindDetect.ShakeTimes > 2))
    {
        MOE = 0;
        SetBit(DRV_CR, FOCEN, 0);   //�ر�FOC
        mcState                           = mcPosiCheck;
        McStaSet.SetFlag.PosiCheckSetFlag = 0;
        mcFocCtrl.mcPosCheckAngle         = 0xffff;         // �Ƕȸ���ֵ
    }
    else if((TailWindDetect.MotorDir == CCW) && (TailWindDetect.SpeedStoreNum > 2))//||((TailWindDetect.MotorDir == CW)&&(TailWindDetect.TailWindSpeed>TailWindStartMinSpeed)))
    {
        if(TailWindDetect.TailWindSpeed < 130)
        {
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(0.5);       // Q����������
            TailWindDetect.AntiFlag = 1;
        }
        else if(TailWindDetect.TailWindSpeed < 300)
        {
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(1.2);       // Q����������
            TailWindDetect.AntiFlag = 2;
        }
        else if(TailWindDetect.TailWindSpeed < 500)
        {
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(2.0);       // Q����������
            TailWindDetect.AntiFlag = 3;
        }
        else
        {
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(2.0);       // Q����������
            TailWindDetect.AntiFlag = 4;
        }

        FOC_DQKP              = DQKP;
        FOC_DQKI              = DQKI;

        FOC_DMAX              = DOUTMAX;
        FOC_DMIN              = DOUTMIN;
        FOC_QMAX              = QOUTMAX;
        FOC_QMIN              = QOUTMIN;
        FOC_EK2               = OBS_K2T_SMO;

        FOC_IQREF             = mcFocCtrl.mcIqref;
        mcFocCtrl.State_Count = 120;
        FOC_EKP               = OBSW_KP_GAIN;
        FOC_EKI               = OBSW_KI_GAIN;

        FOC_IDREF             = 0;
        mcState               = mcRun;
        mcFocCtrl.CtrlMode    = 0;

    }
    else if(TailWindDetect.MotorDir == CW)
    {
        //����������TailWindDetect.SpeedStoreNum��ת��TailWindStartMinSpeed����ʱֱ��˳������
        if((TailWindDetect.TailWindSpeed > TailWindStartMinSpeed) && (TailWindDetect.SpeedStoreNum >= 2))
        {
            FOCCloseLoopStart();
        }
    }
}

void PLL_TailWindDealwith(void)
{
    //���״̬Ϊ��ת��ת�ټ������ʱ����1-3״̬������������2��ʱ����Ϊ��ֹ
    if(((TailWindDetect.MotorDir == CW) && (TailWindDetect.SpeedOverFlowStatus)) || (TailWindDetect.ShakeTimes > 2))
    {
        MOE = 0;
        SetBit(DRV_CR, FOCEN, 0);   //�ر�FOC
        mcState                           = mcPosiCheck;
        McStaSet.SetFlag.PosiCheckSetFlag = 0;
        mcFocCtrl.mcPosCheckAngle         = 0xffff;         // �Ƕȸ���ֵ

    }
    else if((TailWindDetect.MotorDir == CCW) && (TailWindDetect.TailWindSpeed > 50) && (TailWindDetect.SpeedStoreNum >= 2))// �������תʱ�����ݲ�ͬת�����õ�����������mcPllTect״̬��
    {
        if(TailWindDetect.TailWindSpeed < 130)
        {
            FOC_IQREF               = I_Value(-0.1);
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(0.5);         // Q����������
            TailWindDetect.AntiFlag = 1;
        }
        else if(TailWindDetect.TailWindSpeed < 380)
        {
            FOC_IQREF               = I_Value(-0.4);
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(0.5);         // Q����������
            TailWindDetect.AntiFlag = 2;
        }
        else if(TailWindDetect.TailWindSpeed < 500)
        {
            FOC_IQREF               = I_Value(-0.6);
            mcFocCtrl.mcIqref       = IQ_Start_CURRENT + I_Value(0.8);         // Q����������
            TailWindDetect.AntiFlag = 3;
        }
        else
        {
                FOC_IQREF = I_Value(-0.7);
                mcFocCtrl.mcIqref= IQ_Start_CURRENT + I_Value(0.8);         // Q����������
                TailWindDetect.AntiFlag             = 4;
        }
        TailWindDetect.PLLFlag = 1;
        mcState                = mcPllTect;
    }
    else if(TailWindDetect.MotorDir == CW)//
    {
        //����������TailWindDetect.SpeedStoreNum,��ת�ٴ���TailWindStartMinSpeed����ֱ��˳������
        if((TailWindDetect.TailWindSpeed > TailWindStartMinSpeed) && (TailWindDetect.SpeedStoreNum >= 2))
        {
            FOCCloseLoopStart();
        }
    }

}