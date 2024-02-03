/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : Interrupt.c
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 2017-12-26
* Description        : This file contains all the interrupt function used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>

uint16 xdata spidebug[4] = { 0 };
extern uint16 KeyCount_Long;
uint16  Power_Currt;

/*-------------------------------------------------------------------------------------------------
    Function Name : void FO_INT(void)
    Description   : FO_INT interrupt��Ӳ��FO�����������ض�������ж����ȼ����
    Input         : ��
    Output        : ��
-------------------------------------------------------------------------------------------------*/
//void FO_INT(void) interrupt 1                                                   // Ӳ��FO�����жϣ��ر����
//{
//    FaultProcess();                                                             // �ر����
//    mcFaultSource = FaultHardOVCurrent;                                         // Ӳ����������
//    mcState       = mcFault;                                                    // ״̬ΪmcFault
//    IF0           = 0;                                                          // clear P00 interrupt flag
//}
    
/*---------------------------------------------------------------------------*/
/* Name     :   void EXTERN_INT(void) interrupt 2
/* Input    :   NO
/* Output   :   NO
/* Description: �������
/*---------------------------------------------------------------------------*/
//void EXTERN_INT(void) interrupt 2
//{
//  mcFaultDect.VoltDetecExternCnt = 50;
//  if((mcFocCtrl.TPCtrlDealy ==0)&&(mcState == mcRun))
//  {
////    TempAverage();                    //�¶���ƽ��ֵ
////    TemperaturePID();                 //�¶ȵ���PID
////    
////    TIM3__ARR = User.ScrOnAngleTimeDatum;
////    TIM3__ARR = 20000;
////    TIM3__DR  = TIM3__ARR - 400;  
////    TIM3__CNTR = 0;
////    SetBit(TIM3_CR1, T3EN);            //����������
//    
////  mcFocCtrl.HW_Gcd =  Get_Gcd(mcFocCtrl.HW_ZeroT, mcFocCtrl.HW_ZeroZ);
////  mcFocCtrl.HW_ZeroZ = mcFocCtrl.HW_ZeroZ / mcFocCtrl.HW_Gcd;
////  mcFocCtrl.HW_ZeroT = mcFocCtrl.HW_ZeroT / mcFocCtrl.HW_Gcd;
//    
//    
////    mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_LOW][1];
////    mcFocCtrl.HW_ZeroT = HW_Gear[Temp_LOW][0];

//    if(mcFocCtrl.HW_ZeroCount < mcFocCtrl.HW_ZeroZ)
//    {
//      mcFocCtrl.HW_ZeroCount++;
//    }
//    else
//    {
//      mcFocCtrl.HW_ZeroCount = 1;
//    }
//    
//    if(mcFocCtrl.HW_ZeroCount <= mcFocCtrl.HW_ZeroT)
//    {
////      HW1 = HWON;
////      HW2 = HWON;
//    }
//    else
//    {
//      HW1 = HWOFF;
//      HW2 = HWOFF;
//    }

//  }
//  else
//  {
//    HW1 = HWOFF;
//    HW2 = HWOFF;
//  }
//  ClrBit(P2_IF, P20);                                                          // ����P20��־λ
//}

/*---------------------------------------------------------------------------*/
/* Name     :   void FOC_INT(void) interrupt 3
/* Input    :   NO
/* Output   :   NO
/* Description: FOC�ж�(Drv�ж�),ÿ���ز�����ִ��һ�Σ����ڴ�����Ӧ�ϸߵĳ����ж����ȼ��ڶ���DCEN���˾ͻ�����жϡ�
/*---------------------------------------------------------------------------*/
//void FOC_INT(void) interrupt 3
//{
//    if(ReadBit(DRV_SR, DCIF))                                                   // �Ƚ��ж�
//    {
//        APP_DIV();                                                               //���������������������ֵ�еĳ�����ͻ

//        Fault_Overcurrent(&mcCurVarible);                                       // �����������.Լ30us

//        #if ((FRDetectMethod == FOCMethod) && (TailWind_Mode == TailWind))
//            TailWindSpeedDetect();                                              //˳�����
//        #endif

//        #if defined (SPI_DBG_SW)                                                //�������ģʽ
//            spidebug[0] = mcFocCtrl.mcDcbusFlt<<1;                              //SPIDATA0;
//            spidebug[1] = FOC__IA;                                              //SPIDATA1;
//            spidebug[2] = FOC__IB;                                              //SPIDATA2;
//            spidebug[3] = FOC__THETA;                                            //SPIDATA3;
//        #endif

//        ClrBit(DRV_SR, DCIF);
//    }
//}

/*---------------------------------------------------------------------------*/
/* Name     :   void TIM2_INT(void) interrupt 4
/* Input    :   NO
/* Output   :   NO
/* Description:  ����ת���(RSD)
/*---------------------------------------------------------------------------*/
void TIM2_INT(void) interrupt 4
{
    if(ReadBit(TIM2_CR1, T2IR))
    {
        ClrBit(TIM2_CR1, T2IR);
    }
    if(ReadBit(TIM2_CR1, T2IP))
    {
        #if (FRDetectMethod == RSDMethod)
            RSDFRDetect();                                                      //RSD����ת���
        #endif

        ClrBit(TIM2_CR1, T2IP);
    }
    if(ReadBit(TIM2_CR1, T2IF))                                                 //����ж�,�����жϾ�ֹ,ʱ��Ϊ349ms��
    {
        #if (FRDetectMethod == RSDMethod)
            RSDDetect.RSDState=Static;
            RSDDetect.RSDFlag=1;
        #endif
      
        ClrBit(TIM2_CR1, T2IF);
    }
}

/*-------------------------------------------------------------------------------------------------
    Function Name : void CMP_ISR(void)
    Description   : CMP3��Ӳ���Ƚ��������������ض�������ж����ȼ����
                    CMP0/1/2��˳����ж�
    Input         : ��
    Output        : ��
-------------------------------------------------------------------------------------------------*/
//void CMP_ISR(void) interrupt 7
//{
//    if(ReadBit(CMP_SR, CMP3IF))
//    {
//        if(mcState!=mcPosiCheck)
//        {
//            FaultProcess();                                                     // �ر����
//            mcFaultSource=FaultHardOVCurrent;                                   // Ӳ����������
//            mcState = mcFault;                                                  // ״̬ΪmcFault
//        }
//        else
//        {
//            MOE     = 0;                                                                        // �ر�MOE
//            RPDPara.InsetCount[RPDPara.injecttimes]  = TIM2__CNTR;                              // ����ʱ��2�ļ���ֵ��ֵ������
//            RPDPara.DetectCount[RPDPara.injecttimes] = RPDPara.InsetCount[RPDPara.injecttimes]; // �������ݣ�һ�����ڹ۲�ԭʼ���ݣ�һ�����ڴ�������
//            TIM2__CNTR                               = 0;                                       // TIM2������ֵ����
//            RPDPara.injecttimes++;                                                              // RPDע�������ۼ�
//        }
//        ClrBit(CMP_SR, CMP3IF);
//    }

//    #if (FRDetectMethod == BEMFMethod)
//        //ͨ��BEMF��˳����������
//        BEMFDetectFunc();
//    #endif
//}

/*---------------------------------------------------------------------------*/
/* Name     :   void TIM23_INT(void) interrupt 9
/* Input    :   NO
/* Output   :   NO
/* Description: Capture PWM ���ж����ȼ��ڶ�������FOC�жϣ�����PWM����
/*---------------------------------------------------------------------------*/
void TIM3_INT(void) interrupt 9
{
    if(ReadBit(TIM3_CR1, T3IR))
    {
      ClrBit(TIM3_CR1, T3IR);
    }
    if(ReadBit(TIM3_CR1, T3IP))                                                  //�����ж�
    {
       ClrBit(TIM3_CR1, T3IP);
    }
    if(ReadBit(TIM3_CR1, T3IF))
    {
       ClrBit(TIM3_CR1, T3EN);            //�رռ�����      
       ClrBit(TIM3_CR1, T3IF);
    }
}

/*---------------------------------------------------------------------------*/
/* Name     :   void TIM_1MS_INT(void) interrupt 10
/* Input    :   NO
/* Output   :   NO
/* Description: 1ms��ʱ���жϣ�SYS TICK�жϣ������ڴ����ӹ��ܣ�����ƻ�·��Ӧ�����ֱ����ȡ��ж����ȼ�����FO�жϺ�FOC�жϡ�
/*---------------------------------------------------------------------------*/
//void TIM_1MS_INT(void) interrupt 10
//{
//    if(ReadBit(DRV_SR, SYSTIF))                                                  // SYS TICK�ж�
//    {    
//        SetBit(ADC_CR, ADCBSY);                                                  //ʹ��ADC��DCBUS����
//        /****�����˲�*****/
//        if(mcState == mcRun)
//        {
//            mcFocCtrl.CurrentPower = FOC__POW << 1;
//            mcFocCtrl.Powerlpf     = LPFFunction(mcFocCtrl.CurrentPower,mcFocCtrl.Powerlpf,20);      //ע���ͨ�˲���ϵ����ΧΪ0---127
//        }

//        /****�ٶ��˲������綯���˲�*****/
//        if((mcState != mcInit) && (mcState != mcReady))
//        {
//            mcFocCtrl.SpeedFlt = LPFFunction(FOC__EOME, mcFocCtrl.SpeedFlt, 30);              //100      //ע���ͨ�˲���ϵ����ΧΪ0---127
//            mcFocCtrl.EsValue  = LPFFunction(FOC__ESQU,mcFocCtrl.EsValue,10);
//        }
//        else
//        {
//            mcFocCtrl.SpeedFlt = 0;
//        }
//        Power_Currt = (ADC3_DR<<3);
//        if(Power_Currt > mcCurOffset.Iw_busOffset)
//        {
//         Power_Currt   = Power_Currt - mcCurOffset.Iw_busOffset;
//        }
//        else  
//        {
//          Power_Currt   = 0;
//        }
//        
//        mcFocCtrl.mcADCCurrentbus  = LPFFunction(Power_Currt,mcFocCtrl.mcADCCurrentbus,100);
//        /****UQ��ѹֵ�˲�****/
//        mcFocCtrl.UqFlt = LPFFunction(FOC__UQ,mcFocCtrl.UqFlt,10);              // UQֵ
//        mcFocCtrl.UdFlt = LPFFunction(FOC__UD,mcFocCtrl.UdFlt,10);              // UDֵ

//        Motor_EN();
//        
////        KEYControl();
//        
//        Speed_response();                                                        //��·��Ӧ�����ٶȻ���ת�ػ������ʻ���            
//        
//        #if (StartONOFF_Enable)
//            ONOFF_Starttest(&ONOFFTest);
//        #endif

//        /*****DCbus�Ĳ�����ȡֵ���˲�******/
//        AdcSampleValue.ADCDcbus = ADC2_DR<<3;
//        mcFocCtrl.mcDcbusFlt    = LPFFunction(AdcSampleValue.ADCDcbus,mcFocCtrl.mcDcbusFlt,100);
//        
//        /***********����˿�¶Ȳɼ�***********/
//        AdcSampleValue.ADCTemp  = ADC1_DR<<3 ;
//        User.Temperature = LPFFunction(AdcSampleValue.ADCTemp ,User.Temperature,20);  
//       /*********���ư��¶Ȳɼ�*********/
//        mcFocCtrl.mcADCTemperature = LPFFunction((ADC7_DR<<3),mcFocCtrl.mcADCTemperature,10);        

//        /*****���ϱ����������ܣ����Ƿѹ����������������ȱ�ࡢ��ת��********/
//        Fault_Detection();

////        LED_Display();//LED����ʾ

//        /********˯��ģʽ*******/
////        Sleepmode();

//        /*****���״̬����ʱ����*****/
//        if(mcFocCtrl.State_Count > 0)          mcFocCtrl.State_Count--;
//        if(BEMFDetect.BEMFTimeCount > 0)       BEMFDetect.BEMFTimeCount--;
//        if(RSDDetect.RSDCCWSBRCnt > 0)         RSDDetect.RSDCCWSBRCnt--;
//        if(mcFaultDect.VoltDetecExternCnt > 0) mcFaultDect.VoltDetecExternCnt--;
//        if(KS.KeyLongPressFlag==1)            KeyCount_Long++;
////        if(KeyCount_Long>=12000)
////        {
////          mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_Off][1];
////          mcFocCtrl.HW_ZeroT = HW_Gear[Temp_Off][0];
////          mcSpeedRamp.FlagONOFF = 0;
////          mcSpeedRamp.TargetValue = 0;
////          KS.KeyLongPressFlag = 0;
////          KeyCount_Long = 0;
////        }
//        
//        if(mcState == mcRun)
//        {
//          if(mcFocCtrl.TPCtrlDealy > 0)         mcFocCtrl.TPCtrlDealy--;
//        }

//        #if (FRDetectMethod==FOCMethod)
//            FOCTailWindTimeLimit();
//        #endif

//        /*****����������º�������*****/
//        StarRampDealwith();

//        ClrBit(DRV_SR, SYSTIF);                                                 // �����־λ
//    }
//    
//    
//    
//    if(ReadBit(TIM4_CR1, T4IR))
//    {
//      ClrBit(TIM4_CR1, T4IR);
//    }
//    if(ReadBit(TIM4_CR1, T4IP))                                                  //�����ж�
//    {
//      ClrBit(TIM4_CR1, T4IP);
//    }
//    if(ReadBit(TIM4_CR1, T4IF))
//    {
//      ClrBit(TIM4_CR1, T4IF);
//    }
//}

/*---------------------------------------------------------------------------*/
/* Name     :   void USART_INT(void) interrupt 12
/* Input    :   NO
/* Output   :   NO
/* Description: �����жϣ��ж����ȼ���ͣ����ڽ��յ����ź�,���жϲ���ʱ8us
/*---------------------------------------------------------------------------*/
//void USART_INT(void)  interrupt 12
//{
//    if(RI == 1)
//    {
//        RI = 0;
//        Uart.Uredata= UT_DR;                                                    //����������
//    }
//}

/* Private variables ----------------------------------------------------------------------------*/
void INT0(void) interrupt 0
{
}
void INT5(void) interrupt 5
{
}
void INT6(void) interrupt 6
{
}
void INT8(void) interrupt 8
{
}
void INT11(void) interrupt 11
{
}
void INT13(void) interrupt 13
{
}
void INT14(void) interrupt 14
{
}
void INT15(void) interrupt 15
{
}
