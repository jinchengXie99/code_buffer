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
    Description   : FO_INT interrupt，硬件FO过流保护，关断输出，中断优先级最高
    Input         : 无
    Output        : 无
-------------------------------------------------------------------------------------------------*/
//void FO_INT(void) interrupt 1                                                   // 硬件FO过流中断，关闭输出
//{
//    FaultProcess();                                                             // 关闭输出
//    mcFaultSource = FaultHardOVCurrent;                                         // 硬件过流保护
//    mcState       = mcFault;                                                    // 状态为mcFault
//    IF0           = 0;                                                          // clear P00 interrupt flag
//}
    
/*---------------------------------------------------------------------------*/
/* Name     :   void EXTERN_INT(void) interrupt 2
/* Input    :   NO
/* Output   :   NO
/* Description: 过零点检测
/*---------------------------------------------------------------------------*/
//void EXTERN_INT(void) interrupt 2
//{
//  mcFaultDect.VoltDetecExternCnt = 50;
//  if((mcFocCtrl.TPCtrlDealy ==0)&&(mcState == mcRun))
//  {
////    TempAverage();                    //温度求平均值
////    TemperaturePID();                 //温度调节PID
////    
////    TIM3__ARR = User.ScrOnAngleTimeDatum;
////    TIM3__ARR = 20000;
////    TIM3__DR  = TIM3__ARR - 400;  
////    TIM3__CNTR = 0;
////    SetBit(TIM3_CR1, T3EN);            //开启计数器
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
//  ClrBit(P2_IF, P20);                                                          // 清零P20标志位
//}

/*---------------------------------------------------------------------------*/
/* Name     :   void FOC_INT(void) interrupt 3
/* Input    :   NO
/* Output   :   NO
/* Description: FOC中断(Drv中断),每个载波周期执行一次，用于处理响应较高的程序，中断优先级第二。DCEN开了就会产生中断。
/*---------------------------------------------------------------------------*/
//void FOC_INT(void) interrupt 3
//{
//    if(ReadBit(DRV_SR, DCIF))                                                   // 比较中断
//    {
//        APP_DIV();                                                               //启动除法器，避免与过调值中的除法冲突

//        Fault_Overcurrent(&mcCurVarible);                                       // 软件过流保护.约30us

//        #if ((FRDetectMethod == FOCMethod) && (TailWind_Mode == TailWind))
//            TailWindSpeedDetect();                                              //顺逆风检测
//        #endif

//        #if defined (SPI_DBG_SW)                                                //软件调试模式
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
/* Description:  正反转检测(RSD)
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
            RSDFRDetect();                                                      //RSD正反转检测
        #endif

        ClrBit(TIM2_CR1, T2IP);
    }
    if(ReadBit(TIM2_CR1, T2IF))                                                 //溢出中断,用于判断静止,时间为349ms。
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
    Description   : CMP3：硬件比较器过流保护，关断输出，中断优先级最高
                    CMP0/1/2：顺逆风判断
    Input         : 无
    Output        : 无
-------------------------------------------------------------------------------------------------*/
//void CMP_ISR(void) interrupt 7
//{
//    if(ReadBit(CMP_SR, CMP3IF))
//    {
//        if(mcState!=mcPosiCheck)
//        {
//            FaultProcess();                                                     // 关闭输出
//            mcFaultSource=FaultHardOVCurrent;                                   // 硬件过流保护
//            mcState = mcFault;                                                  // 状态为mcFault
//        }
//        else
//        {
//            MOE     = 0;                                                                        // 关闭MOE
//            RPDPara.InsetCount[RPDPara.injecttimes]  = TIM2__CNTR;                              // 将定时器2的计数值赋值给数组
//            RPDPara.DetectCount[RPDPara.injecttimes] = RPDPara.InsetCount[RPDPara.injecttimes]; // 两组数据，一组用于观察原始数据，一组用于处理数据
//            TIM2__CNTR                               = 0;                                       // TIM2计数器值清零
//            RPDPara.injecttimes++;                                                              // RPD注入拍数累加
//        }
//        ClrBit(CMP_SR, CMP3IF);
//    }

//    #if (FRDetectMethod == BEMFMethod)
//        //通过BEMF做顺风启动功能
//        BEMFDetectFunc();
//    #endif
//}

/*---------------------------------------------------------------------------*/
/* Name     :   void TIM23_INT(void) interrupt 9
/* Input    :   NO
/* Output   :   NO
/* Description: Capture PWM ，中断优先级第二，高于FOC中断，用于PWM调速
/*---------------------------------------------------------------------------*/
void TIM3_INT(void) interrupt 9
{
    if(ReadBit(TIM3_CR1, T3IR))
    {
      ClrBit(TIM3_CR1, T3IR);
    }
    if(ReadBit(TIM3_CR1, T3IP))                                                  //周期中断
    {
       ClrBit(TIM3_CR1, T3IP);
    }
    if(ReadBit(TIM3_CR1, T3IF))
    {
       ClrBit(TIM3_CR1, T3EN);            //关闭计数器      
       ClrBit(TIM3_CR1, T3IF);
    }
}

/*---------------------------------------------------------------------------*/
/* Name     :   void TIM_1MS_INT(void) interrupt 10
/* Input    :   NO
/* Output   :   NO
/* Description: 1ms定时器中断（SYS TICK中断），用于处理附加功能，如控制环路响应、各种保护等。中断优先级低于FO中断和FOC中断。
/*---------------------------------------------------------------------------*/
//void TIM_1MS_INT(void) interrupt 10
//{
//    if(ReadBit(DRV_SR, SYSTIF))                                                  // SYS TICK中断
//    {    
//        SetBit(ADC_CR, ADCBSY);                                                  //使能ADC的DCBUS采样
//        /****功率滤波*****/
//        if(mcState == mcRun)
//        {
//            mcFocCtrl.CurrentPower = FOC__POW << 1;
//            mcFocCtrl.Powerlpf     = LPFFunction(mcFocCtrl.CurrentPower,mcFocCtrl.Powerlpf,20);      //注意低通滤波器系数范围为0---127
//        }

//        /****速度滤波、反电动势滤波*****/
//        if((mcState != mcInit) && (mcState != mcReady))
//        {
//            mcFocCtrl.SpeedFlt = LPFFunction(FOC__EOME, mcFocCtrl.SpeedFlt, 30);              //100      //注意低通滤波器系数范围为0---127
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
//        /****UQ电压值滤波****/
//        mcFocCtrl.UqFlt = LPFFunction(FOC__UQ,mcFocCtrl.UqFlt,10);              // UQ值
//        mcFocCtrl.UdFlt = LPFFunction(FOC__UD,mcFocCtrl.UdFlt,10);              // UD值

//        Motor_EN();
//        
////        KEYControl();
//        
//        Speed_response();                                                        //环路响应，如速度环、转矩环、功率环等            
//        
//        #if (StartONOFF_Enable)
//            ONOFF_Starttest(&ONOFFTest);
//        #endif

//        /*****DCbus的采样获取值并滤波******/
//        AdcSampleValue.ADCDcbus = ADC2_DR<<3;
//        mcFocCtrl.mcDcbusFlt    = LPFFunction(AdcSampleValue.ADCDcbus,mcFocCtrl.mcDcbusFlt,100);
//        
//        /***********发热丝温度采集***********/
//        AdcSampleValue.ADCTemp  = ADC1_DR<<3 ;
//        User.Temperature = LPFFunction(AdcSampleValue.ADCTemp ,User.Temperature,20);  
//       /*********控制板温度采集*********/
//        mcFocCtrl.mcADCTemperature = LPFFunction((ADC7_DR<<3),mcFocCtrl.mcADCTemperature,10);        

//        /*****故障保护函数功能，如过欠压保护、启动保护、缺相、堵转等********/
//        Fault_Detection();

////        LED_Display();//LED灯显示

//        /********睡眠模式*******/
////        Sleepmode();

//        /*****电机状态机的时序处理*****/
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

//        /*****电机启动爬坡函数处理*****/
//        StarRampDealwith();

//        ClrBit(DRV_SR, SYSTIF);                                                 // 清零标志位
//    }
//    
//    
//    
//    if(ReadBit(TIM4_CR1, T4IR))
//    {
//      ClrBit(TIM4_CR1, T4IR);
//    }
//    if(ReadBit(TIM4_CR1, T4IP))                                                  //周期中断
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
/* Description: 串口中断，中断优先级最低，用于接收调速信号,无中断插入时8us
/*---------------------------------------------------------------------------*/
//void USART_INT(void)  interrupt 12
//{
//    if(RI == 1)
//    {
//        RI = 0;
//        Uart.Uredata= UT_DR;                                                    //读接收数据
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
