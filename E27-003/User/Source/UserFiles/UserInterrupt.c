
/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

#include <UserGlobal.h>
#include <UserDefine.h>

/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------*/

extern uint16 Power_Currt;
extern u8  Loop1msFlag;
extern u8  idata AlarmHardBeark;
extern u8  idata VZeroCrossGetFlag;
extern u8  idata ACCrossForThryrist;
extern u8  UTXMark;
extern bit Heat_wire_H;
extern bit Heat_wire_L;

/*-------------------------------------------------------------------------------------------------
    Function Name : void FO_INT(void)
    Description   : FO_INT interrupt，硬件FO过流保护，关断输出，中断优先级最高
    Input         : 无
    Output        : 无
-------------------------------------------------------------------------------------------------*/
void FO_INT(void) interrupt 1                                                   // 硬件FO过流中断，关闭输出
{
  ShutPerform();
  SetFaultResatrt();
  AlarmHardBeark = 1;
}
/*---------------------------------------------------------------------------*/
/* Name       :   void EXTERN_INT(void) interrupt 2
/* Input      :   NO
/* Output     :   NO
/* Description: 过零点检测
/*---------------------------------------------------------------------------*/
void EXTERN_INT(void) interrupt 2
{
  IF0 = 0;
  VZeroCrossGetFlag = 1;
  ACCrossForThryrist = 1;
  ClrBit(P2_IF, P20);                                                          // 清零P20标志位
  GP11 = Heat_wire_H;
  GP10 = Heat_wire_L;
}
/*---------------------------------------------------------------------------*/
/* Name     :   void FOC_INT(void) interrupt 3
/* Input    :   NO
/* Output   :   NO
/* Description: FOC中断(Drv中断),每个载波周期执行一次，用于处理响应较高的程序，中断优先级第二。DCEN开了就会产生中断。
/*---------------------------------------------------------------------------*/
void FOC_INT(void) interrupt 3
{
  if(ReadBit(DRV_SR, DCIF))                                                 // 比较中断
  {
    APP_DIV();                                                              //启动除法器，避免与过调值中的除法冲突
    CheckCurrent();
    ClrBit(DRV_SR, DCIF);
  }
}
/*-------------------------------------------------------------------------------------------------
    Function Name : void CMP_ISR(void)
    Description   : CMP3：硬件比较器过流保护，关断输出，中断优先级最高
                    CMP0/1/2：顺逆风判断
    Input         : 无
    Output        : 无
-------------------------------------------------------------------------------------------------*/
void CMP_ISR(void) interrupt 7
{
  if(ReadBit(CMP_SR, CMP3IF))
  { 
    if(mcState!=mcPosiCheck)
    {
//      FaultProcess();                                                     // 关闭输出
//      mcFaultSource=FaultHardOVCurrent;                                   // 硬件过流保护
//      mcState = mcFault;                                                  // 状态为mcFault
      ShutPerform();
      SetFaultResatrt();
      AlarmHardBeark = 1;
    }
    else
    {
      MOE     = 0;                                                                        // 关闭MOE
      RPDPara.InsetCount[RPDPara.injecttimes]  = TIM2__CNTR;                              // 将定时器2的计数值赋值给数组
      RPDPara.DetectCount[RPDPara.injecttimes] = RPDPara.InsetCount[RPDPara.injecttimes]; // 两组数据，一组用于观察原始数据，一组用于处理数据
      TIM2__CNTR                               = 0;                                       // TIM2计数器值清零
      RPDPara.injecttimes++;                                                              // RPD注入拍数累加
    }
    ClrBit(CMP_SR, CMP3IF);
  }

  #if (FRDetectMethod == BEMFMethod)
    //通过BEMF做顺风启动功能
    BEMFDetectFunc();
  #endif
}
/*---------------------------------------------------------------------------*/
/* Name     :   void TIM_1MS_INT(void) interrupt 10
/* Input    :   NO
/* Output   :   NO
/* Description: 1ms定时器中断（SYS TICK中断），用于处理附加功能，如控制环路响应、各种保护等。中断优先级低于FO中断和FOC中断。
/*---------------------------------------------------------------------------*/
void TIM_1MS_INT(void) interrupt 10
{
  if(ReadBit(DRV_SR, SYSTIF))                                                 // SYS TICK中断
  {
    SetBit(ADC_CR, ADCBSY);                                                   //使能ADC的DCBUS采样
  
    Loop1msFlag = 1;
    
    /****功率滤波*****/
    if(mcState == mcRun)
    {
      mcFocCtrl.CurrentPower = FOC__POW << 1;
      mcFocCtrl.Powerlpf     = LPFFunction(mcFocCtrl.CurrentPower,mcFocCtrl.Powerlpf,20); //注意低通滤波器系数范围为0---127
    }

    /****速度滤波、反电动势滤波*****/
    if((mcState != mcInit) && (mcState != mcReady))
    {
      mcFocCtrl.SpeedFlt = LPFFunction(FOC__EOME, mcFocCtrl.SpeedFlt, 30); //100          //注意低通滤波器系数范围为0---127
      mcFocCtrl.EsValue  = LPFFunction(FOC__ESQU,mcFocCtrl.EsValue,10);
    }
    else
    {
      mcFocCtrl.SpeedFlt = 0;
    }
    
    /****UQ电压值滤波****/
    mcFocCtrl.UqFlt = LPFFunction(FOC__UQ,mcFocCtrl.UqFlt,10);              // UQ值
    mcFocCtrl.UdFlt = LPFFunction(FOC__UD,mcFocCtrl.UdFlt,10);              // UD值
       
    // 母线电流滤波
    Power_Currt = LPFFunction((ADC3_DR<<3),Power_Currt,50);
    
    // DCbus的采样获取值并滤波
    AdcSampleValue.ADCDcbus = ADC2_DR<<3;
    mcFocCtrl.mcDcbusFlt    = LPFFunction(AdcSampleValue.ADCDcbus,mcFocCtrl.mcDcbusFlt,60);
    
    // 发热丝温度采集
    AdcSampleValue.ADCTemp  = ADC1_DR<<3;
    User.Temperature        = LPFFunction(AdcSampleValue.ADCTemp,User.Temperature,20);
    
    // 控制板温度采集
    mcFocCtrl.mcADCTemperature = LPFFunction((ADC7_DR<<3),mcFocCtrl.mcADCTemperature,10);

    /*****电机状态机的时序处理*****/
    if(BEMFDetect.BEMFTimeCount > 0)        BEMFDetect.BEMFTimeCount--;
    if(RSDDetect.RSDCCWSBRCnt > 0)          RSDDetect.RSDCCWSBRCnt--;
    if(mcFaultDect.VoltDetecExternCnt > 0)  mcFaultDect.VoltDetecExternCnt--;
    
    if(mcFocCtrl.State_Count > 0)
    {
      mcFocCtrl.State_Count--;
      StarRampDealwith();                   // 电机启动爬坡函数处理
    }
  
    if(mcState == mcRun)
    {
      if(mcFocCtrl.TPCtrlDealy > 0)
        mcFocCtrl.TPCtrlDealy--;
    }

    #if (FRDetectMethod==FOCMethod)
      FOCTailWindTimeLimit();
    #endif
   
    ClrBit(DRV_SR, SYSTIF);                                                   // 清零标志位
  }
    
  //if(ReadBit(TIM4_CR1, T4IR))
  {
    ClrBit(TIM4_CR1, T4IR);
  }
  //if(ReadBit(TIM4_CR1, T4IP))                                                 //周期中断
  {
    ClrBit(TIM4_CR1, T4IP);
  }
  //if(ReadBit(TIM4_CR1, T4IF))
  {
    ClrBit(TIM4_CR1, T4IF);
  }
}
/*---------------------------------------------------------------------------*/
/* Name     :   void USART_INT(void) interrupt 12
/* Input    :   NO
/* Output   :   NO
/* Description: 串口中断，中断优先级最低，用于接收调速信号,无中断插入时8us
/*---------------------------------------------------------------------------*/

void USART_INT(void)  interrupt 12
{
  if(RI == 1)                       //接收中断
  {
    #if (COMMAND_MODE == CMD_MODE_UART)
    UARTGetData();
    #endif
    RI = 0;
  }
  
  if(TI == 1)                       //发送中断
  {
    #if (COMMAND_MODE == CMD_MODE_UART)
    UTXMark = 1;
    #endif
    TI = 0;                         //发送完成中断标志位清零
  }
}
