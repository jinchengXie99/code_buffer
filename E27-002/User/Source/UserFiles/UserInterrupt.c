
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
    Description   : FO_INT interrupt��Ӳ��FO�����������ض�������ж����ȼ����
    Input         : ��
    Output        : ��
-------------------------------------------------------------------------------------------------*/
void FO_INT(void) interrupt 1                                                   // Ӳ��FO�����жϣ��ر����
{
  ShutPerform();
  SetFaultResatrt();
  AlarmHardBeark = 1;
}
/*---------------------------------------------------------------------------*/
/* Name       :   void EXTERN_INT(void) interrupt 2
/* Input      :   NO
/* Output     :   NO
/* Description: �������
/*---------------------------------------------------------------------------*/
void EXTERN_INT(void) interrupt 2
{
  IF0 = 0;
  VZeroCrossGetFlag = 1;
  ACCrossForThryrist = 1;
  ClrBit(P2_IF, P20);                                                          // ����P20��־λ
  GP11 = Heat_wire_H;
  GP10 = Heat_wire_L;
}
/*---------------------------------------------------------------------------*/
/* Name     :   void FOC_INT(void) interrupt 3
/* Input    :   NO
/* Output   :   NO
/* Description: FOC�ж�(Drv�ж�),ÿ���ز�����ִ��һ�Σ����ڴ�����Ӧ�ϸߵĳ����ж����ȼ��ڶ���DCEN���˾ͻ�����жϡ�
/*---------------------------------------------------------------------------*/
void FOC_INT(void) interrupt 3
{
  if(ReadBit(DRV_SR, DCIF))                                                 // �Ƚ��ж�
  {
    APP_DIV();                                                              //���������������������ֵ�еĳ�����ͻ
    CheckCurrent();
    ClrBit(DRV_SR, DCIF);
  }
}
/*-------------------------------------------------------------------------------------------------
    Function Name : void CMP_ISR(void)
    Description   : CMP3��Ӳ���Ƚ��������������ض�������ж����ȼ����
                    CMP0/1/2��˳����ж�
    Input         : ��
    Output        : ��
-------------------------------------------------------------------------------------------------*/
void CMP_ISR(void) interrupt 7
{
  if(ReadBit(CMP_SR, CMP3IF))
  { 
    if(mcState!=mcPosiCheck)
    {
//      FaultProcess();                                                     // �ر����
//      mcFaultSource=FaultHardOVCurrent;                                   // Ӳ����������
//      mcState = mcFault;                                                  // ״̬ΪmcFault
      ShutPerform();
      SetFaultResatrt();
      AlarmHardBeark = 1;
    }
    else
    {
      MOE     = 0;                                                                        // �ر�MOE
      RPDPara.InsetCount[RPDPara.injecttimes]  = TIM2__CNTR;                              // ����ʱ��2�ļ���ֵ��ֵ������
      RPDPara.DetectCount[RPDPara.injecttimes] = RPDPara.InsetCount[RPDPara.injecttimes]; // �������ݣ�һ�����ڹ۲�ԭʼ���ݣ�һ�����ڴ�������
      TIM2__CNTR                               = 0;                                       // TIM2������ֵ����
      RPDPara.injecttimes++;                                                              // RPDע�������ۼ�
    }
    ClrBit(CMP_SR, CMP3IF);
  }

  #if (FRDetectMethod == BEMFMethod)
    //ͨ��BEMF��˳����������
    BEMFDetectFunc();
  #endif
}
/*---------------------------------------------------------------------------*/
/* Name     :   void TIM_1MS_INT(void) interrupt 10
/* Input    :   NO
/* Output   :   NO
/* Description: 1ms��ʱ���жϣ�SYS TICK�жϣ������ڴ����ӹ��ܣ�����ƻ�·��Ӧ�����ֱ����ȡ��ж����ȼ�����FO�жϺ�FOC�жϡ�
/*---------------------------------------------------------------------------*/
void TIM_1MS_INT(void) interrupt 10
{
  if(ReadBit(DRV_SR, SYSTIF))                                                 // SYS TICK�ж�
  {
    SetBit(ADC_CR, ADCBSY);                                                   //ʹ��ADC��DCBUS����
  
    Loop1msFlag = 1;
    
    /****�����˲�*****/
    if(mcState == mcRun)
    {
      mcFocCtrl.CurrentPower = FOC__POW << 1;
      mcFocCtrl.Powerlpf     = LPFFunction(mcFocCtrl.CurrentPower,mcFocCtrl.Powerlpf,20); //ע���ͨ�˲���ϵ����ΧΪ0---127
    }

    /****�ٶ��˲������綯���˲�*****/
    if((mcState != mcInit) && (mcState != mcReady))
    {
      mcFocCtrl.SpeedFlt = LPFFunction(FOC__EOME, mcFocCtrl.SpeedFlt, 30); //100          //ע���ͨ�˲���ϵ����ΧΪ0---127
      mcFocCtrl.EsValue  = LPFFunction(FOC__ESQU,mcFocCtrl.EsValue,10);
    }
    else
    {
      mcFocCtrl.SpeedFlt = 0;
    }
    
    /****UQ��ѹֵ�˲�****/
    mcFocCtrl.UqFlt = LPFFunction(FOC__UQ,mcFocCtrl.UqFlt,10);              // UQֵ
    mcFocCtrl.UdFlt = LPFFunction(FOC__UD,mcFocCtrl.UdFlt,10);              // UDֵ
       
    // ĸ�ߵ����˲�
    Power_Currt = LPFFunction((ADC3_DR<<3),Power_Currt,50);
    
    // DCbus�Ĳ�����ȡֵ���˲�
    AdcSampleValue.ADCDcbus = ADC2_DR<<3;
    mcFocCtrl.mcDcbusFlt    = LPFFunction(AdcSampleValue.ADCDcbus,mcFocCtrl.mcDcbusFlt,60);
    
    // ����˿�¶Ȳɼ�
    AdcSampleValue.ADCTemp  = ADC1_DR<<3;
    User.Temperature        = LPFFunction(AdcSampleValue.ADCTemp,User.Temperature,20);
    
    // ���ư��¶Ȳɼ�
    mcFocCtrl.mcADCTemperature = LPFFunction((ADC7_DR<<3),mcFocCtrl.mcADCTemperature,10);

    /*****���״̬����ʱ����*****/
    if(BEMFDetect.BEMFTimeCount > 0)        BEMFDetect.BEMFTimeCount--;
    if(RSDDetect.RSDCCWSBRCnt > 0)          RSDDetect.RSDCCWSBRCnt--;
    if(mcFaultDect.VoltDetecExternCnt > 0)  mcFaultDect.VoltDetecExternCnt--;
    
    if(mcFocCtrl.State_Count > 0)
    {
      mcFocCtrl.State_Count--;
      StarRampDealwith();                   // ����������º�������
    }
  
    if(mcState == mcRun)
    {
      if(mcFocCtrl.TPCtrlDealy > 0)
        mcFocCtrl.TPCtrlDealy--;
    }

    #if (FRDetectMethod==FOCMethod)
      FOCTailWindTimeLimit();
    #endif
   
    ClrBit(DRV_SR, SYSTIF);                                                   // �����־λ
  }
    
  //if(ReadBit(TIM4_CR1, T4IR))
  {
    ClrBit(TIM4_CR1, T4IR);
  }
  //if(ReadBit(TIM4_CR1, T4IP))                                                 //�����ж�
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
/* Description: �����жϣ��ж����ȼ���ͣ����ڽ��յ����ź�,���жϲ���ʱ8us
/*---------------------------------------------------------------------------*/

void USART_INT(void)  interrupt 12
{
  if(RI == 1)                       //�����ж�
  {
    #if (COMMAND_MODE == CMD_MODE_UART)
    UARTGetData();
    #endif
    RI = 0;
  }
  
  if(TI == 1)                       //�����ж�
  {
    #if (COMMAND_MODE == CMD_MODE_UART)
    UTXMark = 1;
    #endif
    TI = 0;                         //��������жϱ�־λ����
  }
}
