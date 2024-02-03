/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : main.c
* Author             : Fortiortech Appliction Team
* Version            : V1.0
* Date               : 2017-12-27
* Description        : This file contains main function used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/

/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------*/
void SoftwareInit(void);
void HardwareInit(void);
void DebugSet(void);
void MotorControlInit(void);
extern uint16 KeyCount_Long;
/*-------------------------------------------------------------------------------------------------
        Function Name : void main(void)
        Description   : ��������Ҫ�����ǳ�ʼ���������ϵ�ȴ��������ʼ����Ӳ����ʼ��������ģʽ���ã���ѭ��ɨ�衣
                                    �����ʼ��--��ʼ�����ж���ı���
                                    Ӳ����ʼ��--��ʼ��Ӳ���豸����
                                    ����ģʽ����--����ģʽ
        Input         : ��
        Output        : ��
-------------------------------------------------------------------------------------------------*/
//void main(void)
//{
//    uint16 PowerUpCnt = 0;

//    for(PowerUpCnt=0;PowerUpCnt<SystemPowerUpTime;PowerUpCnt++){};

//    /*Software Init*/
//    SoftwareInit();

//    /*Hardware Init*/
//    HardwareInit();

////    /*����ģʽ����--�ڲ�������ѯ��CMP�����ѯ��ADC�����źŲ�ѯ*/
////    DebugSet();

//      /*��ͣ���Լ�*/
////    ONOFFTest.ONOFF_Flag = 1;
////    mcSpeedRamp.FlagONOFF   = 1;
////    mcSpeedRamp.TargetValue = _Q15(110000 / MOTOR_SPEED_BASE);
//  
//    while(1)
//    {
//      GetCurrentOffset();                                                      // ����ƫ�õĻ�ȡ
//      MC_Control();                                                            // ���غ�����״̬ɨ�� 2.6k
//      RealPowerCal();           // ���ʼ���
//      #if (!StartONOFF_Enable)                                                 // ���ٷ�ʽ
//          #if (SPEED_MODE == PWMMODE)                                          // PWM����ģʽ
//                PWMInputCapture();                                            // PWM duty����ģʽ
//          #elif (SPEED_MODE == NONEMODE)                                      // ֱ���ϵ�����ģʽ
//              mcSpeedRamp.FlagONOFF   = 1;
////              mcSpeedRamp.TargetValue = _Q15(69000 / MOTOR_SPEED_BASE);
//          #endif
//      #endif
//    }
//}
/*-------------------------------------------------------------------------------------------------
        Function Name : void DebugSet(void)
        Description   : ����ģʽ����
        Input         : ��
        Output        : ��
-------------------------------------------------------------------------------------------------*/
void DebugSet(void)
{
//    #if defined (SPI_DBG_HW)                                                    // Ӳ������ģʽ
//        Set_DBG_DMA(&HARD_SPIDATA);
//    #elif defined (SPI_DBG_SW)                                                  // �������ģʽ
//        Set_DBG_DMA(spidebug);
//    #endif

//    #if defined (SPI_DBG_HW) && defined (SPI_DBG_SW)
//        #error Only one DBG mode can be selected
//    #endif

//    SetReg(CMP_CR3, DBGSEL0 | DBGSEL1,  GP01_DBG_Conf);
//    SetReg(CMP_CR3, CMPSEL0 | CMPSEL1 | CMPSEL2, GP07_DBG_Conf);
}

/*-------------------------------------------------------------------------------------------------
        Function Name : void SoftwareInit(void)
        Description   : �����ʼ������ʼ�����ж��������������ʼ��ɨ��
        Input         : ��
        Output        : ��
-------------------------------------------------------------------------------------------------*/
void SoftwareInit(void)
{
    /****��ʼ�����ж���Ĳ�������****/
    MotorcontrolInit();
  
   /******������ʼ��******/
    KeyInit();             //�ϵ��ȡflash���水��ֵ
  
    /****�����ʼ״̬ΪmcReady�����ϱ���Ϊ�޹���******/
    mcState = mcReady;
    mcFaultSource = FaultNoSource;
    KeyCount_Long = 0;
}

/*-------------------------------------------------------------------------------------------------
        Function Name : void HardwareInit(void)
        Description   : Ӳ����ʼ������ʼ����Ҫʹ�õ�Ӳ���豸���ã�FOC�������õ����˷ŵ�ѹ���˷ų�ʼ����ADC��ʼ����Driver��ʼ��
                                    TIM4��ʼ���������Ŀɸ���ʵ������ӡ�
        Input         : ��
        Output        : ��
-------------------------------------------------------------------------------------------------*/
void HardwareInit(void)
{
  // Ϊ���оƬ�Ŀ���������������оƬ���ģ����ھ�����Ŀʱ��������Ҫ�õ�GPIOĬ�϶�����Ϊ����������
  // �������ÿ���GPIO_Default_Init���á�
  //    GPIO_Default_Init();

  /******Ӳ��FO�������Ƚ�����ʼ��������Ӳ�������Ƚϱ���******/
  #if (HardwareCurrent_Protect == Hardware_FO_Protect)                        //�ⲿ�жϳ�ʼ���������ⲿ�ж�Ӳ������FO�ı���
      EXTI_Init();
  #elif (HardwareCurrent_Protect == Hardware_CMP_Protect)                     //ѡ��ȽϹ������Ƚ�����ʼ��
      CMP3_Init();
  #elif (HardwareCurrent_Protect == Hardware_FO_CMP_Protect)                  //���߶�ѡ��
      EXTI_Init();
      CMP3_Init();
  #endif

  //Sleepmode_Init();
 
  /*****����IO��ʼ��*****/
  GPIO_Init();
  
  /*****�������ⲿ�жϳ�ʼ��*****/
  ZeroCrossing_Init();

  /*****����Ŵ�����ʼ��*****/
  AMP_Init();

  /*****ADC��ʼ��*****/
  ADC_Init();
  
  /*****�Ƚ����ж�����*****/
  CMP3_Inter_Init();                                                          // ����ͱȽ�����ʼ�����һ��ʱ��

  /*****Driver��ʼ��*****/
  Driver_Init();
    
    /*****UART��ʼ��*****/
//  UART_Init();//δ����

//  #if defined (SPI_DBG_HW) | defined (SPI_DBG_SW)                           // ����ģʽ
//    SPI_Init();                                                             //����ģʽ�£�ռ��SPI�˿ڵ�NSS(GP04),MOSI(GP05),SCK(GP06)
//  #endif

    /*****Timer��ʼ��*****/
//  TIM2_Init();                                                              //�˿�GP07 & GP10
//  TIM3_Init();                                                              //���Ʒ���˿
//  TIM4_Init();                                                              //������ƫ��
  TIM1ms_Init();                                                              //����1ms��ʱ���ж���Ϊ�����ж�,������ϱ����ȸ��ӹ���
}
