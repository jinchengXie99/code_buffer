
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

extern u8  Loop1msFlag;
extern u8  Loop10msFlag;
extern u16 IbusADCOffset;
extern s16 idata Vsp_Q12;

extern u8  idata SpeedShiftStep;
extern u8  idata SpeedCmdLv;
extern u8  idata HeatShiftStep;
extern u8  idata HeatCmdLv;
extern u8  idata CoolShiftLv;
extern u8  idata CoolCmdLv;

bit TestMode = 0;

uint16 PowerUpCnt;
/*-------------------------------------------------------------------------------------------------
  Function Name : void main(void)
  Description   : ��������Ҫ�����ǳ�ʼ���������ϵ�ȴ��������ʼ����Ӳ����ʼ��������ģʽ���ã���ѭ��ɨ�衣
                              �����ʼ��--��ʼ�����ж���ı���
                              Ӳ����ʼ��--��ʼ��Ӳ���豸����
                              ����ģʽ����--����ģʽ
  Input         : ��
  Output        : ��
-------------------------------------------------------------------------------------------------*/
void main(void)
{

  u8  xdata *addr_x;
  u8  idata *addr_i;

  /*Wait For Power Up*/
//  for(PowerUpCnt=0;PowerUpCnt<SystemPowerUpTime;PowerUpCnt++)
//  {}
  
  PowerUpCnt = 0;
  while(PowerUpCnt++ < SystemPowerUpTime)
  {;}

  for(addr_i = 0x0030; addr_i < 0x00FF; addr_i++)   // IRAM��0x30 - 0xFF (���� <= )
    *addr_i = 0;
    
  // ���IRAM��XRAM��ȫ������
  for(addr_x = 0x0000; addr_x <= 0x02FF; addr_x++)  // XRAM��6818: 0 - 0x0FFF , 6861: 0 - 0x02FF
    *addr_x = 0;
 
//  addr_i = 0x00FF;
//  *addr_i = 0;

  *(uint8 idata *)0x00FF = 0;
    
  //--------------------------------------------------------------------------//
  /*Software Init*/
  /*****��ʼ�����ж���Ĳ�������*****/
  MotorcontrolInit();

  /*****������ʼ��*****/
//  KeyInit();             //�ϵ��ȡflash���水��ֵ

  /****�����ʼ״̬ΪmcReady�����ϱ���Ϊ�޹���******/
  mcState = mcReady;
  mcFaultSource = FaultNoSource;
    
  // ����Ͳ�û�Flash������ȡ
  #if (COMMAND_MODE == CMD_MODE_DRYER_KEYS)
    #if (FLASH_SAVE_ENABLE == 0)
      DryerKeysDefault();
    #else
      DryerFlashInit();
    #endif
    
    CalSpeedStepToCmd();
    CalHeatStepToCmd();
    CalCoolLvToCmd();
  #endif
  
  //--------------------------------------------------------------------------//
  /*Hardware Init*/
  // Ϊ���оƬ�Ŀ���������������оƬ���ģ����ھ�����Ŀʱ��������Ҫ�õ�GPIOĬ�϶�����Ϊ����������
  // �������ÿ���GPIO_Default_Init���á�
  //    GPIO_Default_Init();
  
  //Sleepmode_Init();

  /******Ӳ��FO�������Ƚ�����ʼ��������Ӳ�������Ƚϱ���******/
  #if (HardwareCurrent_Protect == Hardware_FO_Protect)                //�ⲿ�жϳ�ʼ���������ⲿ�ж�Ӳ������FO�ı���
  EXTI_Init();
  #elif (HardwareCurrent_Protect == Hardware_CMP_Protect)             //ѡ��ȽϹ������Ƚ�����ʼ��
  CMP3_Init();
  #elif (HardwareCurrent_Protect == Hardware_FO_CMP_Protect)          //���߶�ѡ��
  EXTI_Init();
  CMP3_Init();
  #endif

  /*****����IO��ʼ��*************/
  GPIO_Init();
  
  /*****����Ŵ�����ʼ��*********/
  AMP_Init();

  /*****ADC��ʼ��***************/
  ADC_Init();
  
  /*****�Ƚ����ж�����**********/
  CMP3_Inter_Init();                                                  // ����ͱȽ�����ʼ�����һ��ʱ��

  /*****Driver��ʼ��***********/
  Driver_Init();

  /***����ģʽ�ж����ε�λ�洢***/                                     //  ǿ���趨���ٵ�--1���¶ȵ�--1����絵--0
  if( (GP15 == 0)  &&  (GP13 == 0) )
  {
    TestMode = 1;
    
    SpeedShiftStep = 1;
    SpeedCmdLv = 1;
    
    HeatShiftStep = 1;
    HeatCmdLv = 1;
    
    CoolShiftLv = 0;
    CoolCmdLv = 0;
  }

  /*****�������ⲿ�жϳ�ʼ��***/
  ZeroCrossing_Init();
  
  /*****UART��ʼ��*************/
  #if (COMMAND_MODE == CMD_MODE_UART)
  UART_Init();                                                        // P05-TX P06-RX
  UARTDataInit();
  #endif

//  #if defined (SPI_DBG_HW) | defined (SPI_DBG_SW)                   // ����ģʽ
//  SPI_Init();                                                       //����ģʽ�£�ռ��SPI�˿ڵ�NSS(GP04),MOSI(GP05),SCK(GP06)
//  #endif
  
  // �û�������ֵ
  UserParas();
  
  /*****Timer��ʼ��***********/
//  TIM2_Init();                                                      //�˿�GP07 & GP10
//  TIM3_Init();                                                      //���Ʒ���˿
//  TIM4_Init();                                                      //������ƫ��
  TIM1ms_Init();                                                      //����1ms��ʱ���ж���Ϊ�����ж�,������ϱ����ȸ��ӹ���
  
  //--------------------------------------------------------------------------//
  //��ѭ��
  while(1)
  {
    if(mcCurOffset.OffsetFlag == 0)
    {
      GetCurrentOffset();                                             // ����ƫ�õĻ�ȡ
      
      //IBusSmpOffset = (IBusSmpOffset + ((ADC3_DR&0x0fff)<<3))>>1;

      if(mcCurOffset.OffsetFlag == 1)
      {
        mcCurOffset.OffsetCount = 0;
        IbusADCOffset = mcCurOffset.Iw_busOffset;
      }
    }
    else
    {
      // ÿ1msִ��һ��
      if(Loop1msFlag == 1)
      {
        Loop1msFlag = 0;
        
        LoopProPer1ms();
        
        #if (COMMAND_MODE == CMD_MODE_DRYER_KEYS)
        DryerPer1ms();
        #endif
        //
        // ÿ10msִ��һ��
        if(Loop10msFlag == 1)
        {
          Loop10msFlag = 0;
          CommandPer10ms();
        }
        
        // ��VSP��ֵ��ȷ��Start��Stop״̬
        if(Vsp_Q12 >= VSP_ON_Q12)                 //����
        {
          mcSpeedRamp.FlagONOFF = 1;
          mcSpeedRamp.TargetValue = 1;
        }
        else if(Vsp_Q12 < VSP_OFF_Q12)            //ͣ��
        {
          mcSpeedRamp.FlagONOFF = 0;
          mcSpeedRamp.TargetValue = 0;
        }
        
        /* ���غ�����״̬ɨ�� 1kHz */
        MC_Control();

        //��·��Ӧ�����ٶȻ���ת�ػ������ʻ���
        Regulation();

        // ״̬�洢
        #if ((FLASH_SAVE_ENABLE == 1)&&(COMMAND_MODE == CMD_MODE_DRYER_KEYS))
          FlashLoopPro();
        #endif
        
        #if (COMMAND_MODE == CMD_MODE_UART)
        UARTPer1ms();
        #endif
      }
    }
  }
}
