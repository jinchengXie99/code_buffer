
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
  Description   : 主函数主要功能是初始化，包括上电等待，软件初始化，硬件初始化，调试模式设置，主循环扫描。
                              软件初始化--初始化所有定义的变量
                              硬件初始化--初始化硬件设备配置
                              调试模式设置--调试模式
  Input         : 无
  Output        : 无
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

  for(addr_i = 0x0030; addr_i < 0x00FF; addr_i++)   // IRAM：0x30 - 0xFF (不可 <= )
    *addr_i = 0;
    
  // 清空IRAM及XRAM，全部归零
  for(addr_x = 0x0000; addr_x <= 0x02FF; addr_x++)  // XRAM：6818: 0 - 0x0FFF , 6861: 0 - 0x02FF
    *addr_x = 0;
 
//  addr_i = 0x00FF;
//  *addr_i = 0;

  *(uint8 idata *)0x00FF = 0;
    
  //--------------------------------------------------------------------------//
  /*Software Init*/
  /*****初始化所有定义的参数变量*****/
  MotorcontrolInit();

  /*****按键初始化*****/
//  KeyInit();             //上电读取flash里面按键值

  /****电机初始状态为mcReady，故障保护为无故障******/
  mcState = mcReady;
  mcFaultSource = FaultNoSource;
    
  // 吹风筒用户Flash参数读取
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
  // 为提高芯片的抗干扰能力，降低芯片功耗，请在具体项目时，将不需要用的GPIO默认都配置为输入上拉。
  // 具体配置可在GPIO_Default_Init设置。
  //    GPIO_Default_Init();
  
  //Sleepmode_Init();

  /******硬件FO过流，比较器初始化，用于硬件过流比较保护******/
  #if (HardwareCurrent_Protect == Hardware_FO_Protect)                //外部中断初始化，用于外部中断硬件过流FO的保护
  EXTI_Init();
  #elif (HardwareCurrent_Protect == Hardware_CMP_Protect)             //选择比较过流，比较器初始化
  CMP3_Init();
  #elif (HardwareCurrent_Protect == Hardware_FO_CMP_Protect)          //两者都选择
  EXTI_Init();
  CMP3_Init();
  #endif

  /*****功能IO初始化*************/
  GPIO_Init();
  
  /*****运算放大器初始化*********/
  AMP_Init();

  /*****ADC初始化***************/
  ADC_Init();
  
  /*****比较器中断配置**********/
  CMP3_Inter_Init();                                                  // 建议和比较器初始化间隔一段时间

  /*****Driver初始化***********/
  Driver_Init();

  /***测试模式判断屏蔽档位存储***/                                     //  强制设定风速档--1，温度档--1，冷风档--0
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

  /*****过零检测外部中断初始化***/
  ZeroCrossing_Init();
  
  /*****UART初始化*************/
  #if (COMMAND_MODE == CMD_MODE_UART)
  UART_Init();                                                        // P05-TX P06-RX
  UARTDataInit();
  #endif

//  #if defined (SPI_DBG_HW) | defined (SPI_DBG_SW)                   // 调试模式
//  SPI_Init();                                                       //调试模式下，占用SPI端口的NSS(GP04),MOSI(GP05),SCK(GP06)
//  #endif
  
  // 用户参数赋值
  UserParas();
  
  /*****Timer初始化***********/
//  TIM2_Init();                                                      //端口GP07 & GP10
//  TIM3_Init();                                                      //控制发热丝
//  TIM4_Init();                                                      //检测光耦偏差
  TIM1ms_Init();                                                      //采用1ms定时器中断作为常见中断,处理故障保护等附加功能
  
  //--------------------------------------------------------------------------//
  //主循环
  while(1)
  {
    if(mcCurOffset.OffsetFlag == 0)
    {
      GetCurrentOffset();                                             // 电流偏置的获取
      
      //IBusSmpOffset = (IBusSmpOffset + ((ADC3_DR&0x0fff)<<3))>>1;

      if(mcCurOffset.OffsetFlag == 1)
      {
        mcCurOffset.OffsetCount = 0;
        IbusADCOffset = mcCurOffset.Iw_busOffset;
      }
    }
    else
    {
      // 每1ms执行一次
      if(Loop1msFlag == 1)
      {
        Loop1msFlag = 0;
        
        LoopProPer1ms();
        
        #if (COMMAND_MODE == CMD_MODE_DRYER_KEYS)
        DryerPer1ms();
        #endif
        //
        // 每10ms执行一次
        if(Loop10msFlag == 1)
        {
          Loop10msFlag = 0;
          CommandPer10ms();
        }
        
        // 由VSP的值，确定Start或Stop状态
        if(Vsp_Q12 >= VSP_ON_Q12)                 //运行
        {
          mcSpeedRamp.FlagONOFF = 1;
          mcSpeedRamp.TargetValue = 1;
        }
        else if(Vsp_Q12 < VSP_OFF_Q12)            //停机
        {
          mcSpeedRamp.FlagONOFF = 0;
          mcSpeedRamp.TargetValue = 0;
        }
        
        /* 主控函数，状态扫描 1kHz */
        MC_Control();

        //环路响应，如速度环、转矩环、功率环等
        Regulation();

        // 状态存储
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
