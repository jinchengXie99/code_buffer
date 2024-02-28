/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： hardware_init.c
 * 文件标识：
 * 内容摘要： 硬件初始化代码
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet Li
 * 完成日期： 2015年11月5日
 *
 * 修改记录1：
 * 修改日期：
 * 版 本 号：
 * 修 改 人：
 * 修改内容：
 *
 *******************************************************************************/
#include "basic.h"
#include "hardware_config.h"
#include "global_variable.h"

void SoftDelay(u32 cnt);
void UART_init(void);
void HALL_Perip_Init(void);
void TempSensor_Init(void);

/*******************************************************************************
 函数名称：    void Hardware_init(void)
 功能描述：    硬件部分初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void Hardware_init(void)
{
  __disable_irq();                  /* 关闭中断 中断总开关 */
  SYS_WR_PROTECT = 0x7a83;

  IWDG_PSW = 0xA6B4;
  IWDG_CFG = 0x3C00;

  FLASH_CFG |= 0x00080000;          /* enable prefetch */

//  DSP_init();                       /* DSP模块初始化*/

  SYS_ModuleClockCmd(SYS_Module_DIV, ENABLE);

//  UART_init();                      /* 串口初始化UART0*/
  ADC_init();                      /* ADC初始化 */
  MCPWM_init();                     /* PWM初始化 */
  UTimer_init();                    /* 通用计数器初始化 */
  GPIO_init();                      /* GPIO初始化 */
  DAC_Init();                       /* DAC 初始化 */
  PGA_Init();                       /* PGA 初始化 */
  CMP_init();                       /* 比较器初始化 */
  HALL_Perip_Init();                /* HALL模块初始化 */
  TempSensor_Init();                /* 温度传感器初始化 */

  SoftDelay(100);                   /* 延时等待硬件初始化稳定 */


//  NVIC_SetPriority(UART_IRQn, 2);  /* 设置UART0中断优先级为2 | 共0，1，2，3四级中断优先级，0为最高*/
  NVIC_SetPriority(ADC_IRQn, 1);
  NVIC_SetPriority(HALL_IRQn, 2);
  NVIC_SetPriority(MCPWM0_IRQn, 2);
  NVIC_SetPriority(CMP_IRQn, 0);    /* 设置CMP_IRQn中断优先级为0 | 共0，1，2，3四级中断优先级，0为最高*/
  NVIC_SetPriority(TIMER0_IRQn, 2);
  NVIC_SetPriority(TIMER1_IRQn, 2);
    
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
  NVIC_SetPriority(MCPWM_IRQn, 1);
  NVIC_EnableIRQ (MCPWM_IRQn);
#else
  NVIC_EnableIRQ(ADC_IRQn);        /* enable the ADC0 interrupt */
#endif
  NVIC_EnableIRQ(MCPWM0_IRQn);
 // NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);

  NVIC_EnableIRQ(CMP_IRQn);         /* 打开比较器中断 */

  __enable_irq();                   /* 开启总中断 */
}

/*******************************************************************************
 函数名称：    void Clock_Init(void)
 功能描述：    时钟配置
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void Clock_Init(void)
{

  SYS_WR_PROTECT = 0x7a83;   /* 解除系统寄存器写保护 */

  SYS_AFE_REG0 |= BIT15;     /* BIT15:PLLPDN */
  SYS_CLK_CFG |= 0x000001ff; /* BIT8:0: CLK_HS,1:PLL  | BIT[7:0]CLK_DIV  | 1ff对应48M时钟 */

//    SYS_CLK_FEN = 0xfff;

}

/*******************************************************************************
 函数名称：    void SystemInit(void)
 功能描述：    硬件系统初始化，调用时钟初始化函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2016/3/14      V1.0           Howlet Li          创建
 *******************************************************************************/
void SystemInit(void)
{
  Clock_Init();  /* 时钟初始化 */
}

/*******************************************************************************
 函数名称：    void TempSensor_Init(void)
 功能描述：    温度传感器初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void TempSensor_Init(void)
{

  SYS_WR_PROTECT = 0x7a83;   /* 解除系统寄存器写保护 */

  SYS_AnalogModuleClockCmd(SYS_AnalogModule_TMP,ENABLE);    /* 打开温度传感器开关 */

//    m_TempertureCof.nCofA    = Read_Trim(0x00000398);
//    m_TempertureCof.nOffsetB = Read_Trim(0x0000039C);

}

/*******************************************************************************
 函数名称：    void PGA_Init(void)
 功能描述：    PGA初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2016/3/15      V1.0           Howlet Li          创建
 *******************************************************************************/
void PGA_Init(void)
{

  SYS_AnalogModuleClockCmd(SYS_AnalogModule_OPA, ENABLE);

  SYS_WR_PROTECT = 0x7a83;

  SYS_AFE_REG0 &= ~3;
  SYS_AFE_REG0 |= OPA0_GIAN; /* OPA增益设置 */
}

/*******************************************************************************
 函数名称：    void CMP_Init(void)
 功能描述：    比较器初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2016/3/15      V1.0           Howlet Li          创建
 *******************************************************************************/
void CMP_init(void)
{
  CMP_InitTypeDef CMP_InitStruct;
  CMP_StructInit(&CMP_InitStruct);
	
	SYS_AnalogModuleClockCmd( SYS_AnalogModule_CMP0,DISABLE); 	/* 比较器0开关 */
	SYS_AnalogModuleClockCmd( SYS_AnalogModule_CMP1,ENABLE);  /* 比较器1开关 */
	
  CMP_InitStruct.FIL_CLK10_DIV16 = 10;   // 比较器 1/0 滤波
  CMP_InitStruct.FIL_CLK10_DIV2 = 2;     // 比较器 1/0 滤波时钟分频
	
  // 比较器1
  CMP_InitStruct.CMP1_IE = ENABLE;      // 比较器 1 中断使能
  CMP_InitStruct.CMP1_IN_EN = ENABLE;   // 比较器 1 信号输入使能
  CMP_InitStruct.CMP1_POL = 0;           // 比较器 1 极性选择，0:高电平有效；1:低电平有效
    
  CMP_InitStruct.CMP1_SELN = CMP1_SELN_DAC;
  CMP_InitStruct.CMP1_SELP = CMP1_SELP_CMP1_IP1;
    
  // 比较器0
  CMP_InitStruct.CMP0_IE = DISABLE;            // 比较器 0 中断使能
  CMP_InitStruct.CMP0_IN_EN = DISABLE;         // 比较器 0 信号输入使能
  CMP_InitStruct.CMP0_POL = 0;                // 比较器 0 极性选择，0:高电平有效；1:低电平有效


  CMP_InitStruct.CMP0_SELN = CMP0_SELN_DAC;
  CMP_InitStruct.CMP0_SELP = CMP0_SELP_CMP0_IP3;
	
	CMP_InitStruct.CMP_HYS = CMP_HYS_0mV;

  CMP_Init(&CMP_InitStruct);
}

/*******************************************************************************
 函数名称：    void DAC_Init(void)
 功能描述：    DAC初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2016/3/23      V1.0           Howlet Li          创建
 *******************************************************************************/
void DAC_Init(void)
{
  SYS_AnalogModuleClockCmd(SYS_AnalogModule_DAC, ENABLE);

  SYS_AFE_DAC = 50;                     /* 3*25/256/0.1 =3.0A, 其中0.1为母线采样电阻*/
	
}

/*******************************************************************************
 函数名称：    void UART_init(void)
 功能描述：    UART0寄存器配置
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void UART_init(void)
{
  UART_InitTypeDef UART_InitStruct;

  UART_StructInit(&UART_InitStruct);
  UART_InitStruct.BaudRate = 38400;                 /* 设置波特率38400 */
  UART_InitStruct.WordLength = UART_WORDLENGTH_8b;  /* 发送数据长度8位 */
  UART_InitStruct.StopBits = UART_STOPBITS_1b;
  UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB;   /* 先发送LSB */
  UART_InitStruct.ParityMode = UART_Parity_NO;      /* 无奇偶校验 */
  UART_InitStruct.IRQEna = 0;
  UART_Init(UART0, &UART_InitStruct);
}

/*******************************************************************************
 函数名称：    void UART_SENDDATA(void)
 功能描述：    UART0发送程序
 输入参数：    n：需要发送的值
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void UART_SENDDATA(UINT8 n)
{
  UART_BUFF = n;
}

///*******************************************************************************
// 函数名称：    void UART_init(void)
// 功能描述：    UART1寄存器配置
// 输入参数：    无
// 输出参数：    无
// 返 回 值：    无
// 其它说明：
// 修改日期      版本号          修改人            修改内容
// -----------------------------------------------------------------------------
// 2015/11/5      V1.0           Howlet Li          创建
// *******************************************************************************/
//void UART1_init(void)
//{

//    UART_InitTypeDef UART_InitStruct;
//
//    UART_StructInit(&UART_InitStruct);
//    UART_InitStruct.BaudRate = 38400;                 /* 设置波特率38400 */
//    UART_InitStruct.WordLength = UART_WORDLENGTH_8b;  /* 发送数据长度8位 */
//    UART_InitStruct.StopBits = UART_STOPBITS_1b;
//    UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB;   /* 先发送LSB */
//    UART_InitStruct.ParityMode = UART_Parity_NO;      /* 无奇偶校验 */
//    UART_InitStruct.IRQEna = 0;
//    UART_Init(UART1, &UART_InitStruct);

//}




/*******************************************************************************
 函数名称：    void ADC_init(void)
 功能描述：    ADC0硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void ADC_init(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;
  ADC_InitStructure.Align = ADC_LEFT_ALIGN;
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
  ADC_InitStructure.FirSeg_Ch = 2;                        /* 第一段共采样6个通道 */
  ADC_InitStructure.SecSeg_Ch = 4;                        /* 第二段共采样0个通道 */
  ADC_InitStructure.MCPWM_Trigger_En = ADC_MCPWM_T0_TRG | ADC_MCPWM_T1_TRG;;  /* 打开MCPWM_T0 MCPWM_T1硬件触发事件 */
  ADC_InitStructure.Trigger_Mode = ADC_2SEG_TRG;          /* 设置ADC转换模式为双段式采样 */
  ADC_InitStructure.IE = ADC_EOS1_IRQ_EN;                 /* ADC第一段采样结束中断使能 */
  ADC_InitStructure.ADC_CLK_SEL =  ADC_CLK_48M;
  ADC_InitStructure.ADC_SAMP_CLK = 12; /* 12个时钟周期 */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
  ADC_InitStructure.FirSeg_Ch = 6;                        /* 第一段共采样6个通道 */
  ADC_InitStructure.SecSeg_Ch = 0;                        /* 第二段共采样0个通道 */
  ADC_InitStructure.Trigger_En = ENABLE ; /* 打开MCPWM_T0 MCPWM_T1硬件触发事件 */
  ADC_InitStructure.SEL_En = 0 ; /* TADC触发来源选择。0:MCPWM，1:UTimer */
  ADC_InitStructure.Trigger_Mode = ADC_1SEG_TRG;          /* 设置ADC转换模式为单段式采样 */
  ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;                 /* ADC第一段采样结束中断使能 */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)
  ADC_InitStructure.FirSeg_Ch = 6;                        /* 第一段共采样6个通道 */
  ADC_InitStructure.SecSeg_Ch = 0;                        /* 第二段共采样0个通道 */
  ADC_InitStructure.MCPWM_Trigger_En = ADC_MCPWM_T0_TRG ; /* 打开MCPWM_T0 MCPWM_T1硬件触发事件 */
  ADC_InitStructure.Trigger_Mode = ADC_1SEG_TRG;          /* 设置ADC转换模式为单段式采样 */
  ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;                 /* ADC第一段采样结束中断使能 */
  ADC_InitStructure.ADC_CLK_SEL =  ADC_CLK_48M;
  ADC_InitStructure.ADC_SAMP_CLK = 20; /* 12个时钟周期 */
#endif
#endif
#endif

  ADC_Init(ADC, &ADC_InitStructure);

  ADC_IF = 0xff;
  ADC_STATE_RESET();
  ADC_NormalModeCFG();

  SYS_PROTECT = 0x7A83;    //解除系统寄存器写保护
  SYS_AFE_REG2 = 0x00<< 8;
  SYS_WR_PROTECT = 0;    //使能系统寄存器写保护

}

/*******************************************************************************
 函数名称：    void HALL_Init(void)
 功能描述：    GPIO硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2018/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void HALL_Perip_Init(void)
{
  HALL_InitTypeDef HALL_InitStruct;
  SYS_ModuleClockCmd(SYS_Module_HALL, ENABLE);


  HALL_StructInit(&HALL_InitStruct);
  HALL_InitStruct.FilterLen = 512;                /* Hall信号数字滤波长度 512个时钟周期 */
  HALL_InitStruct.ClockDivision = HALL_CLK_DIV1;  /* 设置Hall模块时钟分频系数 */
  HALL_InitStruct.Filter75_Ena = DISABLE;         /* Hall信号滤波方式，7判5模式或者全1有效模式 */
  HALL_InitStruct.HALL_Ena = ENABLE;              /* 模块使能 */
  HALL_InitStruct.Capture_IRQ_Ena = ENABLE;       /* 捕捉中断使能 */
  HALL_InitStruct.OverFlow_IRQ_Ena = ENABLE;      /* 超时中断使能 */
  HALL_InitStruct.CountTH = 9600000;              /* Hall模块计数模值，计数超过模值会产生超时中断 */

  HALL_Init(&HALL_InitStruct);
}
/*******************************************************************************
 函数名称：    void GPIO_init(void)
 功能描述：    GPIO硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/

void GPIO_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);

  /* MCPWM P0.9 P0.11~P0.15 */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIO0, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_9, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_11, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_12, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_13, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_14, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_15, AF3_MCPWM);

}

/*******************************************************************************
 函数名称：    void UTimer_init(void)
 功能描述：    UTimer硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void UTimer_init(void)
{
    TIM_TimerInitTypeDef TIM_InitStruct;

    TIM_TimerStrutInit(&TIM_InitStruct);
    TIM_InitStruct.EN = ENABLE;
    TIM_InitStruct.TH = 4800;
    TIM_InitStruct.IE = TIM_IRQ_IE_ZC;
    TIM_TimerInit(TIMER1, &TIM_InitStruct);

}

void MCPWM_init(void)
{
  MCPWM_InitTypeDef MCPWM_InitStructure;

  MCPWM_StructInit(&MCPWM_InitStructure);

  MCPWM_InitStructure.CLK_DIV = 0;                          /* MCPWM时钟分频设置 */
  MCPWM_InitStructure.MCLK_EN = ENABLE;                     /* 模块时钟开启 */
  MCPWM_InitStructure.MCPWM_Cnt0_EN = ENABLE;               /* 时基0主计数器开始计数使能开关 */
  MCPWM_InitStructure.MCPWM_Cnt1_EN = ENABLE;               /* 时基1主计数器开始计数使能开关 */
  MCPWM_InitStructure.MCPWM_WorkModeCH0 = CENTRAL_PWM_MODE;
  MCPWM_InitStructure.MCPWM_WorkModeCH1 = CENTRAL_PWM_MODE; /* 通道工作模式设置，中心对齐或边沿对齐 */
  MCPWM_InitStructure.MCPWM_WorkModeCH2 = CENTRAL_PWM_MODE;

  /* 自动更新使能寄存器 MCPWM_TH00 自动加载使能 MCPWM_TMR0 自动加载使能 MCPWM_0TH 自动加载使能 MCPWM_0CNT 自动加载使能*/
  MCPWM_InitStructure.AUEN = TH00_AUEN | TH01_AUEN | TH10_AUEN | TH11_AUEN |
                             TH20_AUEN | TH21_AUEN | TMR0_AUEN | TMR1_AUEN |
                             TMR2_AUEN | TMR3_AUEN | TH0_AUEN | TH30_AUEN | TH31_AUEN ;

  MCPWM_InitStructure.GPIO_BKIN_Filter = 12;                /* 急停事件(来自IO口信号)数字滤波器时间设置 */
  MCPWM_InitStructure.CMP_BKIN_Filter = 1;                 /* 急停事件(来自比较器信号)数字滤波器时间设置 */

  MCPWM_InitStructure.TimeBase0_PERIOD = PWM_PERIOD;        /* 时期0周期设置 */
  MCPWM_InitStructure.TimeBase1_PERIOD = PWM_PERIOD;        /* 时期1周期设置 */
  MCPWM_InitStructure.TriggerPoint0 = (u16)(PWM_PERIOD - 10);//100 /* MCPWM_TMR0 ADC触发事件T0 设置 */
  MCPWM_InitStructure.TriggerPoint1 = (u16)(800-PWM_PERIOD);/* MCPWM_TMR1 ADC触发事件T1 设置 */
  MCPWM_InitStructure.DeadTimeCH0N = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH0P = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH1N = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH1P = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH2N = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH2P = DEADTIME;              /* 死区时间设置 */

#if (PRE_DRIVER_POLARITY == P_HIGH__N_LOW)                    /* CHxP 高有效， CHxN低电平有效 */
  MCPWM_InitStructure.CH0N_Polarity_INV = ENABLE;           /* CH0N通道输出极性设置 | 正常输出或取反输出*/
  MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;          /* CH0P通道输出极性设置 | 正常输出或取反输出 */
  MCPWM_InitStructure.CH1N_Polarity_INV = ENABLE;
  MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH2N_Polarity_INV = ENABLE;
  MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

  MCPWM_InitStructure.Switch_CH0N_CH0P =  DISABLE;           /* 通道交换选择设置 | CH0P和CH0N是否选择信号交换 */
  MCPWM_InitStructure.Switch_CH1N_CH1P =  DISABLE;           /* 通道交换选择设置 */
  MCPWM_InitStructure.Switch_CH2N_CH2P =  DISABLE;           /* 通道交换选择设置 */

  /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                   通道交换和极性控制的影响，直接控制通道输出 */
  MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH0N_default_output = HIGH_LEVEL;
  MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;      /* CH1P对应引脚在空闲状态输出低电平 */
  MCPWM_InitStructure.CH1N_default_output = HIGH_LEVEL;     /* CH1N对应引脚在空闲状态输出高电平 */
  MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH2N_default_output = HIGH_LEVEL;
#else
#if (PRE_DRIVER_POLARITY == P_HIGH__N_HIGH)                    /* CHxP 高有效， CHxN高电平有效 */
  MCPWM_InitStructure.CH0N_Polarity_INV = DISABLE;           /* CH0N通道输出极性设置 | 正常输出或取反输出*/
  MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;          /* CH0P通道输出极性设置 | 正常输出或取反输出 */
  MCPWM_InitStructure.CH1N_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH2N_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

  MCPWM_InitStructure.Switch_CH0N_CH0P =  DISABLE;           /* 通道交换选择设置 | CH0P和CH0N是否选择信号交换 */
  MCPWM_InitStructure.Switch_CH1N_CH1P =  DISABLE;           /* 通道交换选择设置 */
  MCPWM_InitStructure.Switch_CH2N_CH2P =  DISABLE;           /* 通道交换选择设置 */

  /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                   通道交换和极性控制的影响，直接控制通道输出 */
  MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH0N_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;      /* CH1P对应引脚在空闲状态输出低电平 */
  MCPWM_InitStructure.CH1N_default_output = LOW_LEVEL;     /* CH1N对应引脚在空闲状态输出高电平 */
  MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH2N_default_output = LOW_LEVEL;
#endif
#endif

  MCPWM_InitStructure.DebugMode_PWM_out = DISABLE;           /* 在接上仿真器debug程序时，暂停MCU运行时，选择各PWM通道正常输出调制信号
                                                                 还是输出默认电平，保护功率器件 ENABLE:正常输出 DISABLE:输出默认电平*/
  MCPWM_InitStructure.MCPWM_Base0T0_UpdateEN = ENABLE;      /* MCPWM 时基0 T0事件更新使能 */
  MCPWM_InitStructure.MCPWM_Base0T1_UpdateEN = DISABLE;     /* MCPWM 时基0 T1事件更新 禁止*/

  MCPWM_InitStructure.MCPWM_Base1T0_UpdateEN = ENABLE;      /* MCPWM 时基1 T0事件更新使能 */
  MCPWM_InitStructure.MCPWM_Base1T1_UpdateEN = DISABLE;     /* MCPWM 时基1 T1事件更新 禁止*/

#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
  MCPWM_InitStructure.CNT0_T1_Update_INT_EN = ENABLE;           /* T0更新事件 中断使能或关闭 */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
  MCPWM_InitStructure.CNT0_T0_Update_INT_EN = DISABLE;           /* T0更新事件 中断使能或关闭 */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)
  MCPWM_InitStructure.CNT0_T0_Update_INT_EN = DISABLE;           /* T0更新事件 中断使能或关闭 */
#endif
#endif
#endif

  MCPWM_InitStructure.FAIL0_INT_EN = DISABLE;               /* FAIL0事件 中断使能或关闭 */
  MCPWM_InitStructure.FAIL1_INT_EN = ENABLE;               /* FAIL1事件 中断使能或关闭 */
  MCPWM_InitStructure.FAIL1_INPUT_EN = ENABLE;             /* FAIL1通道急停功能打开或关闭 */
  MCPWM_InitStructure.FAIL1_Signal_Sel = FAIL_SEL_CMP;      /* FAIL1事件信号选择，比较器或IO口 */
  MCPWM_InitStructure.FAIL1_Polarity = HIGH_LEVEL_VALID;           /* FAIL1事件极性选择，高有效或低有效 */

#ifdef MCPWM_SWAP_FUNCTION      /* 使用054D, 057D内置预驱芯片需要打开PWM交换功能 */
  MCPWM_PRT = 0x0000DEAD; /*enter password to unlock write protection */
  MCPWM_SWAP = 0x67;
#endif
  MCPWM_Init(MCPWM0, &MCPWM_InitStructure);                 /* MCPWM0 模块初始化 */

  mIPD_CtrProc.hDriverPolarity = MCPWM_IO01;                /* 读出驱动极性 */
}




/*******************************************************************************
 函数名称：    void seekPosADCsetting(void)
 功能描述：    调用IPD功能前，设置ADC工作模式，为初始位置检测做准备
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2017/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void seekPosADCsetting(void)
{

  ADC_STATE_RESET();         /*状态机复位，idle状态，完成后自动请0 */
  MCPWM_TMR0 = PWM_PERIOD;   /*ADC采样时刻0设置为下桥关断中心点 */
  MCPWM_TMR1 = (-600);  /*ADC采样时刻1设置为PWM中心点向后偏600个Clock*/
}

/*******************************************************************************
 函数名称：    void DebugPWM_OutputFunction(void)
 功能描述：    PWM输出功能调试   输出25%占空比
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2017/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void DebugPWM_OutputFunction(void)
{
  MCPWM_TH00 = (-(PWM_PERIOD >> 2));
  MCPWM_TH01 = (PWM_PERIOD >> 2);
  MCPWM_TH10 = (-(PWM_PERIOD >> 2));
  MCPWM_TH11 = (PWM_PERIOD >> 2);
  MCPWM_TH20 = (-(PWM_PERIOD >> 2));
  MCPWM_TH21 = (PWM_PERIOD >> 2);

  PWMOutputs(MCPWM0, ENABLE);
  while(1)
  {
  }
}



