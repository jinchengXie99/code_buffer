/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_MCPWM.h
 * 文件标识： 
 * 内容摘要： MCPWM外设驱动程序头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： HMG
 * 完成日期： 2021年10月14日
 *
 *
 *******************************************************************************/

#ifndef __LKS03X_PWM_H
#define __LKS03X_PWM_H

/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"

typedef struct
{
  __IO u32 PWM_TH00; /* MCPWM CH0_P 比较门限值寄存器 0x00*/
  __IO u32 PWM_TH01; /* MCPWM CH0_N 比较门限值寄存器*/
  __IO u32 PWM_TH10;
  __IO u32 PWM_TH11;
  __IO u32 PWM_TH20;
  __IO u32 PWM_TH21;
  __IO u32 PWM_TH30;
  __IO u32 PWM_TH31;
  __IO u32 PWM_TMR0; /* ADC 采样定时器比较门限 0 寄存器 */
  __IO u32 PWM_TMR1;
  __IO u32 PWM_TMR2;
  __IO u32 PWM_TMR3;
  __IO u32 PWM_TH0; /* MCPWM 时基 0 门限值寄存器 */
  __IO u32 PWM_TH1;
  __IO u32 PWM_CNT0; /* MCPWM 时基 0 计数器寄存器 0x38 */
  __IO u32 PWM_CNT1;
  __IO u32 PWM_UPDATE; /* MCPWM 手动加载控制寄存器 */
  __IO u32 PWM_FCNT;   /* MCPWM FAIL 时刻 CNT 值 */
  __IO u32 PWM_EVT0;   /* MCPWM 时基 0 外部触发 */
  __IO u32 PWM_EVT1;
  __IO u32 PWM_DTH0; /* MCPWM CH0123 N通道死区宽度控制寄存器 */
  __IO u32 PWM_DTH1; /* MCPWM CH0123 P通道死区宽度控制寄存器 */
  __IO u32 null0;
  __IO u32 null1;
  __IO u32 null2;
  __IO u32 null3;
  __IO u32 null4;
  __IO u32 null5;
  __IO u32 PWM_FLT;   /* MCPWM 滤波时钟分频寄存器 0x70 */
  __IO u32 PWM_SDCFG; /* MCPWM 加载配置寄存器 */
  __IO u32 PWM_AUEN;  /* MCPWM 自动更新使能寄存器 */
  __IO u32 PWM_TCLK;  /* MCPWM 时钟分频控制寄存器 */
  __IO u32 PWM_IE0;   /* MCPWM 时基 0 中断控制寄存器 */
  __IO u32 PWM_IF0;   /* MCPWM 时基 0 中断标志位寄存器 */
  __IO u32 PWM_IE1;
  __IO u32 PWM_IF1;
  __IO u32 PWM_EIE; /* MCPWM 异常中断控制寄存器 0x90*/
  __IO u32 PWM_EIF;
  __IO u32 PWM_RE;   /* MCPWM DMA 请求控制寄存器 */
  __IO u32 PWM_PP;   /* MCPWM 推挽模式使能寄存器 */
  __IO u32 PWM_IO01; /* MCPWM CH0 CH1 IO 控制寄存器 */
  __IO u32 PWM_IO23;
  __IO u32 PWM_FAIL012; /* MCPWM CH0 CH1 CH2 短路控制寄存器 */
  __IO u32 PWM_FAIL3;   /* MCPWM CH3 短路控制寄存器 */
  __IO u32 PWM_PRT;     /* MCPWM 保护寄存器 */
  __IO u32 PWM_CHMSK;   /* MCPWM 通道屏蔽位寄存器 */
} MCPWM_REG_TypeDef;

typedef struct
{
  u16 TimeBase0_PERIOD; /* 时期0周期设置*/
  u16 TimeBase1_PERIOD; /* 时期1周期设置*/
  u8 CLK_DIV;           /* MCPWM 分频系数 */
  u8 MCLK_EN;           /* MCPWM 时钟使能开关 */
  u8 MCPWM_Cnt0_EN;     /* MCPWM 时基0主计数器使能开关 */
  u8 MCPWM_Cnt1_EN;     /* MCPWM 时基1主计数器使能开关 */
  u8 GPIO_BKIN_Filter;  /* GPIO输入滤波时钟设置1-16 */
  u8 CMP_BKIN_Filter;   /* 比较器CMP输入滤波时钟设置1-16 */

  u8 TMR2_TimeBase_Sel; /* TMR2 比较门限寄存器 时基选择 0:时基0 | 1:时基1 */
  u8 TMR3_TimeBase_Sel; /* TMR3 比较门限寄存器 时基选择 1:时基0 | 1:时基1 */

  u8 TimeBase0_Trig_Enable; /* 时基0 外部触发使能 */
  u8 TimeBase1_Trig_Enable; /* 时基1 外部触发使能*/

  u16 TimeBase_TrigEvt0; /* 时基0 外部触发事件选择 */
  u16 TimeBase_TrigEvt1; /* 时基1 外部触发事件选择 */

  s16 TimeBase0Init_CNT; /* 时基0 计数器初始值 */
  s16 TimeBase1Init_CNT; /* 时基1 计数器初始值 */

  u16 MCPWM_WorkModeCH0; /* MCPWM CH0工作模式：边沿对齐/中心对齐 */
  u16 MCPWM_WorkModeCH1; /* MCPWM CH0工作模式：边沿对齐/中心对齐 */
  u16 MCPWM_WorkModeCH2; /* MCPWM CH0工作模式：边沿对齐/中心对齐 */
  u16 MCPWM_WorkModeCH3; /* MCPWM CH0工作模式：边沿对齐/中心对齐 */

  u16 TriggerPoint0; /* PWM触发ADC事件0，时间点设置 */
  u16 TriggerPoint1; /* PWM触发ADC事件1，时间点设置 */
  u16 TriggerPoint2; /* PWM触发ADC事件2，时间点设置 */
  u16 TriggerPoint3; /* PWM触发ADC事件3，时间点设置 */

  u16 DeadTimeCH0N; /* CH0N死区时间设置　*/
  u16 DeadTimeCH0P; /* CH0P死区时间设置　*/
  u16 DeadTimeCH1N; /* CH1N死区时间设置　*/
  u16 DeadTimeCH1P; /* CH1P死区时间设置　*/
  u16 DeadTimeCH2N; /* CH2N死区时间设置　*/
  u16 DeadTimeCH2P; /* CH2P死区时间设置　*/
  u16 DeadTimeCH3N; /* CH3N死区时间设置　*/
  u16 DeadTimeCH3P; /* CH3P死区时间设置　*/

  u8 CH0N_Polarity_INV; /* CH0N输出极性取反，0:正常输出；1:取反输出 */
  u8 CH0P_Polarity_INV; /* CH0P输出极性取反，0:正常输出；1:取反输出 */
  u8 CH1N_Polarity_INV; /* CH1N输出极性取反，0:正常输出；1:取反输出 */
  u8 CH1P_Polarity_INV; /* CH1P输出极性取反，0:正常输出；1:取反输出 */
  u8 CH2N_Polarity_INV; /* CH2N输出极性取反，0:正常输出；1:取反输出 */
  u8 CH2P_Polarity_INV; /* CH2P输出极性取反，0:正常输出；1:取反输出 */
  u8 CH3N_Polarity_INV; /* CH3N输出极性取反，0:正常输出；1:取反输出 */
  u8 CH3P_Polarity_INV; /* CH3P输出极性取反，0:正常输出；1:取反输出 */

  u8 Switch_CH0N_CH0P; /* 交换CH0N, CH0P信号输出使能开关 */
  u8 Switch_CH1N_CH1P; /* 交换CH1N, CH1P信号输出使能开关 */
  u8 Switch_CH2N_CH2P; /* 交换CH2N, CH2P信号输出使能开关 */
  u8 Switch_CH3N_CH3P; /* 交换CH3N, CH3P信号输出使能开关 */

  u8 MCPWM_UpdateT0Interval; /* MCPWM T0事件更新间隔 */
  u8 MCPWM_UpdateT1Interval; /* MCPWM T1事件更新间隔 */
  u8 MCPWM_Base0T0_UpdateEN; /* MCPWM 时基0 T0事件更新使能 */
  u8 MCPWM_Base0T1_UpdateEN; /* MCPWM 时基0 T1事件更新使能 */
  u8 MCPWM_Base1T0_UpdateEN; /* MCPWM 时基1 T0事件更新使能 */
  u8 MCPWM_Base1T1_UpdateEN; /* MCPWM 时基1 T1事件更新使能 */
  u8 MCPWM_Auto0_ERR_EN;     /* MCPWM 时基0更新事件是否自动打开MOE, 使能开关 */
  u8 MCPWM_Auto1_ERR_EN;     /* MCPWM 时基1更新事件是否自动打开MOE, 使能开关 */

  u8 FAIL0_INPUT_EN;    /* FAIL0 输入功能使能 */
  u8 FAIL1_INPUT_EN;    /* FAIL1 输入功能使能 */
  u8 FAIL0_Signal_Sel;  /* FAIL0 信号选择，比较器0或GPIO */
  u8 FAIL1_Signal_Sel;  /* FAIL1 信号选择，比较器0或GPIO */
  u8 FAIL0_Polarity;    /* FAIL0 信号极性设置，高有效或低有效 */
  u8 FAIL1_Polarity;    /* FAIL1 信号极性设置，高有效或低有效 */
  u8 DebugMode_PWM_out; /* Debug时，MCU进入Halt, MCPWM信号是否正常输出 */
  u8 FAIL2_INPUT_EN;    /* FAIL2 输入功能使能 */
  u8 FAIL3_INPUT_EN;    /* FAIL3 输入功能使能 */
  u8 FAIL2_Signal_Sel;  /* FAIL2 信号选择，比较器0或GPIO */
  u8 FAIL3_Signal_Sel;  /* FAIL3 信号选择，比较器0或GPIO */
  u8 FAIL2_Polarity;    /* FAIL2 信号极性设置，高有效或低有效 */
  u8 FAIL3_Polarity;    /* FAIL3 信号极性设置，高有效或低有效 */

  u8 CH0P_default_output; /* CH0P MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH0N_default_output; /* CH0N MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH1P_default_output; /* CH1P MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH1N_default_output; /* CH1N MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH2P_default_output; /* CH2P MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH2N_default_output; /* CH2N MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH3P_default_output; /* CH3P MOE为0时或发生FAIL事件时，默认电平输出 */
  u8 CH3N_default_output; /* CH3N MOE为0时或发生FAIL事件时，默认电平输出 */

  u8 CNT0_T0_Update_INT_EN;  /* 时基0 T0更新事件中斷使能 */
  u8 CNT0_T1_Update_INT_EN;  /* 时基0 T1更新事件中斷使能 */
  u8 CNT0_TMR0_Match_INT_EN; /* 时基0 TMR0计数事件匹配事件中断使能 */
  u8 CNT0_TMR1_Match_INT_EN; /* 时基0 TMR1计数事件匹配事件中断使能 */

  u8 CNT1_T0_Update_INT_EN;  /* 时基1 T0更新事件中斷使能 */
  u8 CNT1_T1_Update_INT_EN;  /* 时基1 T1更新事件中斷使能 */
  u8 CNT1_TMR0_Match_INT_EN; /* 时基1 TMR0计数事件匹配事件中断使能 */
  u8 CNT1_TMR1_Match_INT_EN; /* 时基1 TMR1计数事件匹配事件中断使能 */

  u8 TMR0_DMA_RE;   /* MCPWM计数器命中TMR0，DMA请求使能*/
  u8 TMR1_DMA_RE;   /* MCPWM计数器命中TMR1，DMA请求使能*/
  u8 TMR2_DMA_RE;   /* MCPWM计数器命中TMR2，DMA请求使能*/
  u8 TMR3_DMA_RE;   /* MCPWM计数器命中TMR3，DMA请求使能*/
  u8 TR0_T0_DMA_RE; /* 时基0 T0事件DMA请求使能*/
  u8 TR0_T1_DMA_RE; /* 时基0 T1事件DMA请求使能*/
  u8 TR1_T0_DMA_RE; /* 时基1 T0事件DMA请求使能*/
  u8 TR1_T1_DMA_RE; /* 时基1 T1事件DMA请求使能*/

  u8 FAIL0_INT_EN; /* FAIL0事件中断使能 */
  u8 FAIL1_INT_EN; /* FAIL1事件中断使能 */
  u8 FAIL2_INT_EN; /* FAIL2事件中断使能 */
  u8 FAIL3_INT_EN; /* FAIL3事件中断使能 */

  u16 AUEN; /* 自动更新使能寄存器 */

} MCPWM_InitTypeDef;

#define MCPWM_MOE_ENABLE_MASK ((u16)0x0040)   /* 打开MOE位MASK位 */
#define MCPWM_MOE_DISABLE_MASK ((u16)~0x0040) /* 关MOE位MASK位 */

#define PWM_F_SELECT_CMP0 BIT0 /* 比较器信号选择 */
#define PWM_F_SELECT_CMP1 BIT1
#define PWM_F_LFAIL_CH0 BIT2
#define PWM_F_LFAIL_CH1 BIT3
#define PWM_F_ENABLE_CH0 BIT4
#define PWM_F_ENABLE_CH1 BIT5

#define PWM_ENABLE_BK0IF BIT4
#define PWM_ENABLE_BK1IF BIT5
#define PWM_ENABLE_CMP0IF BIT6
#define PWM_ENABLE_CMP1IF BIT7

#define MCPWM_TMR0_IF BIT10 /* 计数值等于 MCPWM_TMR0 中断源事件 */

#define TH00_AUEN BIT0 /* MCPWM_TH00 自动加载使能 */
#define TH01_AUEN BIT1
#define TH10_AUEN BIT2
#define TH11_AUEN BIT3
#define TH20_AUEN BIT4
#define TH21_AUEN BIT5
#define TH30_AUEN BIT6
#define TH31_AUEN BIT7
#define TMR0_AUEN BIT8
#define TMR1_AUEN BIT9
#define TMR2_AUEN BIT10
#define TMR3_AUEN BIT11
#define TH0_AUEN BIT12  /* MCPWM_0TH 自动加载使能 */
#define TH1_AUEN BIT13  /* MCPWM_1TH 自动加载使能 */
#define CNT0_AUEN BIT14 /* MCPWM_0CNT 自动加载使能 */
#define CNT1_AUEN BIT15 /* MCPWM_1CNT 自动加载使能 */

#define MCPWM0_T0_TRG_EVT (BIT0)   /* ADC采用MCPWM0 T0事件触发 */
#define MCPWM0_T1_TRG_EVT (BIT1)   /* ADC采用MCPWM0 T1事件触发 */
#define MCPWM0_T2_TRG_EVT (BIT2)   /* ADC采用MCPWM0 T2事件触发 */
#define MCPWM0_T3_TRG_EVT (BIT3)   /* ADC采用MCPWM0 T3事件触发 */
#define MCPWM1_T0_TRG_EVT (BIT4)   /* ADC采用MCPWM1 T0事件触发 */
#define MCPWM1_T1_TRG_EVT (BIT5)   /* ADC采用MCPWM1 T1事件触发 */
#define MCPWM1_T2_TRG_EVT (BIT6)   /* ADC采用MCPWM1 T2事件触发 */
#define MCPWM1_T3_TRG_EVT (BIT7)   /* ADC采用MCPWM1 T3事件触发 */
#define UTIMER0_T0_TRG_EVT (BIT8)  /* ADC采用UTIMER0 T0事件触发 */
#define UTIMER0_T1_TRG_EVT (BIT9)  /* ADC采用UTIMER0 T1事件触发 */
#define UTIMER1_T0_TRG_EVT (BIT10) /* ADC采用UTIMER1 T0事件触发 */
#define UTIMER1_T1_TRG_EVT (BIT11) /* ADC采用UTIMER1 T1事件触发 */
#define UTIMER2_T0_TRG_EVT (BIT12) /* ADC采用UTIMER2 T0事件触发 */
#define UTIMER2_T1_TRG_EVT (BIT13) /* ADC采用UTIMER2 T1事件触发 */
#define UTIMER3_T0_TRG_EVT (BIT14) /* ADC采用UTIMER3 T0事件触发 */
#define UTIMER3_T1_TRG_EVT (BIT15) /* ADC采用UTIMER3 T1事件触发 */

#define CENTRAL_PWM_MODE 0 /* 中心对齐PWM模式 */
#define EDGE_PWM_MODE 1    /* 边沿对齐PWM模式 */

#define HIGH_LEVEL 1 /* 高电平 */
#define LOW_LEVEL 0  /* 低电平 */

#define HIGH_LEVEL_VALID 0 /* FAIL极性选择 高电平有效 */
#define LOW_LEVEL_VALID 1  /* FAIL极性选择 低电平有效 */

#define FAIL_SEL_CMP 1 /* Fail事件来源比较器 */
#define FAIL_SEL_IO 0  /* Fail事件来源IO */

#define FAIL0_IF BIT4 /* FAIL0 中断标志位 写1清零 */
#define FAIL1_IF BIT5 /* FAIL0 中断标志位 写1清零 */
#define FAIL2_IF BIT6 /* FAIL0 中断标志位 写1清零 */
#define FAIL3_IF BIT7 /* FAIL0 中断标志位 写1清零 */

#define MCPWM_UPDATE_REG()  \
  {                         \
    MCPWM_PRT = 0x0000DEAD; \
    MCPWM_UPDATE = 0x00ff;  \
    MCPWM_PRT = 0x0000CAFE; \
  }

void PWMOutputs(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state);
void PWMOutputs_CH3(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state);
void MCPWM_Init(MCPWM_REG_TypeDef *MCPWMx, MCPWM_InitTypeDef *MCPWM_InitStruct);
void MCPWM_StructInit(MCPWM_InitTypeDef *MCPWM_InitStruct);
void PWM_CH3_Outputs(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state);

#endif /*__CHANHOM_PWM_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
