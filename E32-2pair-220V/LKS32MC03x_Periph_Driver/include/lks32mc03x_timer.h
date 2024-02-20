/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_timer.h
 * 文件标识：
 * 内容摘要： 通用定时器驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/

#ifndef __lks32mc03x_TIM_H
#define __lks32mc03x_TIM_H

/* Includes ------------------------------------------------------------------*/

#include "basic.h"

typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t TH;
    __IO uint32_t CNT;
    __IO uint32_t CMPT0;
    __IO uint32_t CMPT1;
    __IO uint32_t EVT;
    __IO uint32_t FLT;
    __IO uint32_t IE;
    __IO uint32_t IF;
} TIM_TimerTypeDef;

// 当timer时钟为MCLK时
#define TIM_Clk_Div1 0x00   // Timer工作频率48Mhz
#define TIM_Clk_Div2 0x01   // Timer工作频率24Mhz
#define TIM_Clk_Div4 0x02   // Timer工作频率12Mhz
#define TIM_Clk_Div8 0x03   // Timer工作频率6Mhz
#define TIM_Clk_Div16 0x04  // Timer工作频率3Mhz
#define TIM_Clk_Div32 0x05  // Timer工作频率1.5Mhz
#define TIM_Clk_Div64 0x06  // Timer工作频率750Khz
#define TIM_Clk_Div128 0x07 // Timer工作频率375Khz

#define TIM_Clk_SRC_MCLK 0x00    // Timer使用芯片内部时钟
#define TIM_Clk_SRC_Timer00 0x01 // Timer使用外部Timer00信号
#define TIM_Clk_SRC_Timer01 0x02 // Timer使用外部Timer01信号
#define TIM_Clk_SRC_Timer10 0x03 // Timer使用外部Timer10信号
#define TIM_Clk_SRC_Timer11 0x04 // Timer使用外部Timer11信号

#define TIM_SRC1_0 0x00     // Timer捕获 外接通道0
#define TIM_SRC1_1 0x01     // Timer捕获 外接通道1
#define TIM_SRC1_CMP0 0x02  // Timer捕获 比较器 0 的输出
#define TIM_SRC1_CMP1 0x03  // Timer捕获 比较器 1 的输出
#define TIM_SRC1_0XOR1 0x04 // Timer捕获 通道0和通道1的异或

#define TIM_SRC0_0 0x00     // Timer捕获 外接通道0
#define TIM_SRC0_1 0x01     // Timer捕获 外接通道1
#define TIM_SRC0_CMP0 0x02  // Timer捕获 比较器 0 的输出
#define TIM_SRC0_CMP1 0x03  // Timer捕获 比较器 1 的输出
#define TIM_SRC0_0XOR1 0x04 // Timer捕获 通道0和通道1的异或

// Timer计数使能信号来源
#define TIM_EVT_SRC_Timer00 0x00 // Timer00比较事件
#define TIM_EVT_SRC_Timer01 0x01 // Timer01比较事件
#define TIM_EVT_SRC_Timer10 0x02 // Timer10比较事件
#define TIM_EVT_SRC_Timer11 0x03 // Timer11比较事件
#define TIM_EVT_SRC_MCPWM_0 0x0A // MCPWM TADC0事件
#define TIM_EVT_SRC_MCPWM_1 0x0B // MCPWM TADC1事件
#define TIM_EVT_SRC_MCPWM_2 0x0C // MCPWM TADC2事件
#define TIM_EVT_SRC_MCPWM_3 0x0D // MCPWM TADC3事件

// TIMER产生DMA请求
#define TIM_IRQ_RE_ZC (BIT10) // Timer计数器过0(计数器回零)
#define TIM_IRQ_RE_CH1 (BIT9) // Timer1比较OR捕获事件
#define TIM_IRQ_RE_CH0 (BIT8) // Timer0比较OR捕获事件

// TIMER中断使能
#define TIM_IRQ_IE_ZC (BIT2)  // Timer计数器过0(计数器回零)
#define TIM_IRQ_IE_CH1 (BIT1) // Timer1比较OR捕获事件
#define TIM_IRQ_IE_CH0 (BIT0) // Timer0比较OR捕获事件

// TIMER中断标志位
#define TIM_IRQ_IF_ZC (BIT2)  // Timer计数器过0(计数器回零)
#define TIM_IRQ_IF_CH1 (BIT1) // Timer1比较OR捕获事件
#define TIM_IRQ_IF_CH0 (BIT0) // Timer0比较OR捕获事件

typedef struct
{
    uint32_t EN;          // Timer 模块使能
    uint32_t CAP1_CLR_EN; // 当发生 CAP1 捕获事件时，清零 Timer 计数器，高有效
    uint32_t CAP0_CLR_EN; // 当发生 CAP0 捕获事件时，清零 Timer 计数器，高有效
    uint32_t ETON;        // Timer 计数器计数使能配置 0: 自动运行 1: 等待外部事件触发计数
    uint32_t CLK_DIV;     // Timer 计数器分频
    uint32_t CLK_SRC;     // Timer 时钟源
    uint32_t TH;          // Timer 计数器计数门限。计数器从 0 计数到 TH 值后再次回 0 开始计数

    uint32_t SRC1;          // Timer 通道 1 捕获模式信号来源
    uint32_t CH1_POL;       // Timer 通道 1 在比较模式下的输出极性控制，计数器回0后的输出值
    uint32_t CH1_MODE;      // Timer 通道 1 工作模式选择，0：比较模式，1：捕获模式
    uint32_t CH1_FE_CAP_EN; // Timer 通道 1 下降沿捕获事件使能。1:使能；0:关闭
    uint32_t CH1_RE_CAP_EN; // Timer 通道 1 上升沿捕获事件使能。1:使能；0:关闭
    uint32_t CMP1;          // Timer 通道 1 比较门限

    uint32_t SRC0;          // Timer 通道 0 捕获模式信号来源
    uint32_t CH0_POL;       // Timer 通道 0 在比较模式下的输出极性控制，计数器回0后的输出值
    uint32_t CH0_MODE;      // Timer 通道 0 工作模式选择，0：比较模式，1：捕获模式
    uint32_t CH0_FE_CAP_EN; // Timer 通道 0 下降沿捕获事件使能。1:使能；0:关闭
    uint32_t CH0_RE_CAP_EN; // Timer 通道 0 上升沿捕获事件使能。1:使能；0:关闭
    uint32_t CMP0;          // Timer 通道 0 比较门限

    uint32_t CNT;     // Timer 计数器当前计数值。写操作可以写入新的计数值。
    uint32_t EVT_SRC; // Timer 计数使能开始后，外部事件选择
    uint32_t FLT;     // 通道 0/1 信号滤波宽度选择，0-255
    uint32_t IE;      // Timer 中断使能
} TIM_TimerInitTypeDef;

/*Timer初始化*/
void TIM_TimerInit(TIM_TimerTypeDef *TIMERx, TIM_TimerInitTypeDef *TIM_TimerInitStruct);
void TIM_TimerStrutInit(TIM_TimerInitTypeDef *TIM_TimerInitStruct);

#endif /*__lks32mc03x_TIM_H */
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
