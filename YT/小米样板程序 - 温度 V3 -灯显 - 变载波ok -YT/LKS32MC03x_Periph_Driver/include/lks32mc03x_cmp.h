/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_cmp.h
 * 文件标识：
 * 内容摘要： 比较器驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/
#ifndef __lks32mc03x_cmp_H
#define __lks32mc03x_cmp_H
/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"
// 比较器回查选择
#define CMP_HYS_20mV         0
#define CMP_HYS_0mV          1

// 比较器 1 信号正端选择
#define CMP1_SELP_CMP1_IP0   0
#define CMP1_SELP_OPA0_IP    1
#define CMP1_SELP_OPA0_OUT   2
#define CMP1_SELP_CMP1_IP1   3
#define CMP1_SELP_CMP1_IP2   4
#define CMP1_SELP_CMP1_IP3   5

// 比较器 1 信号负端选择
#define CMP1_SELN_CMP1_IN    0
#define CMP1_SELN_REF        1
#define CMP1_SELN_DAC        2
#define CMP1_SELN_HALL_MID   3

// 比较器 0 信号正端选择
#define CMP0_SELP_CMP0_IP0   0
#define CMP0_SELP_OPA0_IP    1
#define CMP0_SELP_OPA0_OUT   2
#define CMP0_SELP_CMP0_IP1   3
#define CMP0_SELP_CMP0_IP2   4
#define CMP0_SELP_CMP0_IP3   5

// 比较器 0 信号负端选择
#define CMP0_SELN_CMP0_IN    0
#define CMP0_SELN_REF        1
#define CMP0_SELN_DAC        2
#define CMP0_SELN_HALL_MID   3

typedef struct
{
    __IO uint32_t IE;       // 比较器中断使能寄存器
    __IO uint32_t IF;       // 比较器中断标志寄存器
    __IO uint32_t TCLK;     // 比较器分频时钟控制寄存器
    __IO uint32_t CFG;      // 比较器控制寄存器
    __IO uint32_t BLCWIN;   // 比较器开窗控制寄存器
    __IO uint32_t DATA;     // 比较器输出数值寄存器
} CMP_TypeDef;

typedef struct
{
    // 比较器IO配置
    u32 FIL_CLK10_DIV16;    // 比较器 1/0 滤波
    u32 CLK10_EN;           // 比较器 1/0 滤波时钟使能
    u32 FIL_CLK10_DIV2;     // 比较器 1/0 滤波时钟分频
    u32 CMP_FT;             // 比较器快速比较使能
    u32 CMP_HYS;            // 比较器回差选择
    
    // 比较器1
    u32 CMP1_RE;            // 比较器 1 DMA 请求使能
    u32 CMP1_IE;            // 比较器 1 中断使能
    u32 CMP1_W_PWM_POL;     // 比较器 1 开窗 PWM 信号极性选择
    u32 CMP1_IRQ_TRIG;      // 比较器 1 边沿触发使能
    u32 CMP1_IN_EN;         // 比较器 1 信号输入使能
    u32 CMP1_POL;           // 比较器 1 极性选择
    u32 CMP1_SELP;          // 比较器 1 信号正端选择
    u32 CMP1_SELN;          // 比较器 1 信号负端选择
    u32 CMP1_CHN3P_WIN_EN;  // MCPWM 模块 CHN3_P 通道使能比较器 1 开窗
    u32 CMP1_CHN2P_WIN_EN;  // MCPWM 模块 CHN2_P 通道使能比较器 1 开窗
    u32 CMP1_CHN1P_WIN_EN;  // MCPWM 模块 CHN1_P 通道使能比较器 1 开窗
    u32 CMP1_CHN0P_WIN_EN;  // MCPWM 模块 CHN0_P 通道使能比较器 1 开窗
    
    // 比较器0
    u32 CMP0_IE;            // 比较器 0 中断使能
    u32 CMP0_RE;            // 比较器 0 DMA 请求使能
    u32 CMP0_W_PWM_POL;     // 比较器 0 开窗 PWM 信号极性选择
    u32 CMP0_IRQ_TRIG;      // 比较器 0 边沿触发使能
    u32 CMP0_IN_EN;         // 比较器 0 信号输入使能
    u32 CMP0_POL;           // 比较器 0 极性选择
    u32 CMP0_SELP;          // 比较器 0 信号正端选择
    u32 CMP0_SELN;          // 比较器 0 信号负端选择
    u32 CMP0_CHN3P_WIN_EN;  // MCPWM 模块 CHN3_P 通道使能比较器 0 开窗
    u32 CMP0_CHN2P_WIN_EN;  // MCPWM 模块 CHN2_P 通道使能比较器 0 开窗
    u32 CMP0_CHN1P_WIN_EN;  // MCPWM 模块 CHN1_P 通道使能比较器 0 开窗
    u32 CMP0_CHN0P_WIN_EN;  // MCPWM 模块 CHN0_P 通道使能比较器 0 开窗
}CMP_InitTypeDef;

void CMP_Init(CMP_InitTypeDef *);        // CMP初始化
void CMP_StructInit(CMP_InitTypeDef *);   // CMP配置结构体初始化

#endif /*__lks32mc03x_cmp_H */
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
