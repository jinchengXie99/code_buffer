/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_adc.h
 * 文件标识：
 * 内容摘要： ADC外设驱动程序头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   HMG
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/

#ifndef _CONNIE_ADC_H_
#define _CONNIE_ADC_H_

/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"

typedef struct
{
    __IO uint32_t DAT0;
    __IO uint32_t DAT1;
    __IO uint32_t DAT2;
    __IO uint32_t DAT3;
    __IO uint32_t DAT4;
    __IO uint32_t DAT5;
    __IO uint32_t DAT6;
    __IO uint32_t DAT7;
    __IO uint32_t DAT8;
    __IO uint32_t DAT9;
    __IO uint32_t null0;
    __IO uint32_t null1;
    __IO uint32_t LTH;
    __IO uint32_t HTH;
    __IO uint32_t GEN;
    __IO uint32_t null2;
    __IO uint32_t CHN0;
    __IO uint32_t CHN1;
    __IO uint32_t CHN2;
    __IO uint32_t null3;

    __IO uint32_t CHNT;

    __IO uint32_t CFG;
    __IO uint32_t SWT;

    __IO uint32_t null;

    __IO uint32_t DC;
    __IO uint32_t AMC;

    __IO uint32_t IE;
    __IO uint32_t IF;

} ADC_TypeDef;

typedef struct
{
    uint16_t IE;           /* Configures ADC_IRQ */
    uint16_t Align;        /* Configures ADC Result Align */
    uint16_t Con_Sample;   /* 连续采样模式 */
    uint16_t Trigger_Cnt;  /* 单段模式下触发下一次采样所需的触发事件数 */
    uint16_t FirSeg_Ch;    /* 第一段采样的总通道数 */
    uint16_t SecSeg_Ch;    /* 第二段采样的总通道数 */
    uint16_t ThrSeg_Ch;    /* 第三段采样的总通道数 */
    uint16_t FouSeg_Ch;    /* 第四段采样的总通道数 */
    uint16_t Trigger_Mode; /* ADC trigger mode */
    uint16_t Trigger_En;   /* TADC触发ADC采样使能信号 */
    uint16_t SEL_En;       /* TADC触发来源选择。0:MCPWM，1:UTimer */
    uint16_t ADC_GEN_En;  /* ADC模拟看门狗 */
    uint16_t ADC_GEN_HTH; /* ADC模拟看门狗上阈值 */
    uint16_t ADC_GEN_LTH; /* ADC模拟看门狗下阈值 */
} ADC_InitTypeDef;

#define ADC_1_TIMES_SAMPLE ((u8)1) /* ADC采样通道数1次采样 */
#define ADC_2_TIMES_SAMPLE ((u8)2) /* ADC采样通道数2次采样 */
#define ADC_3_TIMES_SAMPLE ((u8)3) /* ADC采样通道数3次采样 */
#define ADC_4_TIMES_SAMPLE ((u8)4)
#define ADC_5_TIMES_SAMPLE ((u8)5)
#define ADC_6_TIMES_SAMPLE ((u8)6)
#define ADC_7_TIMES_SAMPLE ((u8)7)
#define ADC_8_TIMES_SAMPLE ((u8)8)
#define ADC_9_TIMES_SAMPLE ((u8)9)
#define ADC_10_TIMES_SAMPLE ((u8)10)
#define ADC_11_TIMES_SAMPLE ((u8)11)
#define ADC_12_TIMES_SAMPLE ((u8)12)
#define ADC_13_TIMES_SAMPLE ((u8)13)
#define ADC_14_TIMES_SAMPLE ((u8)14)
#define ADC_15_TIMES_SAMPLE ((u8)15)
#define ADC_16_TIMES_SAMPLE ((u8)16)
#define ADC_17_TIMES_SAMPLE ((u8)17)
#define ADC_18_TIMES_SAMPLE ((u8)18)
#define ADC_19_TIMES_SAMPLE ((u8)19)
#define ADC_20_TIMES_SAMPLE ((u8)20) /* ADC采样通道数20次采样 */

#define ADC_GEN_DAT0 ((uint16_t)0x01)  /* DAT0看门狗监测使能 */
#define ADC_GEN_DAT1 ((uint16_t)0x02)  /* DAT1看门狗监测使能 */
#define ADC_GEN_DAT2 ((uint16_t)0x04)  /* DAT2看门狗监测使能 */
#define ADC_GEN_DAT3 ((uint16_t)0x08)  /* DAT3看门狗监测使能 */
#define ADC_GEN_DAT4 ((uint16_t)0x10)  /* DAT4看门狗监测使能 */
#define ADC_GEN_DAT5 ((uint16_t)0x20)  /* DAT5看门狗监测使能 */
#define ADC_GEN_DAT6 ((uint16_t)0x40)  /* DAT6看门狗监测使能 */
#define ADC_GEN_DAT7 ((uint16_t)0x80)  /* DAT7看门狗监测使能 */
#define ADC_GEN_DAT8 ((uint16_t)0x100) /* DAT8看门狗监测使能 */
#define ADC_GEN_DAT9 ((uint16_t)0x200) /* DAT9看门狗监测使能 */

#define ADC_EOS0_IRQ_EN ((uint16_t)0x01)     /* 第一段扫描结束中断 */
#define ADC_EOS1_IRQ_EN ((uint16_t)0x02)     /* 第二段扫描结束中断 */
#define ADC_EOS2_IRQ_EN ((uint16_t)0x04)     /* 第三段扫描结束中断 */
#define ADC_EOS3_IRQ_EN ((uint16_t)0x08)     /* 第四段扫描结束中断 */
#define ADC_CONFLICT_IRQ_EN ((uint16_t)0x10) /* 软件触发冲突 */
#define ADC_HERR_IRQ_EN ((uint16_t)0x20)     /* 硬件触发冲突 */
#define ADC_DAT0_OV_IRQ_EN ((uint16_t)0x40)  /* ADC_DAT0超阈值中断 */

#define ADC_EOS0_DMA_EN ((uint16_t)0x100)      /* 第一段扫描结束DMA请求使能 */
#define ADC_EOS1_DMA_EN ((uint16_t)0x200)      /* 第二段扫描结束DMA请求使能 */
#define ADC_EOS2_DMA_EN ((uint16_t)0x400)      /* 第三段扫描结束DMA请求使能 */
#define ADC_EOS3_DMA_EN ((uint16_t)0x800)      /* 第四段扫描结束DMA请求使能 */
#define ADC_CONFLICT_DMA_EN ((uint16_t)0x1000) /* 软件触发冲突DMA请求使能 */
#define ADC_HERR_DMA_EN ((uint16_t)0x2000)     /* 硬件触发冲突DMA请求使能 */
#define ADC_DAT0_OV_DMA_EN ((uint16_t)0x4000)  /* ADC_DAT0超阈值DMA请求使能 */

#define ADC_DAT0_OV_IRQ_IF ((uint16_t)0x40)  /* ADC_DAT0超阈值中断 */
#define ADC_CONFLICT_IRQ_IF ((uint16_t)0x20) /* 硬件触发冲突标志 */
#define ADC_HERR_IRQ_IF ((uint16_t)0x10)     /* 软件触发冲突标志 */
#define ADC_EOS3_IRQ_IF ((uint16_t)0x08)     /* 第四段扫描结束中断标志 */
#define ADC_EOS2_IRQ_IF ((uint16_t)0x04)     /* 第三段扫描结束中断标志*/
#define ADC_EOS1_IRQ_IF ((uint16_t)0x02)     /* 第二段扫描结束中断标志 */
#define ADC_EOS0_IRQ_IF ((uint16_t)0x01)     /* 第一段扫描结束中断标志 */

#define ADC_LEFT_ALIGN ((uint16_t)0x0000)  /* ADC数据输出左对齐 */
#define ADC_RIGHT_ALIGN ((uint16_t)0x0001) /* ADC数据输出右对齐 */

#define ADC_DAT0_HTH ((uint16_t)0x0001) /* ADC_DAT0_TH作为上阈值 */
#define ADC_DAT0_LTH ((uint16_t)0x0000) /* ADC_DAT0_TH作为下阈值 */

#define ADC_MCPWM_SEL ((uint16_t)0x0000)  /* MCPWM 触发ADC采样 */
#define ADC_UTIMER_SEL ((uint16_t)0x1000) /* UTIMER触发ADC采样 */

#define ADC_1SEG_TRG ((uint16_t)0x00) /* ADC采用单段模式 */
#define ADC_2SEG_TRG ((uint16_t)0x01) /* ADC采用2段模式 */
#define ADC_4SEG_TRG ((uint16_t)0x03) /* ADC采用4段模式 */

#define ADC_CLK_12M (u8)(0x02) /* 12M ADC时钟频率 */
#define ADC_CLK_24M (u8)(0x03) /* 24M ADC时钟频率 */
#define ADC_CLK_48M (u8)(0)    /* 48M ADC时钟频率 */

#define ADC_CHANNEL_0 ((uint16_t)0x00)  /* ADC通道0  OPA0_OUT*/
#define ADC_CHANNEL_1 ((uint16_t)0x01)  /* ADC通道1  ADC_CH1*/
#define ADC_CHANNEL_2 ((uint16_t)0x02)  /* ADC通道2  ADC_CH2*/
#define ADC_CHANNEL_3 ((uint16_t)0x03)  /* ADC通道3  ADC_CH3*/
#define ADC_CHANNEL_4 ((uint16_t)0x04)  /* ADC通道4  ADC_CH4*/
#define ADC_CHANNEL_5 ((uint16_t)0x05)  /* ADC通道5  ADC_CH5*/
#define ADC_CHANNEL_6 ((uint16_t)0x06)  /* ADC通道6  ADC_CH6*/
#define ADC_CHANNEL_7 ((uint16_t)0x07)  /* ADC通道7  ADC_CH7*/
#define ADC_CHANNEL_8 ((uint16_t)0x08)  /* ADC通道8  ADC_CH8/OPA1_OUT*/
#define ADC_CHANNEL_9 ((uint16_t)0x09)  /* ADC通道9  ADC_CH9*/
#define ADC_CHANNEL_10 ((uint16_t)0x0a) /* ADC通道10 ADC_CH10*/
#define ADC_CHANNEL_11 ((uint16_t)0x0b) /* ADC通道11 温度传感器*/
#define ADC_CHANNEL_12 ((uint16_t)0x0c) /* ADC通道12 内部地*/
#define ADC_CHANNEL_13 ((uint16_t)0x0d) /* ADC通道13 2.4V基准源*/

#define ADC_HARDWARE_T0_TRG ((uint16_t)0x01) /* ADC采用硬件T0事件触发 */
#define ADC_HARDWARE_T1_TRG ((uint16_t)0x02) /* ADC采用硬件T1事件触发 */
#define ADC_HARDWARE_T2_TRG ((uint16_t)0x04) /* ADC采用硬件T2事件触发 */
#define ADC_HARDWARE_T3_TRG ((uint16_t)0x08) /* ADC采用硬件T3事件触发 */

void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct);

#endif /*_CONNIE_ADC_H_ */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
