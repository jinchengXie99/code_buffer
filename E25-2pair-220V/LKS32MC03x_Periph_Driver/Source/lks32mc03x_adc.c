/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_adc.c
 * 文件标识：
 * 内容摘要： ADC外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   HMG
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/
#include "lks32mc03x_adc.h"
#include "lks32mc03x.h"
#include "lks32mc03x_sys.h"

/*******************************************************************************
 函数名称：    void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
 功能描述：    ADC初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0             HMG               创建
 *******************************************************************************/
void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct)
{

    uint16_t t_reg;

    SYS_AnalogModuleClockCmd(SYS_AnalogModule_ADC, ENABLE); // ADC模块使能

    ADCx->IE = ADC_InitStruct->IE;

    t_reg = (ADC_InitStruct->Align << 10) | (ADC_InitStruct->Trigger_En) | (ADC_InitStruct->SEL_En << 12) |
            (ADC_InitStruct->Trigger_Cnt << 4) | (ADC_InitStruct->Trigger_Mode << 8);
    ADCx->CFG = t_reg;

    ADCx->CHNT = ADC_InitStruct->FirSeg_Ch | (ADC_InitStruct->SecSeg_Ch << 4) |
                  (ADC_InitStruct->ThrSeg_Ch << 8) | (ADC_InitStruct->FouSeg_Ch << 12);
}

/*******************************************************************************
 函数名称：    void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
 功能描述：    ADC结构体初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：     无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0             HMG               创建
 *******************************************************************************/
void ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct)
{
    ADC_InitStruct->IE = 0;
    ADC_InitStruct->Align = 0;
    ADC_InitStruct->Con_Sample = 0;
    ADC_InitStruct->Trigger_Cnt = 0;
    ADC_InitStruct->FirSeg_Ch = 0;
    ADC_InitStruct->SecSeg_Ch = 0;
    ADC_InitStruct->ThrSeg_Ch = 0;
    ADC_InitStruct->FouSeg_Ch = 0;
    ADC_InitStruct->Trigger_Mode = 0;
    ADC_InitStruct->Trigger_En = 0;
    ADC_InitStruct->SEL_En = 0;
    ADC_InitStruct->ADC_GEN_En = 0;
    ADC_InitStruct->ADC_GEN_HTH = 0;
    ADC_InitStruct->ADC_GEN_LTH = 0;
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
