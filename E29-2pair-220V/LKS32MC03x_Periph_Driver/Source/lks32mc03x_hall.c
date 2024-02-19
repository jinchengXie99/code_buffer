/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_hall.c
 * 文件标识： 
 * 内容摘要： HALL驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   HMG
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/
#include "lks32mc03x_hall.h"
#include "lks32mc03x.h"
#include "lks32mc03x_sys.h"

/*******************************************************************************
 函数名称：    void HALL_Init(HALL_InitTypeDef* HALL_InitStruct)
 功能描述：    HALL初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
void HALL_Init(HALL_InitTypeDef *HALL_InitStruct)
{

   SYS_ModuleClockCmd(SYS_Module_HALL, ENABLE); //HALL时钟使能
   HALL_CFG = (HALL_InitStruct->FilterLen) | (HALL_InitStruct->ClockDivision << 16) 
              | (HALL_InitStruct->Filter75_Ena << 20) | (HALL_InitStruct->HALL_Ena << 24) 
              | (HALL_InitStruct->HALL_CHGDMA_Ena << 25) | (HALL_InitStruct->HALL_OVDMA_Ena << 26) 
              | (HALL_InitStruct->Capture_IRQ_Ena << 28) | (HALL_InitStruct->OverFlow_IRQ_Ena << 29) 
              | (HALL_InitStruct->softIE << 30);
   HALL_TH = HALL_InitStruct->CountTH;
   HALL_INFO = 0;
}

/*******************************************************************************
 函数名称：    void HALL_StructInit(HALL_InitTypeDef* HALL_InitStruct)
 功能描述：    HALL结构体初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
void HALL_StructInit(HALL_InitTypeDef *HALL_InitStruct)
{
   HALL_InitStruct->FilterLen = 1023;
   HALL_InitStruct->ClockDivision = HALL_CLK_DIV1;
   HALL_InitStruct->Filter75_Ena = ENABLE;
   HALL_InitStruct->HALL_Ena = ENABLE;
   HALL_InitStruct->Capture_IRQ_Ena = ENABLE;
   HALL_InitStruct->HALL_CHGDMA_Ena = DISABLE;
   HALL_InitStruct->HALL_OVDMA_Ena = DISABLE;
   HALL_InitStruct->OverFlow_IRQ_Ena = DISABLE;
   HALL_InitStruct->CountTH = 1000;
   HALL_InitStruct->softIE = DISABLE;
}

/*******************************************************************************
 函数名称：    uint32_t HALL_GetFilterValue(void)
 功能描述：    取得HALL值，滤波结果
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
uint32_t HALL_GetFilterValue(void)
{
   return (HALL_INFO & 0x07);
}

/*******************************************************************************
 函数名称：    uint32_t HALL_GetCaptureValue(void)
 功能描述：    取得HALL值，发生HALL事件时的锁存结果
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
uint32_t HALL_GetCaptureValue(void)
{
   return (HALL_INFO >> 8) & 0x07;
}

/*******************************************************************************
 函数名称：    uint32_t HALL_GetCount(void)
 功能描述：    取得HALL计数器值
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
uint32_t HALL_GetCount(void)
{
   return HALL_CNT;
}

/*******************************************************************************
 函数名称：    uint32_t HALL_IsCaptureEvent(void)
 功能描述：    是否发生HALL信号变化捕获事件
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
uint32_t HALL_IsCaptureEvent(void)
{
   return (HALL_INFO & HALL_CAPTURE_EVENT);
}

/*******************************************************************************
 函数名称：    uint32_t HALL_IsOverFlowEvent(void)
 功能描述：    是否发生HALL计数器溢出事件
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
uint32_t HALL_IsOverFlowEvent(void)
{
   return (HALL_INFO & HALL_OVERFLOW_EVENT);
}

/*******************************************************************************
 函数名称：    void HALL_Clear_IRQ(void)
 功能描述：    通过对INFO地址进行写操作清空
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
2021/10/14    V1.0            HMG               创建
 *******************************************************************************/
void HALL_Clear_IRQ(void)
{
   uint32_t temp = HALL_INFO & 0xFFFCFFFF;
   HALL_INFO = temp;
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
