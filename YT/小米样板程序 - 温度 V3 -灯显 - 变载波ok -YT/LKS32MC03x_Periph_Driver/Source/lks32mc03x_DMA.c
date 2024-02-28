/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： LKS32MC03x_DMA.c
 * 文件标识：
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Yangzj
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/
#include "lks32mc03x_DMA.h"
#include "string.h"

/*******************************************************************************
 函数名称：    void DMA_StructInit(DMA_InitTypeDef *DMAInitStruct)
 功能描述：    DMA结构体初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021.10.14      V1.0           Yangzj              创建
 *******************************************************************************/
void DMA_StructInit(DMA_InitTypeDef *DMAInitStruct)
{
    memset(DMAInitStruct, 0, sizeof(DMA_InitTypeDef));
}

/*******************************************************************************
 函数名称：    void DMA_Init(DMA_RegTypeDef *DMAx, DMA_InitTypeDef *DMAInitStruct)
 功能描述：    DMA初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021.10.14      V1.0           Yangzj              创建
 *******************************************************************************/
void DMA_Init(DMA_RegTypeDef *DMAx, DMA_InitTypeDef *DMAInitStruct)
{
    DMA_CTRL       = 0x0001;
    /* 通道配置寄存器 斌值 */
    DMAx->DMA_CCR  = 0;
    DMAx->DMA_CTMS = DMAInitStruct->DMA_TIMES; /* 传输次数寄存器 */
    DMAx->DMA_SADR = DMAInitStruct->DMA_SADR;   /* 外设地址寄存器 */
    DMAx->DMA_DADR = DMAInitStruct->DMA_DADR;   /* 内存地址寄存器 */
    DMAx->DMA_REN  = DMAInitStruct->DMA_REQ_EN;
    DMAx->DMA_CCR  = (DMAInitStruct -> DMA_SBTW  << 10) | (DMAInitStruct -> DMA_DBTW  <<8) |
                     (DMAInitStruct -> DMA_SINC  <<  6) | (DMAInitStruct -> DMA_DINC  <<4) |
                     (DMAInitStruct -> DMA_CIRC  <<  3) | (DMAInitStruct -> DMA_RMODE <<1) |
                     (DMAInitStruct -> DMA_Channel_EN);
    if(DMAInitStruct->DMA_IRQ_EN)
    switch((u32)DMAx)
    {
        case (u32)DMA_CH0:
            DMA_IE |= BIT0;
            break;
        case (u32)DMA_CH1:
            DMA_IE |= BIT1;
            break;
        case (u32)DMA_CH2:
            DMA_IE |= BIT2;
            break;
        case (u32)DMA_CH3:
            DMA_IE |= BIT3;
            break;
    }
    DMA_CTRL  = 0x0001;  /*enable dma, mcu has higher priorit */
}  

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/

