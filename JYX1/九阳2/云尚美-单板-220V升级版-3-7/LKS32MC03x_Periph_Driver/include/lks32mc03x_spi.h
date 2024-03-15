/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_spi.h
 * 文件标识：
 * 内容摘要： SPI驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Yangzj
 * 完成日期： 21/11/10
 *
 *******************************************************************************/
 
#ifndef __lks32mc03x_SPI_H
#define __lks32mc03x_SPI_H


/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"

typedef enum
{
   SPI_Master = 0x01,
   SPI_Slave  = 0x00
}SPI_Mode;

typedef enum
{
   SPI_Full    = 0x0,           //全双工
   SPI_Half_Tx = 0x2,           //半双工，仅发送
   SPI_Half_Rx = 0x3            //半双工，仅接受
}SPI_Duplex;

typedef struct
{
   __IO uint32_t CFG;
   __IO uint32_t IE;
   __IO uint32_t BAUD;
   __IO uint32_t TX_DATA;
   __IO uint32_t RX_DATA;
   __IO uint32_t SIZE;
}SPI_TypeDef;

typedef struct
{
   SPI_Duplex Duplex;       //半双工模式设置
   uint8_t    CS;           //SPI 从设备下，片选信号来源。0:Slave 模式下，片选信号恒为有效值 1:Slave 模式下，片选信号来自 Master 设备
   SPI_Mode   Mode;         //SPI 主从模式选择。
   uint8_t    CPHA;         //SPI 相位选择。
   uint8_t    CPOL;         //SPI 极性选择。
   uint8_t    ENDIAN;       //SPI 模块传输顺序。
   uint8_t    EN;           //SPI Enable Signal, 0: disable;1: enable

   uint8_t    IRQEna;       //SPI 中断使能。
   uint8_t    TRANS_TRIG;   //传输触发选择。

   uint8_t    TRANS_MODE;   //SPI 数据搬移方式。0 DMA 方式。
   uint8_t    BAUD;         //SPI 传输波特率配置, SPI 传输速度 = 系统时钟 / (2*(BAUD + 1)),最小为3

   uint8_t    BITSIZE;      //字节长度寄存器。8-16
}SPI_InitTypeDef;

#define SPI_FIRSTSEND_LSB 1
#define SPI_FIRSTSEND_MSB 0

#define SPI_DMA_ENABLE  0
#define SPI_DMA_DISABLE 1

/*中断使能定义*/
#define SPI_IRQEna_Enable              BIT7            //SPI Interrupt Enable
#define SPI_IRQEna_TranDone            BIT6            //Transmit Done IRQ
#define SPI_IRQEna_SSErr               BIT5            //SS Signal Error IRQ
#define SPI_IRQEna_DataOver            BIT4            //Transmit Data Over Flow IRQ

/*中断标志定义*/
#define SPI_IF_TranDone                BIT2            //Transmit Done IF
#define SPI_IF_SSErr                   BIT1            //SS Signal Error IF
#define SPI_IF_DataOver                BIT0            //Transmit Data Over Flow IF

void SPI_Init(SPI_TypeDef *SPIx, SPI_InitTypeDef *SPI_InitStruct);
void SPI_StructInit(SPI_InitTypeDef *SPI_InitStruct);

uint8_t SPI_GetIRQFlag(SPI_TypeDef *SPIx);

void    SPI_SendData(SPI_TypeDef *SPIx, uint8_t n);
uint8_t SPI_ReadData(SPI_TypeDef *SPIx);
#endif /*__lks32mc03x_SPI_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
