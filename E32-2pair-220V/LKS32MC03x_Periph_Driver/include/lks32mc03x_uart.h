/*******************************************************************************
 * 版权所有 (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_uart.h
 * 文件标识：
 * 内容摘要： UART驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   HMG
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/

#ifndef __lks32mc03x_UART_H
#define __lks32mc03x_UART_H

/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"

typedef enum
{
   UART_Parity_NO = 0x00,
   UART_Parity_EVEN = 0x01,
   UART_Parity_ODD = 0x02
} UART_ParityMode;

typedef struct
{
   __IO uint32_t CTRL;
   __IO uint32_t DIVH;
   __IO uint32_t DIVL;
   __IO uint32_t BUFF;
   __IO uint32_t ADR;
   __IO uint32_t STT;
   __IO uint32_t RE;
   __IO uint32_t IE;
   __IO uint32_t IF;
   __IO uint32_t INV;
} UART_TypeDef;

typedef struct
{
   uint32_t BaudRate;          /*波特率*/
   uint8_t WordLength;         /*数据长度*/
   uint8_t StopBits;           /*停止位长度*/
   uint8_t FirstSend;          /*First bit to be sent,0:LSB 1:MSB*/
   UART_ParityMode ParityMode; /*奇偶校验,None,EVEN,ODD*/

   uint8_t MultiDropEna; /*使能Multi-drop,0:Disable 1:Enable*/

   uint16_t Match485Addr; /*用作485通信时的匹配地址*/
   uint8_t IRQEna;        /*中断使能寄存器，具体含义*/
   uint8_t DMARE;         /*DMA 请求使能，具体含义*/
   uint8_t RXD_INV;       /* 接收电平取反 */
   uint8_t TXD_INV;       /* 发送电平取反 */
} UART_InitTypeDef;

#define UART_WORDLENGTH_8b 0
#define UART_WORDLENGTH_9b 1

#define UART_STOPBITS_1b 0
#define UART_STOPBITS_2b 1

#define UART_FIRSTSEND_LSB 0
#define UART_FIRSTSEND_MSB 1

/*中断使能定义*/
#define UART_IRQEna_SendOver BIT0      /*使能发送完成中断*/
#define UART_IRQEna_RcvOver BIT1       /*使能接收完成中断*/
#define UART_IRQEna_SendBuffEmpty BIT2 /*使能发送缓冲区空中断*/
#define UART_IRQEna_StopError BIT3     /*使能停止位错误*/
#define UART_IRQEna_CheckError BIT4    /*使能校验错误*/

/*中断标志定义*/
#define UART_IF_SendOver BIT0     /*发送完成中断*/
#define UART_IF_RcvOver BIT1      /*接收完成中断*/
#define UART_IF_SendBufEmpty BIT2 /*发送缓冲区空中断*/
#define UART_IF_StopError BIT3    /*停止位错误*/
#define UART_IF_CheckError BIT4   /*校验错误 */

/*DMA请求模式定义*/
#define UART_TX_DONE_RE BIT0      /*发送缓冲区空DMA请求开关*/
#define UART_RX_DONE_RE BIT1      /*接收完成DMA请求开关*/
#define UART_TX_BUF_EMPTY_RE BIT2 /*发送完成DMA请求开关*/

void UART_Init(UART_TypeDef *UARTx, UART_InitTypeDef *UART_InitStruct);
void UART_StructInit(UART_InitTypeDef *UART_InitStruct);

void UART_SendData(UART_TypeDef *UARTx, uint32_t n);
uint32_t UART_ReadData(UART_TypeDef *UARTx);

void UART_SendnData(UART_TypeDef *UARTx, char *sendData, uint32_t len);
void UART_RecvnData(UART_TypeDef *UARTx, char *recvData, uint32_t maxlen);

uint32_t UART_GetIRQFlag(UART_TypeDef *UARTx);
void UART_ClearIRQFlag(UART_TypeDef *UARTx, uint32_t tempFlag);

#endif /*__lks32mc03x_UART_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
