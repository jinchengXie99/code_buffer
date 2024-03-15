/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_sys.h
 * 文件标识：
 * 内容摘要： SYS外设驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/
 
#ifndef __lks32mc03x_SYS_H
#define __lks32mc03x_SYS_H


/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"
#include "lks32mc03x_gpio.h"

typedef struct
{
	uint32_t PLL_SrcSel;                      /*PLL信号输入源选择，RC时钟或外部晶体*/
	uint32_t PLL_DivSel;                      /*选择8个时钟周期中，哪个周期输出时钟*/
	uint32_t PLL_ReDiv;                       /*PLL再分频，1分频或4分频*/

	uint32_t Clk_DivI2C;                      /*SPI I2C时钟分频*/
	uint32_t Clk_DivUART;                     /*UART时钟分频*/
	uint32_t Clk_FEN;                         /*模块时钟使能*/

}SYS_InitTypeDef;

/*PLL信号输入源选择*/
#define SYS_PLLSRSEL_RCH        0x00          /*使用4MHz RC时钟*/
#define SYS_PLLSRSEL_CRYSTAL    0x01          /*使用晶体时钟*/

/*PLL再分频定义*/
#define SYS_PLLREDIV_4			0x00          /*在PLL分频基础上再/4*/
#define SYS_PLLREDIV_1			0x01          /*在PLL分频基础上不再分频*/

/*SPI时钟分频*/
#define SYS_Clk_SPIDiv1          0
#define SYS_Clk_SPIDiv2          1
#define SYS_Clk_SPIDiv4          2
#define SYS_Clk_SPIDiv8          3

/*UART时钟分频*/
#define SYS_Clk_UARTDiv1         0
#define SYS_Clk_UARTDiv2         1
#define SYS_Clk_UARTDiv4         2
#define SYS_Clk_UARTDiv8         3

/*数字模块位定义*/
#define SYS_Module_I2C          BIT0   //I2C
#define SYS_Module_HALL         BIT1   //HALL
#define SYS_Module_UART         BIT2   //UART
#define SYS_Module_CMP          BIT3   //CMP
#define SYS_Module_MCPWM        BIT4   //MCPWM
#define SYS_Module_TIMER0       BIT5   //TIMER0
#define SYS_Module_TIMER1       BIT6   //TIMER1
#define SYS_Module_GPIO         BIT7   //GPIO
#define SYS_Module_DIV          BIT8   //DIV
#define SYS_Module_ADC          BIT9   //ADC
#define SYS_Module_SPI          BIT10  //SPI
#define SYS_Module_DMA          BIT11  //DMA

/*模拟模块定义*/
#define SYS_AnalogModule_ADC    BIT8   // ADC
#define SYS_AnalogModule_OPA    BIT9   // OPA
#define SYS_AnalogModule_BGP    BIT10  // BGP
#define SYS_AnalogModule_DAC    BIT11  // DAC
#define SYS_AnalogModule_TMP    BIT12  // TMP
#define SYS_AnalogModule_CMP0   BIT13  // CMP0
#define SYS_AnalogModule_CMP1   BIT14  // CMP1
#define SYS_AnalogModule_PLL    BIT15  // PLL

/*看门狗超时时间*/
#define SYS_WD_TimeOut2s  0         /*2s复位*/
#define SYS_WD_TimeOut4s  1         /*4s复位*/
#define SYS_WD_TimeOut8s 2         /*8s复位*/
#define SYS_WD_TimeOut64s 3         /*64s复位*/

/*复位信号源定义*/
#define SYS_RstSrc_LPOR        0x01
#define SYS_RstSrc_HPOR        0x02
#define SYS_RstSrc_KEYBOARD    0x04     /*按键复位*/
#define SYS_RstSrc_WDT         0x08     /*WDT复位*/

/*休眠唤醒间隔时间*/
#define SYS_WakeInterval_025s  0x00     /*0.25s*/    
#define SYS_WakeInterval_05s   0x01     /*0.5s*/
#define SYS_WakeInterval_1s    0x02     /*1s*/
#define SYS_WakeInterval_2s    0x03     /*2s*/
#define SYS_WakeInterval_4s    0x04     /*4s*/
#define SYS_WakeInterval_8s    0x05     /*8s*/
#define SYS_WakeInterval_16s   0x06     /*16s*/
#define SYS_WakeInterval_32s   0x07     /*32s*/


void SYS_Init(SYS_InitTypeDef* SYS_InitStruct);         // sys模块初始化
void SYS_StructInit(SYS_InitTypeDef* SYS_InitStruct);   // SYS结构体初始化
void SYS_ClearRst(void);                                // SYS清除复位标志
void SYS_ModuleClockCmd(uint32_t,FuncState);            // 数字模块时钟使能和停止
void SYS_AnalogModuleClockCmd(uint32_t, FuncState);     // 模拟模块使能和停止
void SYS_SoftResetModule(uint32_t nModule);             // 模块软复位

u32 SYS_GetRstSource(void);                             // 获得SYS复位源信号

#endif /*__lks32mc03x_SYS_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
