/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Main.c
 * 文件标识：
 * 内容摘要： 工程主代码
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet Li
 * 完成日期： 2020年8月5日
 *
 * 修改记录1：
 * 修改日期：2020年8月16日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet Li
 * 修改内容：创建
 *
 *******************************************************************************/
#include "hardware_config.h"
#include "Global_Variable.h"
#include "USER_APP.h"
void Hardware_init(void);
void Task_Scheduler(void);
void sys_init(void);

extern u16 g_AngleComp;

/*******************************************************************************
 函数名称：    int main(void)
 功能描述：    主程序入口
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
int main(void)
{
    SYS_DBG_CFG |= BIT14; //soft reset peripheral regs
	
	USER_GPIO_Init();
	
	Hardware_init();      /* 硬件初始化 */

    sys_init();           /* 系统初始化 */

    USER_Init();    
    
	g_AngleComp = 2500;
	
    __enable_irq();                   /* 开启总中断 */
//    StateInit();    
	   for(;;)
    {
        Task_Scheduler(); /* 任务调度函数，根据时间分片处理 */
    }
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

