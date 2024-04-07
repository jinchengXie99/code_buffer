/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Time_Process.h
 * 文件标识：
 * 内容摘要： 定时相关函数
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet Li
 * 完成日期： 2020年8月16日
 *
 * 修改记录1：
 * 修改日期：2020年8月16日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet Li
 * 修改内容：创建
 *
 *******************************************************************************/
/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __TIME_PROCESS_H
#define __TIME_PROCESS_H

#include "basic.h"
#include "Global_Variable.h"

#define PWM_TIME_500uS                    (u16)(PWM_FREQ/1000/2)
#define PWM_TIME_1MS                      (u16)(PWM_FREQ/1000)
#define PWM_TIME_2MS                      (u16)(2*PWM_FREQ/1000)
#define PWM_TIME_4MS                      (u16)(4*PWM_FREQ/1000)
#define PWM_TIME_10MS                     (u16)(20*PWM_FREQ/1000)
#define PWM_TIME_20MS                     (u16)(20*PWM_FREQ/1000)

#define TASK_SCHEDU_05MS                  (1)                                      /* 任务调度0.5ms计数时长 */
#define TASK_SCHEDU_1MS                   (10)                                      /* 任务调度1ms计数时长 */
#define TASK_SCHEDU_10MS                  (100)                                     /* 任务调度1ms计数时长 */
#define TASK_SCHEDU_100MS                 (1000)                                    /* 任务调度100ms计数时长 */
#define TASK_SCHEDU_500MS                 (5000)                                   /* 任务调度500ms计数时长 */

#define TIMER_PERIOD                      (u32)500                                 /* 定时500us, 单位us */
#define TIMER_500US_TH                    (u32)(MCU_MCLK/1000000*TIMER_PERIOD - 1) /* MCU_MCLK/1000000 * PERIOD  */

#define HALL_OVERTIME_1MS                 (MCU_MCLK/1000)                          /* Hall模块超时1ms, 分频系数为1 */
#define HALL_OVERTIME_100MS               (u32)(HALL_OVERTIME_1MS*100)             /* Hall模块超时100ms  */

#define STATE_MACHINE_200MS               (200)                                    /* 状态机200ms计数时长 */
#define STATE_MACHINE_10MS                (10)                                     /* 状态机10ms计数时长 */

/* Includes ------------------------------------------------------------------*/
/*******************************************************************************
 函数名称：    void SetTimeOut_Counter(u16 hTimeout)
 功能描述：    设置等待函数等待时间
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
#define SetTimeOut_Counter(SetTimeLeftCnt, hTimeout)  {SetTimeLeftCnt = hTimeout;}


/*******************************************************************************
 函数名称：    u8 SetTime_IsElapsed(void)
 功能描述：    判断等待时间是否到
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
#define SetTime_IsElapsed(SetTimeLeftCnt)     (SetTimeLeftCnt)

/*******************************************************************************
 函数名称：    void SetTime_CountDown(void)
 功能描述：    倒计数函数计时处理
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
static __inline void SetTime_CountDown(void)
{
    if(struFOC_CtrProc.nSetTimeLeftCnt)
    {   /* 状态机等待函数等待时间按1mS递减 */
        struFOC_CtrProc.nSetTimeLeftCnt--;
    }
}

void SoftDelay(u32 cnt);

#endif
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
