/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Global_Variable.h
 * 文件标识：
 * 内容摘要： 全局变量声明文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet
 * 完成日期： 2020年8月5日
 *
 * 修改记录1：
 * 修改日期：2020年8月5日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet
 * 修改内容：创建
 *
 *******************************************************************************/

#ifndef __GLOBAL_VARIABLE__
#define __GLOBAL_VARIABLE__

#include "basic.h"
#include "function_config.h"
#include "foc_drive.h" 
#include "state_machine.h"
#include "pmsmFluxObserve.h"
#include "MC_IPD.h"
#include "mc_hall.h"

extern const char sVersion[10];                      /* 程序版本 */
                                                  
extern stru_TaskSchedulerDef struTaskScheduler;      /* 任务调度结构体 */

extern stru_FOC_CtrProcDef struFOC_CtrProc;          /* FOC电流内环结构体 */
extern stru_FOC_CurrLoopDef struFOC_CurrLoop;        /* 电流内环结构体 */

extern stru_CommData struAppCommData;                /* 运行流程控制结构体 */

extern stru_FluxOB_ParameterDef struFluxOB_Param;    /* 观测器参数结构体 */

extern stru_MotorParameterDef struMotorParam;        /* 电机参数结构体 */

extern stru_OpenForceRunDef mOpenForceRun;           /* 强拖运行结构体 */
 
extern stru_BoardParameterDef struBoardParam;        /* 功率板硬件电路特性参数 */ 

extern stru_HallProcessDef struHallProcess;

extern stru_TransCoef struUser2Core, struUser2App;   /* 标幺化处理 */
extern stru_TransCoef struCore2App, struApp2Core;    /* 标幺化处理 */

extern stru_IPD_CtrProcDef mIPD_CtrProc;             /* IPD 转子位置估计结构体 */
extern stru_HallProcessDef  struHallProcess;         /* Hall 信号处理结构体 */

extern Stru_FOC_ConstParame struFOC_ConstParam;      /* FOC 控制 常量定义 */
extern MechanicalQuantity struMotorSpeed;            /* 转速矢量结构体，core */

extern void DebugPWM_OutputFunction(void);
extern stru_FluxOBS_Def m_FluxOBSPara;               /* 观测器参数 */

#endif

/* ********************** (C) COPYRIGHT LINKO SEMICONDUCTOR ******************** */
/* ------------------------------END OF FILE------------------------------------ */
