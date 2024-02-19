/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Global_Variable.c
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

#include "Global_Variable.h"
#include "time_process.h"
#include "MC_Parameter.h"

const char sVersion[10] = "Ver_3.9";        /* 程序版本 */

stru_TaskSchedulerDef struTaskScheduler;    /* 任务调度结构体 */

stru_FOC_CtrProcDef struFOC_CtrProc;        /* FOC电机控制流程结构体 */

stru_FOC_CurrLoopDef struFOC_CurrLoop;      /* 电流内环结构体 */

stru_FluxOB_ParameterDef struFluxOB_Param;  /* 观测器参数结构体 */

stru_CommData struAppCommData;              /* 运行流程控制结构体 */

MechanicalQuantity struMotorSpeed;          /* 转速矢量结构体，core */

Stru_OnTheFlyDetect mOnTheFlyDetect;        /* 顺逆风启动结构体 */

stru_HallProcessDef struHallProcess;

stru_PIRegulator mSpeedPI;                  /* 速度环 */

stru_OpenForceRunDef mOpenForceRun;         /* 强拖运行结构体 */

stru_TransCoef struUser2App;                /* 标幺化处理 */
stru_TransCoef struCore2App, struApp2Core;  /* 标幺化处理 */

//stru_HallProcessDef  struHallProcess;       /* Hall 信号处理结构体 */

stru_IPD_CtrProcDef mIPD_CtrProc;           /* IPD 转子位置估计结构体 */

/* FOC 控制 常量定义 */
Stru_FOC_ConstParame struFOC_ConstParam = FOC_CONST_PARAMETER;

/* 电机参数结构体 */
stru_MotorParameterDef struMotorParam = MOTOR_PARAMETER_TAB;
/* 功率板硬件电路特性参数 */
stru_BoardParameterDef struBoardParam = BOARD_PARAMETER_TAB;




/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
