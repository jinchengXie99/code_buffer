/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： state_machine.h
 * 内容摘要： state machine
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2019年11月19日
 *
 *******************************************************************************/
#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "basic.h"
#include "mc_type.h"

typedef struct 
{
    s32 wStartCurSet1;              //第一段定位电流
    s32 wStartCurSet2;              //第二段定位电流
    s32 wSvcMinFreq;                //开环拖动频率
    u16 nStartAngleComp;            //初始位置补偿角度

    u8  bChargeFlag;                //预充电标志位
    u8  bChargeEndFlag;             //预充电完成标志位

    s16 nDirTrackFreq;              //电机切入闭环频率
    s16 nDirEbreakFreq;             //电机刹车频率
    s16 nCurrentACC;                //电流加速调整值
    s16 nCurrentDEC;                //电流减速调整值
    s32 wSpeedRef;                  //速度给定值
    
    s16 nLoopCntr;                  //环路计数

    u8  bOpenRunFlag;               //开环运行标志位
    s32 wThetaErr;                  //估算和给定角度误差
    s16 nMatchCnt;                  //估算角度匹配计数

    s16 nPowerFistFlag;             //第一次进入功率环标志位

    s16 nTempADC;                   //NTC采样AD值
    
    u16 nMotorRunDirPre;            //上次电机转向
    
    s32 wPWMPowerSet;              /* PWM调速给定功率 */
    
    s32 wPWMSpeedSet;              /* PWM调速给定转速 */
    
    s16 nVspADC;                   /* VSP采样电压 */
    s32 wVSPSpeedSet;              /* VSP采样给定转速 */
    
    s32 wPIMatchSpeed;           /* 电流环和PLL参数调整转速*/
    
    s32 wCloseSpeed;          /* 切入功率环的频率 */
}stru_CommData;

typedef struct
{
    s16 nChargeTime;       //充电时间，以1ms为单位
    s16 nAlignTime;        //定位时间，以1ms为单位
    s16 nDirCheckTime;     //顺逆风时间，以1ms为单位
    s16 nLoopDelyTime;     //速度环延迟时间，以1ms为单位
    s16 nStopDelayTime;    //电机停止延迟时间，以1ms为单位
    s16 nStopDelayCntr;    //电机停止滤波时间，即连续判断电机停止的时间，以1ms为单位
    s16 nOpenRunCntr;      //开环切闭环时间，以1ms为单位
}stru_Time_t;

extern stru_Time_t                 struTime;

extern void StateInit(void);
extern void StateCalib(void);
extern void StateCharge(void);
extern void StateDirCheck(void);
extern void StatePosSeek(void);
extern void StateAlign(void);
extern void StateOpen(void);
extern void StateRun(void);
extern void StateFault(void);
extern void StateIdle(void);
extern void StateStop(void);

extern void FluxObserveInit(void);
extern void MatchPICoef(void);
#endif

