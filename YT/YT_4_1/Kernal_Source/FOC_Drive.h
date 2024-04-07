/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： FOC_Drive.h
 * 文件标识：
 * 内容摘要： 电机矢量控制电流内环
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_FOC_DRIVE_H
#define __MC_FOC_DRIVE_H

#include "mc_type.h"
#include "PID_regulators.h"
#include "function_config.h"
#include "pmsmFluxObserve.h"

typedef struct
{
    void (*ADC_A_B_Set)(void); /* ADC相电流采样时序 */
    void (*ADC_A_C_Set)(void); /* ADC相电流采样时序 */
    void (*ADC_B_C_Set)(void); /* ADC相电流采样时序 */
} Stru_ADC_SampleDef;


typedef const struct
{
    u16 MIN_PWMSampTime3Shunt;    /* 下桥电阻采样时,三相中最大占空比相,满足ADC可靠采样的最小的PWM值 */
    u16 SampStableTime3Shunt;     /* 下桥电阻采样时,需要保证ADC有正确采样窗口，判断两相PWM占空之间的差值是否满足采样条件 */
                                     
    s16 ShiftSampTime3Shunt;      /* 三电阻采样移位时间 */
    s16 SampWaitTime3Shunt;       /* 三电阻采样等待时间 */
  
    s16 SampStableTime1Shunt;     /* 单电阻采样，信号上升沿时间 */
    u16 ShiftSampTime1Shunt;      /* 单电阻采样移相时间 */


    u16 nPWM_PERIOD;              /* PWM周期值 */
    u16 nDEADTIMECOMP;            /* 死区补偿值 */

} Stru_FOC_ConstParame, *pStru_FOC_ConstParame;

typedef struct
{
    stru_CurrPhaseUVW mStatCurrUVW;              /* 电机定子A相B相电流  */
    stru_CurrPhaseUVW mLastStatCurrUVW;          /* 上一次电机定子A相B相电流  */
    stru_CurrVoctorDQ mStatCurrDQ;               /* 电机定子Q相D相电流  */
    stru_CurrAlphBeta mStatCurrAlfaBeta;         /* 电机定子alfa beta轴电流  */

    stru_VoltVoctorDQ mStatVoltDQ;               /* 电机定子Q相D相电压  */
    stru_VoltAlphBeta mStatVoltAlfaBeta;         /* 电机定子alfa beta轴电压力 */

    stru_TrigComponents mTrigSinCos;             /* 定子角度sin Cos值 */
    u16 wElectrical_Angle;                       /* 当前转子位置角度值，0~65535对应0~360度 */

    s16 nBusVoltage;                             /* 直流母线电压 */             
                                                                          
    stru_VoltPhaseUVW mVdtComp;                  /* 死区时间补偿电压 */
    stru_VoltPhaseUVW mVoltUVW_PWM;              /* UVW PWM输出电压 */
    stru_VoltPhaseUVW mVoltUVW_NegPWM;           /* UVW PWM移相输出电压 */
  
    u16 nVoltageCircleLim;                       /* 电压极限圆限制 单位：Q12 最大值4096 */  

    u8 bSampCur0;                                /* 当前电流采样通道标志 */
    u8 bSampCur1;                                /* 当前电流采样通道标志 */
    u8 bSampCur2;                                /* 当前电流采样通道标志 */

    u16 nADC_TrigPoint0;                         /* ADC 触发点设置0 */
    u16 nADC_TrigPoint1;                         /* ADC 触发点设置1 */
    
    u16 ADC_A_B_CurrentSamp;                     /* ADC相采样时序 */
    u16 ADC_A_C_CurrentSamp;                     /* ADC相采样时序 */
    u16 ADC_B_C_CurrentSamp;                     /* ADC相采样时序 */

    s16 nPhaseAOffset;                           /* A相ADC采样 Offset值 */
    s16 nPhaseBOffset;                           /* B相ADC采样 Offset值 */
    s16 nPhaseCOffset;                           /* C相ADC采样 Offset值 */
    s16 nBusShuntOffset;                         /* 直流母线电流ADC采样 Offset值 */
    
    s16 nQCur_Reference;                         /* Q轴电流环参考给定 */
    s16 nDCur_Reference;                         /* D轴电流环参考给定 */
    
    s16 nQCurrentSet;                            /* Q轴电流设定值 */
    s16 nDCurrentSet;                            /* D轴电流设定值 */    

    s16 nRequestPower;                           /* Q轴电压给定 */
    
    s16 nWeakFieldLim;                           /* 弱磁电流限制 */

    s8 bCntFirCurA;                              /* A相电流滤波计数器 */
    s8 bCntFirCurB;                              /* B相电流滤波计数器 */
    stru_RC_Def mCurrA_RC;                       /* A相相电流RC滤波结构体  */
    stru_RC_Def mCurrB_RC;                       /* B相相电流RC滤波结构体  */

    u8 bSVPWM_Type;                              /* PWM 调制类型，1：7段SVPWM,0五段PWM */

    u16 nCurrentSqrtOrg;                         /* 相电流均方根值  */

    stru_PIRegulator mPI_Torque;                 /* Q轴电流环PI参数 */
    stru_PIRegulator mPI_Flux;                   /* D轴电流环PI参数 */

    s16 nMAX_CurrentLimThd;                      /* 最大电流限制值 */

    pStru_FOC_ConstParame pStruFOC_ConstPar;     /* FOC控制Const变量 */
    
    void (*MCPWMx_RegUpdate)(void);              /* 加载PWM寄存器函数 */  
		
    stru_PIRegulator mPI_Torque_BRAKE;                 /* 刹车Q轴电流环PI参数 */
    stru_PIRegulator mPI_Flux_BRAKE;                   /* 刹车D轴电流环PI参数 */

} stru_FOC_CurrLoopDef;


  

typedef struct
{

 
    volatile u16 nSys_TimerPWM;                  /* PWM周期计数Cnt */

    u8 bMotorDirtionCtrl;                        /* 期望控制的电机运行方向 */

    u8 bMC_RunFlg;                               /* 电机启动指令 */
    enum_SystStatus eSysState;                   /* 当前电机运行状态机 */
    u16 nSetTimeLeftCnt;                         /* 状态机等待时间计数器 */

    stru_RC_Def struBusCurr_RC;                  /* 母线电流RC滤波结构体  */
  

} stru_FOC_CtrProcDef;


typedef struct
{
   s32 wMotorSpeedEst;          /* 观测器电机速度 */
   stru_RC_Def mRC_Obs;         /* 观测器低通滤波 */
  
   u16 nObsFactor;              /* 观测器增益系数 */
   u8 bDivisor;                 /* 观测器输出精度，除法系数 */
   s32 wIS2;                    /* 电流向量模值 */
   stru_PIRegulator mPI_Obs;    /* D轴电流环PI参数 */
  
   s8 bMotorDir;                /* 电机转运行方向 */
   u16 nObsCoef;                /* 观测器滤波系数 */
   u16 nMaxOutFreq;             /* 最大观测速度 */
   u16 nMinOutFreq;             /* 最小观测速度 */
}stru_FluxOBS_Def;


#define MAX_DELTA_CUR           20000            /* 相电流滤波，两次ADC电流采样，变化率不应超过1000个ADC Lsb */



#define SAMP_NO               0
#define SAMP_IA               1
#define SAMP_IB               2
#define SAMP_IC               3
#define SAMP_NIA              4
#define SAMP_NIB              5
#define SAMP_NIC              6
#define SAMP_OLDA             7
#define SAMP_OLDB             8
#define SAMP_OLDC             9

#if (ROTOR_SENSOR_TYPE == ROTOR_HALL_SENSOR)  
#define GET_ELECT_ANGLE()     {tElectAngle = struHallProcess.nElectrical_Angle; struFluxOB_Param.wElectAngle = tElectAngle<<16;}
#endif

#if (ROTOR_SENSOR_TYPE == ROTOR_SENSORLESS) 
#define GET_ELECT_ANGLE()     {tElectAngle = struFluxOB_Param.wElectAngle>>16;}
#endif

/* Exported functions ------------------------------------------------------- */
void FOC_Model(stru_FOC_CtrProcDef *pStruFOC_CtrProc);
void MCL_Init(stru_FOC_CtrProcDef *this);
stru_TrigComponents Trig_Functions(s16 hAngle);
void RevPark_Circle_Limitation(stru_VoltVoctorDQ *mStatVolt_q_d, u16 MAX_ModuleCof);

stru_CurrPhaseUVW SVPWM_1ShuntGetPhaseCurrent(stru_FOC_CurrLoopDef *this);
stru_CurrPhaseUVW SVPWM_2ShuntGetPhaseCurrent(stru_FOC_CurrLoopDef *this);
stru_CurrPhaseUVW SVPWM_3ShuntGetPhaseCurrent(stru_FOC_CurrLoopDef *this);

void SVPWM_1ShuntGEN(stru_FOC_CurrLoopDef *this);
void SVPWM_2ShuntGEN(stru_FOC_CurrLoopDef *this);
void SVPWM_3ShuntGEN(stru_FOC_CurrLoopDef *this);

s16 lowPass_filter(stru_RC_Def *rc,s16 signal);

void ADC_SampPhaseA_B_Set(void);
void ADC_SampPhaseA_C_Set(void);
void ADC_SampPhaseB_C_Set(void);


void MCPWM0_RegUpdate(void);
void MCPWM1_RegUpdate(void);

void FOC_InitstruParam(void);

s16 DcVoltPerUnitCalc(s16 sav);
s16 DcVoltPerUnitCalcInt(s16 sav);

void CurrentLoopReg(stru_FOC_CurrLoopDef *this);

void SpeedLoopReg(MechanicalQuantity *this);

void BrakeStateCheck(void);

s32 App2CoreCurTrans(s32 val);
s32 App2CoreVolTrans(s32 val);
s32 App2CoreFreqTrans(s32 val);
s32 App2CoreVdcTrans(s32 val);
u32 App2CoreAngleTrans(u16 val);
s32 Core2AppCurTrans(s32 val);
s32 Core2AppVolTrans(s32 val);
s32 Core2AppFreqTrans(s32 val);
s32 Core2AppVdcTrans(s32 val);
u16 Core2AppAngleTrans(u32 val);
s32 User2AppCurTrans(float val);
s32 User2AppVolTrans(float val);
s32 User2AppFreqTrans(float val);
u16 User2AppAngleTrans(float val);


#endif /* __MC_FOC_DRIVE_H */
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

