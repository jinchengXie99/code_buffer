/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： PmsmFluxObserve.h
 * 文件标识：
 * 内容摘要： 磁通观测相关文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： WenCai Zhao
 * 完成日期： 2020年8月20日
 *
 * 修改记录1：
 * 修改日期： 2020年8月20日
 * 版 本 号： V 2.0
 * 修 改 人： Howlet
 * 修改内容： 创建
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __PMSM_FLUX_OBSERVE_H_
#define __PMSM_FLUX_OBSERVE_H_

#include "mc_type.h"
#include "PID_regulators.h"

typedef struct
{ /* 顺风逆风启动，转子速度位置方向检测 */
    u16 nElectAngle;          /* 电角度 */
    s16 nFreq;                /* 转子电频率 */
    s16 nFreqAvg;             /* 转子电频率平均值 */
    s32 wBemfFreq;            /* 反电势检测滤波值 */
    s32 wVolMagAvg;           /* 当前反电势电压平均值 */
    s16 nMotorVolMag;         /* 当前电机电压给定 */
    s32 wBEMF_DppValueAcc;    /* 速度积分器 */
    volatile s32 wAngleDpp;   /* 角速度值 */
    s16 nPWM1msTick;          /* 以PWM为时基1mS计数值 */
    u8  bMotorStopDecFlag;    /* 电机停转标志 */
    s16 nMotorStopCurrentThd; /* 电机停转电流检测阈值 */
    s16 nMotorStopCurDifThd;  /* 电机停转电流差值检测阈值 */
} Stru_OnTheFlyDetect;


typedef struct {
    s32 wSpeedRef;                     /* 速度给定值 */
    s32 wSpeedfbk;                     /* 速度反馈值 */
    s32 we;                            /* we,omegaE; */
    s32 wSpeedSet;                     /* 速度设定 */
    s32 wSpeedEst;                     /* 观测器角速度输出 */
    u16 nSpeedLoopKP;                  /* 速度估算Kp 值 */
    u16 nSpeedLoopKI;                  /* 速度估算Ki 值 */ 

    stru_SpeedPIRegulator mSpeedPI; /* 速度环 PI */

    s32 wSpeedRampACCStep;             /*速度爬坡加速值*/
    s32 wSpeedRampDECStep;             /*速度爬坡减速值*/
    
    s32 wPowerLimitSpeedRef;           /*限功率转速给定值*/
    s32 wPowerLimitSpeedSet;           /*限功率转速设定值*/
    s32 wPowerLimitValue;              /*限功率大小*/
    s16 nPowerLimitCntr;               /*限功率计算计数*/
	
    s16 nBrakeCurrent;                 /* Q轴刹车电流 */
	  s32 wBrakeSpeed;                   /* 刹车转速 */
	
} MechanicalQuantity;

typedef const struct
{   /* 电机参数结构体 */
    s8  bMotorType;           /* 0:im,1:spm,2:ipm,3...*/
    s16 nRatedPower;          /* 额定功率 0.1W */
    s32 wRatedFreq;           /* 额定频率 单位：0.01Hz*/
    s8  bPolePairs;           /* 电机磁极对数 */
    s16 nRatedCur;            /* 额定电流 单位：0.1A */
    s32 wMotorRs;             /* 电机内阻 单位：0.0001欧姆 */
    s32 wMotorLd;             /* 电机D轴电感 单位：uH */
    s32 wMotorLq;             /* 电机Q轴电感 单位：uH */
    u32 nFluxConst;           /* 电机磁链常数 0.00001WB */
    s32 wMotorRr;             /* 电机转子内阻 单位：0.0001欧姆 */
    s32 wMotorLr;             /* 电机转子电感 单位：uH */
    s32 wMotorlm;             /* 电机互感 单位：uH */
    s16 nNoLoadCur;           /* 电机空载电流 单位：0.1A */
    u16 nRatedTorque;         /* 电机额定转矩 单位：0.001Nm */
} stru_MotorParameterDef, *pStru_MotorParameterDef;

typedef const struct
{   /* 功率板硬件电路特性参数 */
    s16 nRatedUdc;            /* 额定直流母线电压 */
    s16 nMaxUdc;              /* 最大直流母线电压 */
    s16 nMaxCur;              /* 最大相电流 单位：0.1A */
    s16 nDeadTime;            /* 死区时间 单位：ns */ 
    s32 nMaxFreq;             /* 最大工作频率 单位：Hz */
    u16 nPWMFreq;             /* PWM载频 */
  
    float fAdcSupplyVol;      /* ADC参考电压 单位：伏特 */
    float fRShunt;            /* 分压电阻 单位：欧姆*/
    float fAmpGain;           /* 放大器放大倍数 单位：n倍 */ 
    float fVolShuntRatio;     /* 母线电压采样分压比 */ 
    float fBemfShuntRatio;    /* 反电势电压采样分压比 */  

} stru_BoardParameterDef, *pStru_BoardParameterDef;

typedef struct
{ /* 观测器参数变量定义 */
  
   u16 nCurrLoopIniKP;        /* 电流环KP初始值, 程序会根据电机电感电阻修正此参数 */
   u16 nCurrLoopIniKI;        /* 电流环KI初始值, 程序会根据电机电感电阻修正此参数 */ 
	
	 u16 nCurrLoopIniKPL;        /* 电流环KP初始值, 程序会根据电机电感电阻修正此参数 */
   u16 nCurrLoopIniKIL;        /* 电流环KI初始值, 程序会根据电机电感电阻修正此参数 */ 

   u16 nD_CurrLoopKP;         /* D轴电流环KP修正值 */
   u16 nD_CurrLoopKI;         /* D轴电流环KI修正值 */  
   u16 nQ_CurrLoopKP;         /* Q轴电流环KP修正值 */
   u16 nQ_CurrLoopKI;         /* Q轴电流环KI修正值 */   
   
   u16 nSTO_KP;               /* 观测器PLL KP值 */
   u16 nSTO_KI;               /* 观测器PLL KP值 */
	
	
	 u16 nSTO_KP_H;               /* 观测器PLL KP值 */
   u16 nSTO_KI_H;               /* 观测器PLL KP值 */
   
   u32 wElectAngleEst;        /* 估算器电角度 */ 
   u32 wElectAngleOpen;       /* 开环角度 */
   u32 wElectAngle;           /* 当前运行角度 */   

   s32 CurrentPerUnitGAIN;    /* 电流标幺化处处理增益 */
   s32 VDC_PerUnitGAIN;       /* 电压采样标幺化处理增益 */
   s32 FREQ_PerUnit_GAIN;     /* 频率标幺化处理增益 */
   s32 BEMF_PerUnit_GAIN;     /* 反电势标幺化处理增益 */

   /* 电机参数结构体 */
   pStru_MotorParameterDef pStruMotorParame;
   /* 功率板硬件电路特性参数 */  
   pStru_BoardParameterDef pStruBoardParame; 

   u16 nObserveMinSpeed;      /* 观测器最小输出值 */
   u16 nSpeedFiltFac;         /* 速度估算滤波系数 */
  
}stru_FluxOB_ParameterDef;

typedef struct
{ /* 观测器变量定义 */
   s32 wOb_F1;              /* 观感器系数F1 */
   s32 wOb_F2;              /* 观感器系数F2 */
   s32 wOb_F3;              /* 观感器系数F3 */
  
   s8 bSftN_F1;             /* 观感器系数F1 移位数 */
   s8 bSftN_F2;             /* 观感器系数F2 移位数 */
   s8 bSftN_F3;             /* 观感器系数F3 移位数 */

   s32 wOBPllInteg;         /* 观测器积分输出值 */  

   u16 nBemfLowPassFac;     /* 反电势滤波系数  */  
  
   u16 nSpeedFiltFac;       /* 速度估算滤波系数 */
  
   u16 nOpenLoopDppKp;      /* 开环运行模式，速度参考增益 */
  
   u16 nProductPLL;         /* PLL 输出增益系数  */
  
   u16 uBusVolt_PerUnit;    /* 归一后的直流母线电压 */
  
}stru_FluxOB_VariableDef;

struct PIRegFieldW {
    INT32 LIM_H;
    INT32 LIM_L;
    UINT16    KP;
    UINT16    KI;
    INT32    INTEG;
    INT32    inError;//ERROR
};


extern Stru_OnTheFlyDetect mOnTheFlyDetect;

void FluxObserveParaCalc(void);
void ModuCircle_Limitation(void);
void PmsmFluxObserve(void);
void PmsmFluxObserve1(void);
void OnTheFlyDetectPro(void);
void OnTheFlyDetectPro1(void);
void OpenCloseAngleSwitch(void);
void PmsmFluxObIni(void);
void OnTheFlyDetectInit(void);


/* ---------------------------观测器参数常量定义 ----------------------------- */ 
#define NOT_APPLICABLE                 (0)               /* 未采用 */

#define MOTOR_PARAMETER_TAB                                       \
{                                                                 \
    NOT_APPLICABLE,           /* 电机类型 */                      \
    NOT_APPLICABLE,           /* 额定功率 */                      \
    (s32)(U_MAX_FREQ * 100),  /* 额定频率 单位：0.01Hz*/          \
    U_MOTOR_PP,               /* 电机磁极对数 */                  \
    (s32)(U_RATED_CURRENT*10),/* 额定电流 单位：0.1A */           \
    (s32)(U_MOTOR_RS*10000),  /* 电机内阻 单位：0.0001欧姆 */     \
    (s32)U_MOTOR_LD,          /* 电机D轴电感 单位：uH */          \
    (s32)U_MOTOR_LQ,          /* 电机Q轴电感 单位：uH */          \
    U_MOTOR_FLUX_CONST*100000,/* 电机磁链常数 单位0.00001WB */    \
    NOT_APPLICABLE,           /* 电机转子内阻 单位：0.0001欧姆 */ \
    NOT_APPLICABLE,           /* 电机转子电感 单位：uH */         \
    NOT_APPLICABLE,           /* 电机互感 单位：uH */             \
    NOT_APPLICABLE,           /* 电机空载电流 单位：0.1A */       \
    NOT_APPLICABLE,           /* 电机额定转矩 单位：0.001Nm */    \
}

#define BOARD_PARAMETER_TAB                                       \
{  /* 功率板硬件电路特性参数表 */                                 \
    (s16)(U_RATED_VOLTAGE*10),/* 额定直流母线电压 */              \
    NOT_APPLICABLE,           /* 最大直流母线电压 */              \
    NOT_APPLICABLE,           /* 最大相电流 单位：0.1A */         \
    DEADTIME_NS,              /* 死区时间 单位：ns */             \
    (s32)(U_MAX_FREQ * 100),  /* 最大工作频率 单位：0.01Hz */     \
    PWM_FREQ,                 /* PWM载频 */                       \
                                                                  \
    ADC_SUPPLY_VOLTAGE,       /* ADC参考电压 单位：伏特 */        \
    RSHUNT,                   /* 分压电阻 单位：欧姆*/            \
    AMPLIFICATION_GAIN,       /* 放大器放大倍数 单位：n倍 */      \
    VOLTAGE_SHUNT_RATIO,      /* 母线电压采样分压比 */            \
    BEMF_SHUNT_RATIO,         /* 反电势电压采样分压比 */          \
}


#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

