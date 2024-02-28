/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： MC_type.c
 * 文件标识：
 * 内容摘要： 电机控制相关结构体声明
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
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/
#include "basic.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/*! Macro converting a signed fractional [-1,1) number into a fixed point 16-bit
 * number in the format Q1.15. */
 
#define S16FRACT_MAX                   (0.999969482421875)
#define S16FRACT_MIN                   (-1.0)

#define S32FRACT_MAX                   (0.9999999995343387126922607421875)
#define S32FRACT_MIN                   (-1.0)

#define FRAC16(x)                      ((s16) ((x) < (S16FRACT_MAX) ? ((x) >= S16FRACT_MIN ? (x)*0x8000 : 0x8000) : 0x7fff))

/*! Macro converting a signed fractional [-1,1) number into a fixed point 32-bit
 * number in the format Q1.31. */
#define FRAC32(x)                      ((s32) ((x) < (S32FRACT_MAX) ? ((x) >= S32FRACT_MIN ? (x)*0x80000000 : 0x80000000) : 0x7fffffff))

#define ABS(X)                         ( ( (X) >= 0 ) ? (X) : -(X) )

#define sat(x,ll,ul)                   ( (x) > (ul) ) ? (ul) : ( ( (x) < (ll) ) ? (ll) : (x) )


#define   _IQ15mpy(A,B)                (((s32)A*B)>>15)  /* 两个Q15格式相乘 */
#define   _IQ12mpy(A,B)                (((s32)A*B)>>12)  /* 两个Q12格式相乘 */

#define CURRENT_SAMPLE_1SHUNT          1                  /* 电流采样方式为单电阻采样拓扑 */
#define CURRENT_SAMPLE_2SHUNT          2                  /* 电流采样方式为双电阻采样拓扑 */
#define CURRENT_SAMPLE_3SHUNT          3                  /* 电流采样方式为三电阻采样拓扑 */
#define CURRENT_SAMPLE_MOSFET          4                  /* 电流采样方式为三个下桥臂MOS内阻采样拓扑 */

#define ROTOR_HALL_SENSOR              1                  /* 电机转子传感器类型为HALL */
#define ROTOR_SENSORLESS               2                  /* 电机转子传感器类型为HALL */
#define ROTOR_HALL2SENSORLESS          3                  /* 电机转子传感器类型为HALL + 观测器Sensorless */

typedef struct
{
    s16 nAxisQ;
    s16 nAxisD;
} stru_CurrVoctorDQ;

typedef struct
{
    s16 nAlph;
    s16 nBeta;
} stru_CurrAlphBeta;

typedef struct
{
    s16 nPhaseU;
    s16 nPhaseV;
    s16 nPhaseW;
} stru_CurrPhaseUVW;


typedef struct
{
    s16 nAxisQ;
    s16 nAxisD;
} stru_VoltVoctorDQ;

typedef struct
{
    s16 nAlph;
    s16 nBeta;
} stru_VoltAlphBeta;

typedef struct
{
    s16 nPhaseU;
    s16 nPhaseV;
    s16 nPhaseW;
} stru_VoltPhaseUVW;

typedef struct
{
    s16 hCos;
    s16 hSin;
} stru_TrigComponents;


typedef struct
{
    s32 yk_1;
    u16 coef;
} stru_RC_Def;

typedef struct
{
    s16 nData0;
    s16 nData1;
} stru_Array_Def;


typedef enum
{
    IDLE       = 0,   /* 空闲状态 */
    INIT       = 1,   /* 初始化状态 */
    CHARGE     = 2,   /* 充电状态 */
    ALIGN      = 3,   /* 预定位 */
    OPEN_RUN   = 4,   /* 开环运行状态 */
    DIR_CHECK  = 5,   /* 顺逆风检测 */
    POS_SEEK   = 6,   /* 初始位置检测 */
    RUN        = 7,   /* 正常运行状态 */
    STOP       = 8,   /* 停止状态 */
    BRAKE      = 9,   /* 刹车状态 */
    WAIT       = 10,  /* 等待状态 */
    FAULT      = 11,  /* 故障状态 */
    SEEK_POS   = 12,  /* 位置检测状态 */
    HALL_RUN   = 13,  /* hall运行状态 */
    HALL_START = 15,  /* HALL起动状态 */
    HALL_LEARN = 16,  /* HALL学习状态 */ 
    BEMF_CHECK = 17,  /* 反电势检测状态*/    
} enum_SystStatus;

typedef enum
{
    NO_FAULT, OVER_VOLT, UNDER_VOLT
} BusV_t;



typedef struct
{
    u16 nTimeBaseFlg;        /* Timer 中断标记，取值 0或1 */
    u8  bPWM_UpdateFlg;      /* PWM周期更新标志，一次间隔为一个PWM周期 */
	u8  bTimeCnt05ms;         /* 1mS计数器 */
    u8  bTimeCnt1ms;         /* 1mS计数器 */
    u16 nTimeCnt10ms;        /* 10mS计数器 */
		u16 nTimeCnt100ms;       /* 100mS计数器 */
    u16 nTimeCnt500ms;       /* 500mS计数器 */
    const char* sVersion;    /* 程序版本号 */
} stru_TaskSchedulerDef;


typedef struct {
    u16 cur;
    s16 curShftNum;
    u16 vol;
    s16 volShftNum;
    u16 freq;
    s16 freqShftNum;
    u16 speed;
    s16 speedShftNum;
    u16 power;
    s16 powerShftNum;
    u16 torque;
    s16 torqueShftNum;
    u16 angle;
    s16 angleShftNum;
    u16 vDc;
    s16 vDcShftNum;
} stru_TransCoef;

typedef struct
{
    s32 IPhAFilt;
    s32 IPhBFilt;
    s32 IPhCFilt;
    s32 IBusFilt;
    s32 UBusFilt;
} stru_OffsetDef; /* 偏置读取结构体 */

typedef struct
{
    s16 nCurrentAccStep;   /* 电流增加步长 */
    s16 nCurrentDecStep;   /* 电流减小步长 */   
    s16 nStartCurSet;      /* 启动电流设定 */
    s16 nStartCurRef;      /* 启动电流参考 */   
    s32 wStartCurRamp;     /* 电流爬坡值 */
  
    s32 wOpen2CloseFreq;   /* 开环切闭环频率 */
    s16 nFreqAccStep;      /* 频率加速步长 */
    s16 nFreqDecStep;      /* 频率减速步长 */
    s32 wRampFreqRef;      /* 频率爬坡值 */
} stru_OpenForceRunDef; /* 角度开环强拖结构体 */




/* Exported constants --------------------------------------------------------*/
#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)


#define S16P80     ((s16)26213)
#define S16P65     ((s16)25000)
#define S16P50     ((s16)13000)
#define S16P95     ((s16)30000)





/**
* Not initialized pointer
*/
#define MC_NULL    (uint16_t)(0x0000u)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MC_TYPE_H */
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

