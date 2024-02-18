/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： SpeedScan.h
 * 文件标识：
 * 内容摘要： 电位器调速、PWM调速
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2019年12月27日
 *
 *******************************************************************************/
#ifndef __SPEEDSCAN_H
#define __SPEEDSCAN_H

#include "basic.h"
#include "FOC_Drive.h"
#include "PowerCalculation.h"


#define PWM_TIME_PERIOD      4800 //输入频率对应Timer计数值。针对不接PWM的时候调用      4800--10K
#define DUTY_ZERO            0   //0占空比
#define DUTY_FULL            1000//100%占空比，数值扩大了10倍

#define PWM_HIGH             (GPIO0_PDI & BIT9) //PWM输入IO电平为高，一般来说恒高占空比为100%，恒低为0%

//#define DIR_FLITER           10//单位：ms 方向按键滤波时间，用于按键去抖，时间长短决定灵敏度
//#define DIR_CW               (GPIO0_PDI & BIT11)//方向输入IO口电平为高

//#define DIR_DELAY            (500)//单位:ms

#define FG_HIGH              (GPIO1_PDO |= BIT4)  //FG输出IO口电平为高    P0.6
#define FG_LOW               (GPIO1_PDO &= (~BIT4)) //FG输出IO口电平为低

/*********************PWM检测设定******************************/
#define PWM_OFF               (100)  //关闭占空比   9%
#define PWM_MIN               (120)  //最小占空比：10%
#define PWM_MAX               (950)  //最大占空比：90%

#define PWM_POWER_OFF         POWER_CALC(2.5)
#define PWM_POWER_MIN         POWER_CALC(3.0)
#define PWM_POWER_MAX         POWER_CALC(32.0)     //31

#define PWM_POWER_SCALE       (float)(PWM_POWER_MAX - PWM_POWER_MIN)/(PWM_MAX - PWM_MIN)  //PWM 调速斜率

#define PWM_SPEED_OFF         (s32)(10.0)      //单位：Hz
#define PWM_SPEED_MIN         (s32)(20.0)    //单位：Hz
#define PWM_SPEED_MAX         (s32)(100.0)   //单位：Hz
//#define PWM_SPEED_TOTAL       (s32)(3300.0)   //单位：Hz

#define PWM_SPEED_SCALE       (float)(PWM_SPEED_MAX - PWM_SPEED_MIN)/(PWM_MAX - PWM_MIN)  //PWM 调速斜率

/***********************VSP检测设定****************************/
#define VSP_OFF               (1820.0) //电位器电压关机值   计算公式： 0.2/3.6*32767 ，0.2为关机电压
#define VSP_MIN               (2730.0) //电位器电压最小值   计算公式： 0.3/3.6*32767 ，0.3为最小电压
#define VSP_MAX               (20024.0)//电位器电压最大值   计算公式： 2.2/3.6*32767 ，2.2为最大电压

#define VSP_SPEED_MIN         (30.0)    //单位：Hz
#define VSP_SPEED_MAX         (350.0)   //单位：Hz

#define VSP_SPEED_SCALE       (float)(VSP_SPEED_MAX - VSP_SPEED_MIN)/(VSP_MAX - VSP_MIN)  //PWM 调速斜率

typedef struct
{
    s32 wHighLevelTime;    //高电平计数值
    s32 wLowLevelTime;     //低电平计数值
    s32 wPeriod;           //周期计数值
    u16 nDuty;             //占空比
    u16 nFlagPeriod;       //周期加测标志位
    s32 wPreTime;          //上次计数值
    u16 nHighCntr;         /*IO口为高时的计数值*/
    u16 nLowCntr;         /*IO口为高时的计数值*/

    float fPwmSpeedScale; /* 转速计算比例值*/
    s32 wPwmSpeedMax;     /* 占空比给定最大值*/
} stru_PWMTime_t;


extern stru_PWMTime_t  struPWMTime;

extern void PWMScan(void);
extern void PWMScanInit(void);
extern void PWMDutyScan(void);
extern void VspSpeedScan(void);
extern void PWMSpeedScan(void);
extern void MotorDirScan(void);

void PWMOutputs(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state);

extern void FGScan(u16 nTheta);

#endif

