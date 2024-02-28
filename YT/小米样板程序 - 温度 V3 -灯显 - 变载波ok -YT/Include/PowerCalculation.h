/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： PowerCalculation.h
 * 文件标识：
 * 内容摘要： 功率处理
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2020年10月9日
 *
 *******************************************************************************/
#ifndef __POWER_H
#define __POWER_H

#include "basic.h"
#include "MC_Parameter.h"
#include "SpeedScan.h"
#include "mc_type.h"

#define POWER_LOWPASS_SCALE   (5)     //功率计算滤波系数
#define POWER_SHIFT           (7)     //功率计算移位系数，根据实际计算数值来修改此参数，如果数值小则适当减小此参数，否则适当加大

//***********************曲线拟合参数*******************//
#define POWER_SLOPE                     (27.502)  //曲线斜率
#define POWER_DIFF                      (18.187) //曲线差值
#define POWER_CALC(val)                 (POWER_SLOPE * val - POWER_DIFF)


typedef struct                        //低通
{
    s32 wInput;                       //输入量
    s32 wOutput;                      //输出量
    s16 nK1;                          //系数
    s64 lTemp;                        //中间值
} stru_LowPass_t;

typedef struct
{
    s32 wRef;                        //给定值
    s32 wSet;                        //设定值
    s16 nACCStep;                    //加速值
    s16 nDECStep;                    //减速值
} stru_Ramp_t;


typedef struct                      
{
    s32 wPowerValue;                  //功率实际值
	s32 wPowerValueLowPass;                  //功率滤波值
    s32 wTemp;                        //中间值
    
    stru_LowPass_t struPowerLowPass;  //低通滤波
    
    stru_Ramp_t  struPowerRamp;       //功率斜坡结构体
    
    stru_PIRegulator struPowerPI;     //功率环PI结构体
    
    s16 nSpeedLimitCntr;              //限转速周期计数
    s32 wSpeedLimitValue;             //转速限制值
    s32 wSpeedLimitPowerSet;          //限转速功率设定值
    s32 wSpeedLimitPowerRef;          //限转速功率给定值
    
    s32 wPowerRef;                    //功率给定值
    s32 wPowerSet;                    //功率设定值
	
	s16 BUSCURR;
	s32 BUSpower;
	
} stru_PowerBasic_t;

extern stru_PowerBasic_t struPower;

extern u16 DIV(u32 Dividend, s16 Divisor);
extern s32 LowPassControl(stru_LowPass_t *pstruLowPass);
extern s32 RampControl(stru_Ramp_t *pstruRamp);
extern void PowerCalc(stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ, stru_PowerBasic_t *pstruPower);

extern void PowerLimitCalc(MechanicalQuantity *pstruMotorSpeed, s32 PowerValue);
extern void SpeedLimitCalc(s32 wSpeedFbk, stru_PowerBasic_t *pstruPower);

extern void PowerLoopInit(void);

extern void PowerLoopReg(stru_PowerBasic_t *this);
#endif
