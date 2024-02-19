/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： PowerCalculation.c
 * 文件标识：
 * 内容摘要： 功率环处理
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2020年10月9日
 *
 *******************************************************************************/
#include "MC_Parameter.h"
#include "PowerCalculation.h"
#include "FOC_Drive.h"
#include "Global_Variable.h"
#include "pmsmFluxObserve.h"

stru_PowerBasic_t struPower;
stru_Ramp_t PowerRamp;

/*****************************************************************************
 * 函数名   : void PowerLimitCalc(MechanicalQuantity *pstruMotorSpeed, s32 PowerValue)
 * 说明     : 限功率计算
 * 设计思路 ：1.如果实际功率超过限制值，则减低速度环的速度参考值，一般是在速度环的时候限功率处理
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 * 修改时间 : 2020.10.09
 * 修改内容 ：调整变量结构
 *****************************************************************************/
void PowerLimitCalc(MechanicalQuantity *pstruMotorSpeed, s32 PowerValue)
{
    if(++pstruMotorSpeed->nPowerLimitCntr >= POWER_LIMIT_TIME)                      //限功率计算周期
    {
        pstruMotorSpeed->nPowerLimitCntr  = 0;
        if(PowerValue > pstruMotorSpeed->wPowerLimitValue)                           //实际功率大于限制功率则减小速度给定值
        {
            if(pstruMotorSpeed->wPowerLimitSpeedSet < pstruMotorSpeed->wPowerLimitSpeedRef)    //功率超出限制后，增加限功率输出
            {
                pstruMotorSpeed->wPowerLimitSpeedSet += pstruMotorSpeed->wSpeedRampACCStep;
            }
            else
            {
                pstruMotorSpeed->wPowerLimitSpeedSet = pstruMotorSpeed->wPowerLimitSpeedRef;
            }
        }
        else                                                                  //功率恢复正常后，减小限功率输出
        {
            if(pstruMotorSpeed->wPowerLimitSpeedSet > 0)
            {
                pstruMotorSpeed->wPowerLimitSpeedSet -= pstruMotorSpeed->wSpeedRampDECStep;
            }
            else
            {
                pstruMotorSpeed->wPowerLimitSpeedSet = 0;
            }
        }
    }
}

/*****************************************************************************
 * 函数名   :void SpeedLimitCalc(s32 wSpeedFbk, stru_PowerBasic_t *pstruPower)
 * 说明     : 限转速计算
 * 设计思路 ：1.如果实际转速超过限制值，则减低功率环的功率参考值             \
 * 参数     ：s32 wSpeedFbk, stru_PowerBasic_t *pstruPower
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 * 修改时间 ：2020.10.09
 *****************************************************************************/
void SpeedLimitCalc(s32 wSpeedFbk, stru_PowerBasic_t *pstruPower)
{
    if(++pstruPower->nSpeedLimitCntr >= SPEED_LIMIT_TIME)                     //限转速计算周期
    {
        pstruPower->nSpeedLimitCntr  = 0;
        if(wSpeedFbk >= pstruPower->wSpeedLimitValue)                        //实际转速大于限制转速则减小功率给定值
        {
            if(pstruPower->wPowerSet < pstruPower->wSpeedLimitPowerRef)      //转速超出限制后，增加限速输出
            {
                pstruPower->wPowerSet += pstruPower->struPowerRamp.nACCStep;
            }
            else
            {
                pstruPower->wPowerSet = pstruPower->wSpeedLimitPowerRef;
            }
        }
        else                                                                  //转速恢复正常后，减小限转速输出
        {
            if(pstruPower->wPowerSet > 0)
            {
                pstruPower->wPowerSet -= pstruPower->struPowerRamp.nDECStep;
            }
            else
            {
                pstruPower->wPowerSet = 0;
            }
        }
    }
}



/*****************************************************************************
 * 函数名   : void PowerCalc(stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ , stru_PowerBasic_t *pstruPower)
 * 说明     : 功率计算
* 设计思路  : 1.P = ABS(Id*Ud) + ABS(Iq*Uq)/Ubus ,计算出来的功率值和实际功率进行曲线拟合，求出斜率和差值 \
 *          : 2.如果想得到更为准确的功率，需要对母线电流进行RC滤波后采样;P = UBus*IBus.
 * 参数     ：stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ , stru_PowerBasic_t *pstruPower
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 * 修改时间 ：2020.10.09
 * 修改内容 ：调整函数传参
 *****************************************************************************/
void PowerCalc(stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ,stru_PowerBasic_t *pstruPower)
{
    pstruPower->wTemp = (s32)(ABS(pstruCurrDQ->nAxisD * pstruVoltDQ->nAxisD)) + (s32)(pstruCurrDQ->nAxisQ * pstruVoltDQ->nAxisQ);  //功率计算

    pstruPower->struPowerLowPass.wInput = (pstruPower->wTemp >> 12);
    pstruPower->wPowerValue = LowPassControl(&pstruPower->struPowerLowPass);          //低通滤波
//    pstruPower->wPowerValue >>= POWER_SHIFT;
}

/*****************************************************************************
 * 函数名   : void LowPassControl(stru_LowPass_t *LowPass)
 * 说明     : 低通滤波
 * 设计思路 : 1. 低通滤波    截止频率 f = 1/((2^K1 -1)*Ts*2PI ,其中Ts为低通计算周期
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
//s32 LowPassControl(stru_LowPass_t *pstruLowPass)
//{
//    pstruLowPass->lTemp += pstruLowPass->wInput - pstruLowPass->wOutput;
//    pstruLowPass->wOutput = pstruLowPass->lTemp >> pstruLowPass->nK1;
//    return (pstruLowPass->wOutput);
//}

s32 LowPassControl(stru_LowPass_t *pstruLowPass)
{
    pstruLowPass->lTemp += (pstruLowPass->wInput * 65536 - pstruLowPass->lTemp) >> pstruLowPass->nK1;
    pstruLowPass->wOutput = pstruLowPass->lTemp >> 16;
    return (pstruLowPass->wOutput);
}

/*****************************************************************************
 * 函数名   : s32 RampControl(stru_Ramp_t *pstruRamp)
 * 说明     : 爬坡函数
 * 设计思路 : 1.爬坡计算
 * 参数     ：
 * 返回值   ：pstruRamp->wRef
 * 修改时间 ：2020.08.17
 *****************************************************************************/
s32 RampControl(stru_Ramp_t *pstruRamp)
{
    if(pstruRamp->wRef < pstruRamp->wSet)
    {
        if((pstruRamp->wRef + pstruRamp->nACCStep) <= pstruRamp->wSet)  //给定值小于设定值则增加
        {
            pstruRamp->wRef += pstruRamp->nACCStep;
        }
        else
        {
            pstruRamp->wRef = pstruRamp->wSet;
        }
    }
    else if(pstruRamp->wRef > pstruRamp->wSet)                         //给定值大于设定值则减小
    {
        if((pstruRamp->wRef - pstruRamp->nDECStep) >= pstruRamp->wSet)
        {
            pstruRamp->wRef -= pstruRamp->nDECStep;
        }
        else
        {
            pstruRamp->wRef = pstruRamp->wSet;
        }
    }

    return(pstruRamp->wRef);
}

/*******************************************************************************
函数名称：    s16 PowerPIRegulator(stru_PIRegulator *Regulator)
功能描述：    功率环PI  增量式PI
输入参数：    stru_PIRegulator *Regulator 结构体指针
输出参数：    PI调节结果
返 回 值：    无
其它说明：
修改日期      版本号          修改人            修改内容
-----------------------------------------------------------------------------
2020/8/5      V1.0           WenCai Zhao          创建
*******************************************************************************/
s16 PowerPIRegulator(stru_PIRegulator *Reg)
{
    long    ACC;
    int AX;

    ACC = (long)(Reg->wInError - Reg->wLastError) * Reg->KP;   /* 比例项计算 */
    ACC = (ACC << 0) + (long)(Reg->wInError) * Reg->KI;        /* 积分项计算 */
    Reg->wIntegral = ACC + Reg->wIntegral;

    if(Reg->wIntegral > Reg->wUpperLimitOutput)                /* 输出最大值限幅 */
    {
        Reg->wIntegral = Reg->wUpperLimitOutput;
    }
    else if(Reg->wIntegral < Reg->wLowerLimitOutput)         /* 输出最小值限幅 */
    {
        Reg->wIntegral = Reg->wLowerLimitOutput;
    }

    AX = Reg->wIntegral >> 16;

    Reg->wLastError = Reg->wInError;                           /* 记录上次误差值 */

    return(AX);
}

/*******************************************************************************
函数名称：    PowerLoopReg(stru_PowerBasic_t *this)
功能描述：    功率环计算
输入参数：    stru_PowerBasic_t *this  结构体指针
输出参数：    无
返 回 值：    无
其它说明：
修改日期      版本号          修改人            修改内容
-----------------------------------------------------------------------------
2020/8/5      V1.0           Howlet Li          创建
*******************************************************************************/
void PowerLoopReg(stru_PowerBasic_t *this)
{
    this->struPowerPI.wInError = this->wPowerRef - this->wPowerValue;

    /* 误差限幅，可根据实际负载调整，影响PI响应的快慢 */
    this->struPowerPI.wInError = sat(this->struPowerPI.wInError, -500, 500);

    struFOC_CurrLoop.nQCurrentSet = PowerPIRegulator(&this->struPowerPI);
}

/*****************************************************************************
 * 函数名   : void PowerLoopInit(void)
 * 说明     : 功率计算变量初始化
 * 设计思路 : 1.变量初始化
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.10.09
 *****************************************************************************/
void PowerLoopInit(void)
{
    /*功率环PI参数初始化*/
    struPower.struPowerPI.KP = POWER_KP;                  //功率环 Kp
    struPower.struPowerPI.KI = POWER_KI;                  //功率环 Ki
    struPower.struPowerPI.wInError = 0;
    struPower.struPowerPI.wIntegral = 0;
    struPower.struPowerPI.wLastError = 0;
    struPower.struPowerPI.wUpperLimitOutput = (App2CoreCurTrans(User2AppCurTrans(POWER_IQMAX)) << 16); //功率环输出最大值
    struPower.struPowerPI.wLowerLimitOutput = (App2CoreCurTrans(User2AppCurTrans(POWER_IQMIN)) << 16); //功率环输出最小值

    /*功率环计算参数初始化*/
    struPower.nSpeedLimitCntr = 0;
    struPower.wPowerRef = 0;
    struPower.wPowerSet = 0;
    struPower.wSpeedLimitPowerRef = POWER_CALC(SPEED_LIMIT_POWER_VALUE);
    struPower.wSpeedLimitPowerSet = 0;
    struPower.wSpeedLimitValue = App2CoreFreqTrans(User2AppFreqTrans(SPEED_LIMIT_VALUE));

    /*功率计算参数初始化*/
    struPower.wPowerValue = 0;
    struPower.wTemp = 0;

    struPower.struPowerLowPass.lTemp = 0;
    struPower.struPowerLowPass.nK1 = POWER_LOWPASS_SCALE;
    struPower.struPowerLowPass.wInput = 0;
    struPower.struPowerLowPass.wOutput = 0;

    /*功率爬坡参数初始化*/
    struPower.struPowerRamp.nACCStep = POWER_CALC(POWER_RUN_ACC);
    struPower.struPowerRamp.nDECStep = POWER_CALC(POWER_RUN_DEC);
    struPower.struPowerRamp.wRef = 0;
    struPower.struPowerRamp.wSet = 0;
    
    struAppCommData.nPowerFistFlag = 0;
}

