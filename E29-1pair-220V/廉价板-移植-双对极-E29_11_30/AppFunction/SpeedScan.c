/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： SpeedScan.c
 * 文件标识：
 * 内容摘要： 电位器调速，PWM调速处理程序
 * 其它说明： 
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2019年12月27日
 *
 *******************************************************************************/
#include "SpeedScan.h"
#include "lks32mc03x.h"
#include "lks32mc03x_timer.h"
#include "state_machine.h"
#include "MC_type.h"
#include "fault_detection.h"
#include "Global_Variable.h"
#include "PowerCalculation.h"

stru_PWMTime_t  struPWMTime;

/*****************************************************************************
 * 函数名   : PWMScan(void)
 * 说明     : 检测输入PWM的占空比和周期，可以用作占空比调速和频率调速
 * 设计思路 ：1.用Timer0 检测PWM占空比，CH1用作捕捉高电平计数，CH0用作捕捉周期计数; \
 *          : 2.针对不接PWM的情况，外围电路要接上拉或者下拉
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PWMScan(void)
{
    if(UTIMER0_IF & BIT0) 
    {
        UTIMER0_IF = BIT0;
        
        struPWMTime.nFlagPeriod = 1;
        struPWMTime.wHighLevelTime = UTIMER0_CMP1; //捕捉高电平计数值
        struPWMTime.wPeriod = UTIMER0_CMP0;           //捕捉周期计数值

        struPWMTime.nHighCntr = 0;
        struPWMTime.nLowCntr = 0;
    }

    if(UTIMER0_IF & BIT2)             
    {
        UTIMER0_IF = BIT2;
        
        if(struPWMTime.nFlagPeriod == 0)     //如果检测不到CH1下降沿变化，则为PWM不接的情况
        {
            if(PWM_HIGH)                          //输入IO口电平为高，占空比为100%
            {

                struPWMTime.nHighCntr ++;
                struPWMTime.nLowCntr = 0;
            }
            else                             //输入IO口电平为低，占空比为0%
            {
                struPWMTime.nLowCntr ++;
                 struPWMTime.nHighCntr = 0;
            }
        } 
        else
        {                                          
            struPWMTime.nHighCntr = 0;
            struPWMTime.nLowCntr = 0;
        }

        struPWMTime.nFlagPeriod = 0;
    }
}

/*****************************************************************************
 * 函数名   : PWMDutyScan(void)
 * 说明     : PWM占空比计算
 * 设计思路 ：1.根据高电平和周期的计数来计算占空比，采用DSP计算，缩短计算时间
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PWMDutyScan(void)
{
    if((struPWMTime.wPeriod > 0)&&(struPWMTime.wHighLevelTime > 0))
    {
        struPWMTime.nDuty = (struPWMTime.wHighLevelTime * 1000 + ((s32)struPWMTime.wPeriod * 5))/struPWMTime.wPeriod;

        if((struPWMTime.nHighCntr > 5)||(struPWMTime.nLowCntr > 5))
        {
            if(struPWMTime.nHighCntr > struPWMTime.nLowCntr)
            {
                struPWMTime.nDuty  = DUTY_FULL;//100%
                struPWMTime.wPeriod = PWM_TIME_PERIOD; 
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                struPWMTime.wHighLevelTime = 0;
            }
            else
            {
                struPWMTime.nDuty  = DUTY_ZERO;//0%
                struPWMTime.wPeriod = PWM_TIME_PERIOD;
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                struPWMTime.wHighLevelTime = 0;
            }
        }
    }
    else
    {
        if((struPWMTime.nHighCntr > 5)||(struPWMTime.nLowCntr > 5))
        {
            if(struPWMTime.nHighCntr > struPWMTime.nLowCntr)
            {
                struPWMTime.nDuty  = DUTY_FULL;//100%
                struPWMTime.wPeriod = PWM_TIME_PERIOD; 
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                
                struPWMTime.wHighLevelTime = 0;
            }
            else
            {
                struPWMTime.nDuty  = DUTY_ZERO;//0%
                struPWMTime.wPeriod = PWM_TIME_PERIOD;
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                
                struPWMTime.wHighLevelTime = 0;
            }
        }
    }
}

//void PWMDutyScan(void)
//{
//    if(struPWMTime.nFlagPeriod == 1)
//    {
//        struPWMTime.nDuty = (struPWMTime.wHighLevelTime * 1000 + ((s32)struPWMTime.wPeriod * 5))/struPWMTime.wPeriod;
//    }
//    else
//    {
//        if((struPWMTime.nHighCntr > 5)||(struPWMTime.nLowCntr > 5))
//        {
//            if(struPWMTime.nHighCntr > struPWMTime.nLowCntr)
//            {
//                struPWMTime.nDuty  = DUTY_FULL;//100%
//                struPWMTime.wPeriod = PWM_TIME_PERIOD; 
//                
//                struPWMTime.nHighCntr = 0;
//                struPWMTime.nLowCntr = 0;
//                
//                struPWMTime.wHighLevelTime = 0;
//            }
//            else
//            {
//                struPWMTime.nDuty  = DUTY_ZERO;//0%
//                struPWMTime.wPeriod = PWM_TIME_PERIOD;
//                
//                struPWMTime.nHighCntr = 0;
//                struPWMTime.nLowCntr = 0;
//                
//                struPWMTime.wHighLevelTime = 0;
//            }
//        }
//    }
//}

/*****************************************************************************
 * 函数名   : PWMScanInit(void)
 * 说明     : PWM占空比检测初始化
 * 设计思路 ：1.变量初始化
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void PWMScanInit(void)
{
    struPWMTime.nDuty = 0;
    struPWMTime.nFlagPeriod = 0;
    struPWMTime.wHighLevelTime = 0;
    struPWMTime.wLowLevelTime = 0;
    struPWMTime.wPeriod = 0;
    struPWMTime.nHighCntr = 0;
    struPWMTime.nLowCntr = 0;
}

/*****************************************************************************
 * 函数名   : PWMSpeedScan(void)
 * 说明     : 
 * 设计思路 ：1.根据检测到的占空比计算对应转速
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
//void PWMSpeedScan(void)
//{
//    if(struPWMTime.nDuty < PWM_OFF)
//    {
//        struFOC_CtrProc.bMC_RunFlg = 0;   //关闭PWM
//        struAppCommData.wPWMPowerSet = PWM_POWER_OFF;
//    }
//    else if(struPWMTime.nDuty < PWM_MIN)
//    {
//        struAppCommData.wPWMPowerSet = PWM_POWER_MIN;
//    }
//    else if(struPWMTime.nDuty < PWM_MAX)
//    {
//        struAppCommData.wPWMPowerSet = (PWM_POWER_SCALE * (struPWMTime.nDuty - PWM_MIN)) + PWM_POWER_MIN;
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else
//    {
//        struAppCommData.wPWMPowerSet = PWM_POWER_MAX;
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//}

void PWMSpeedScan(void) //功率环
{
    if(struPWMTime.nDuty < PWM_OFF)
    {
        struFOC_CtrProc.bMC_RunFlg = 0;   //关闭PWM
        struAppCommData.wPWMPowerSet = PWM_POWER_OFF;
    }
    else if(struPWMTime.nDuty < PWM_MIN)
    {
        struAppCommData.wPWMPowerSet = PWM_POWER_MIN;

    }
    else if(struPWMTime.nDuty < PWM_MAX)
    {
        struAppCommData.wPWMPowerSet = (PWM_POWER_SCALE * (struPWMTime.nDuty - PWM_MIN)) + PWM_POWER_MIN;
        
        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
    }
    else
    {
        struAppCommData.wPWMPowerSet = PWM_POWER_MAX;
        
        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
    }
}




//void PWMSpeedScan(void)
//{
//    if(struPWMTime.nDuty < PWM_OFF)
//    {
//        struFOC_CtrProc.bMC_RunFlg = 0;   //关闭PWM
//    }
//    else if(struPWMTime.nDuty < PWM_MIN)
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_FIRST); 
//        
//    }
//    else if(struPWMTime.nDuty <= PWM_FIRST)    /* 1档 */
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_FIRST);          

//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else if(struPWMTime.nDuty < PWM_SECOND)    /* 2档 */
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_SECOND);        
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else if(struPWMTime.nDuty < PWM_THREE)    /* 3档 */
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_THREE);      
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_FOUR);
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//}

/*****************************************************************************
 * 函数名   : VspSpeedScan(void)
 * 说明     : 
 * 设计思路 ：1.根据检测到电压计算对应转速
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void VspSpeedScan(void)
{
    if(struAppCommData.nVspADC < VSP_MIN)
    {
        struAppCommData.wVSPSpeedSet = VSP_SPEED_MIN;
        
        struFOC_CtrProc.bMC_RunFlg = 0;
    }
    else if(struAppCommData.nVspADC  < VSP_MAX)
    {
        struAppCommData.wVSPSpeedSet = (VSP_SPEED_SCALE * (struAppCommData.nVspADC - VSP_MIN)) + VSP_SPEED_MIN;
        
        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
        
    }
    else
    {
        struAppCommData.wVSPSpeedSet = VSP_SPEED_MAX;

        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
    }
}

/*****************************************************************************
 * 函数名   : MotorDirScan(void)
 * 说明     : 电机运行方向转换
 * 设计思路 ：1.根据方向检测IO的电平来确定电机转向
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
//void MotorDirScan(void)
//{
//    static s16 s16DirectionCnt;

//    if(DIR_CW)                              //IO电平为高时为CW
//    {
//        s16DirectionCnt ++;
//        if(s16DirectionCnt >= DIR_FLITER)   //消抖
//        {
//            s16DirectionCnt = DIR_FLITER;
//            struFOC_CtrProc.bMotorDirtionCtrl = CW;
//        }
//    }
//    else                                     //IO电平为低时为CCW
//    {
//        s16DirectionCnt --;
//        if(s16DirectionCnt <= -DIR_FLITER)
//        {
//            s16DirectionCnt = -DIR_FLITER;
//            struFOC_CtrProc.bMotorDirtionCtrl = CCW;
//        }
//    }

//    if(struAppCommData.nMotorRunDirPre != struFOC_CtrProc.bMotorDirtionCtrl)   //检测到的转向和当前转向不一致，则切换到ilde状态处理。
//    {
//        struFOC_CtrProc.eSysState = IDLE;
//    }

//    struAppCommData.nMotorRunDirPre = struFOC_CtrProc.bMotorDirtionCtrl;
//}

/*****************************************************************************
 * 函数名   : void FGScan(u16 nTheta)
 * 说明     : 电机转速输出信号
 * 设计思路 ：1.机械一圈6个脉冲，6对极 == 一个电周期1个脉冲，需要根据实际需求处理。     \
 *          : 2.对于FG频率和电机电频率有整数倍关系的采用现有的方法，即通过角度来翻转IO; \
 *          : 3.对于特殊频率的FG，建议采用Timer处理。通过转速来计算Timer的周期值。
 * 参数     ：无
 * 返回值   ：无
 * 修改时间 ：2020.08.17
 *****************************************************************************/
void FGScan(u16 nTheta)
{
    if((nTheta > 0) && (nTheta <= 32766))          //0<Theta<180
    {
        FG_HIGH;
    }        
    else if((nTheta > 32766) && (nTheta <= 65535)) //190<Theta<360
    {
        FG_LOW;
    }
}

///*****************************************************************************
// * 函数名   : PowerScan(void)
// * 说明     : 给定功率计算
// * 设计思路 ：1.根据按键给定功率，1档60W，2档120W
// * 参数     ：无
// * 返回值   ：无
// * 修改时间 ：2021.04.21
// *****************************************************************************/
//void PowerScan(void)
//{
//    
//    if(struPWMTime.nVSPValue < VSP_MIN)
//    {
//        if(stru_Faults.R == 0)
//        {
//            struFOC_CtrProc.bMC_RunFlg = 0;
//        }
//    }
//    else if(struPWMTime.nVSPValue < VSP_MAX)
//    {
//        struPWMTime.wPowerValue = (POWER_SCALE * (struPWMTime.nVSPValue - VSP_MIN)) + POWER_MIN;
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else
//    {
//        struPWMTime.wPowerValue = POWER_MAX;
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }

//}


