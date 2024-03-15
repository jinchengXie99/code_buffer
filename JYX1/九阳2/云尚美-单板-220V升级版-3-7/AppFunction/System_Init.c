/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Time_Process.c
 * 文件标识：
 * 内容摘要： 定时相关函数
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

/* Includes ------------------------------------------------------------------*/
#include "Global_Variable.h"
#include "hardware_config.h"
#include "function_config.h"
#include "fault_detection.h"
#include "MC_parameter.h"
#include "math.h"
#include "segger_rtt.h"

void CurrentOffsetCalibration(void);
void FluxObserveInit(void);
extern stru_FluxOBS_Def m_FluxOBSPara;               /* 观测器参数 */

/*******************************************************************************
 函数名称：    int sys_init(void)
 功能描述：    系统变量初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void sys_init(void)
{
    CurrentOffsetCalibration();               /* 读取电流采样通道偏置 */

    FluxObserveInit();                        /* 观测器参数初始化 */

    #if ((ROTOR_SENSOR_TYPE == ROTOR_HALL_SENSOR) || (ROTOR_SENSOR_TYPE == ROTOR_HALL2SENSORLESS))
    HALL_InitHallMeasure(&struHallProcess);   /* HALL传感器初始化 */
    #endif

    FaultInit();                              /* 故障检测初始化 */

    FaultRecoverInit();                       /* 故障恢复初始化 */

    struTaskScheduler.sVersion = &sVersion[0];/* 初始化版本号 */

    #if (RTT_FUNCTION == FUNCTION_ON)
    /* JScope RTT模式初始化 */
    SEGGER_RTT_ConfigUpBuffer(1, "JScope_i2i4", bRttBuf, 200, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    #endif

    #if (DEBUG_PWM_OUTPUT == TEST_ON)
    DebugPWM_OutputFunction(); /* 调试的时候输出25%的PWM波形 */
    #endif
}


/*******************************************************************************
 函数名称：    void CurrentOffsetCalibration(void)
 功能描述：    读电流Offset值
 输入参数：    stru_FOC_CtrProcDef *this  结构体指针
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void CurrentOffsetCalibration(void)
{
    u16 CalibCnt = 0;
    volatile u32 t_delay;
    stru_OffsetDef  struOffset;

    __disable_irq();


    ADC_SOFTWARE_TRIG_ONLY();

    ADC_STATE_RESET();

    ADC_NormalModeCFG();

    struFOC_CurrLoop.mVoltUVW_PWM.nPhaseU = 0;
    struFOC_CurrLoop.mVoltUVW_PWM.nPhaseV = 0;
    struFOC_CurrLoop.mVoltUVW_PWM.nPhaseW = 0;
    MCPWM0_RegUpdate();
    PWMOutputs(MCPWM0, ENABLE);
    for(t_delay = 0; t_delay < 0x7ffff; t_delay++);

    struOffset.IPhAFilt    = 0;
    struOffset.IPhBFilt    = 0;
    struOffset.UBusFilt    = 0;
    struOffset.IBusFilt    = 0;

    for(CalibCnt = 0; CalibCnt < (1 << ADC_GET_OFFSET_SAMPLES); CalibCnt++)
    {
        /* Clear the ADC0 JEOC pending flag */
        ADC_SWT = 0x00005AA5;

        while(!(ADC_IF & BIT0));

        ADC_IF |= BIT1 | BIT0;
        ADC_STATE_RESET();
        struOffset.IPhAFilt +=  (s16)((ADC_DAT0));
        struOffset.IPhBFilt +=  (s16)((ADC_DAT3));
        struOffset.IPhCFilt += (s16)((ADC_DAT5));
        struOffset.IBusFilt += (s16)(ADC_DAT3);
    }

    ADC_init();

    ADC_NormalModeCFG();

    __enable_irq();


    struFOC_CurrLoop.nPhaseAOffset = (s16)(struOffset.IPhAFilt >> ADC_GET_OFFSET_SAMPLES);
    struFOC_CurrLoop.nPhaseBOffset = (s16)(struOffset.IPhBFilt >> ADC_GET_OFFSET_SAMPLES);
    struFOC_CurrLoop.nPhaseCOffset = (s16)(struOffset.IPhCFilt >> ADC_GET_OFFSET_SAMPLES);

    if((ABS(struFOC_CurrLoop.nPhaseAOffset) > ADC_OFFSET_ERROR) ||
            (ABS(struFOC_CurrLoop.nPhaseBOffset) > ADC_OFFSET_ERROR) ||
            (ABS(struFOC_CurrLoop.nPhaseCOffset) > ADC_OFFSET_ERROR))
    {
        stru_Faults.B.OffsetError = 1;
    }




}

/*******************************************************************************
 函数名称：    void FluxObserveInit(void)
 功能描述：    观测器参数初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void FluxObserveInit(void)
{
    struFluxOB_Param.nSTO_KP = PLL_KP_GAIN;              /* 观测器PLL KP 参数 */
    struFluxOB_Param.nSTO_KI = PLL_KI_GAIN;              /* 观测器PLL KI 参数 */

    struFluxOB_Param.nSTO_KI_H = PLL_KI_GAIN_L;
    struFluxOB_Param.nSTO_KP_H = PLL_KP_GAIN_L;

    struFluxOB_Param.nCurrLoopIniKP = P_CURRENT_KP;      /* 电流环初始KP */
    struFluxOB_Param.nCurrLoopIniKI = P_CURRENT_KI;      /* 电流环初始KI */


    struFluxOB_Param.nCurrLoopIniKPL = P_CURRENT_KP_H;      /* 电流环初始KP */
    struFluxOB_Param.nCurrLoopIniKIL = P_CURRENT_KI_H;      /* 电流环初始KI */

    struFluxOB_Param.pStruMotorParame = &struMotorParam; /* 电机参数结构体 */
    struFluxOB_Param.pStruBoardParame = &struBoardParam; /* 功率板硬件电路特性参数 */


    m_FluxOBSPara.nMinOutFreq = OBS_MIN_OUT_SPEED;       /* 观测器最小输出频率 */
    m_FluxOBSPara.nMaxOutFreq = OBS_MAX_OUT_SPEED;       /* 观测器最大输出频率 */

    FluxObserveParaCalc();                               /* 观测器参数计算 */

    FOC_InitstruParam();                                 /* FOC 相关控制变量及结构体初始化 */

//    m_FluxOBSPara.nObsFactor = 65;    //3
//    m_FluxOBSPara.bDivisor = 3;     //4
//    m_FluxOBSPara.nObsCoef = FRAC16(0.25);                /* 观测器滤波系数 */

    m_FluxOBSPara.nObsFactor = START_OBSFACTOR;
    m_FluxOBSPara.bDivisor = START_DIVISOR;

    m_FluxOBSPara.nObsCoef = OBS_COEF;                /* 观测器滤波系数 */

}



/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
