/*******************************************************************************
 * 版权所有 (C)2015, LINKO Semiconductor
 *
 * 文件名称： mc_hall.c
 * 文件标识：
 * 内容摘要： Hall信号处理
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet Li
 * 完成日期： 2016年2月16日
 *
 * 修改记录1：
 *    修改日期：2016年2月26日
 *    版 本 号：V 1.0
 *    修 改 人：Howlet Li
 *    修改内容：创建
 *
 *
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_H
#define __HALL_H

/* Includes ------------------------------------------------------------------*/
#include "hardware_config.h"
#include "mc_type.h"


#define S16_360_PHASE_SHIFT    (u16)(65535)   /* 360度角归一化处理 1/360*65536, 角度0~360度变化，对应归一化数据0~65535 */
#define S16_330_PHASE_SHIFT    (u16)(60075)
#define S16_320_PHASE_SHIFT    (u16)(58253)
#define S16_318_PHASE_SHIFT    (u16)(57889) //58000--57889
#define S16_315_PHASE_SHIFT    (u16)(57344)
#define S16_310_PHASE_SHIFT    (u16)(56433)
#define S16_300_PHASE_SHIFT    (u16)(54613)   /* 300度角归一化处理 1/360*65536, 角度0~360度变化，对应归一化数据0~65535 */
#define S16_270_PHASE_SHIFT    (u16)(49152)
#define S16_240_PHASE_SHIFT    (u16)(43691)
#define S16_180_PHASE_SHIFT    (u16)(32768)
#define S16_120_PHASE_SHIFT    (u16)(21845)
#define S16_90_PHASE_SHIFT     (u16)(16384)
#define S16_75_PHASE_SHIFT     (u16)(13653)
#define S16_70_PHASE_SHIFT     (u16)(12743)
#define S16_63_PHASE_SHIFT     (u16)(11468)
#define S16_60_PHASE_SHIFT     (u16)(10923)
#define S16_45_PHASE_SHIFT     (u16)(8192)
#define S16_30_PHASE_SHIFT     (u16)(5461)
#define S16_20_PHASE_SHIFT     (u16)(3641)
#define S16_15_PHASE_SHIFT     (u16)(2731)
#define S16_10_PHASE_SHIFT     (u16)(1820)
#define S16_1_PHASE_SHIFT      (u16)(182)     /* 1度角归一化处理 1/360*65536, 角度0~360度变化，对应归一化数据0~65535 */

#define HALL_COM_TIMEOUT       BIT0           /* 超时换相错误 */
#define HALL_COM_FLG           BIT1           /* 换相标志，Hall发生变化时置1 */
#define HALL_COM_ERR           BIT2           /* Hall变化和预期值不一致 */
#define HALL_RUN_ERR           BIT6           /* Hall 运行出错 */
#define HALL_DIR_FLG           BIT7           /* 电机运行方向标志，0:正转 1:反转 */

#define REDUCE_TOQUE_PUSE      0x01           /* 电机速度较低时，正向降低换相引起的转矩波动标志 */
#define REDUCE_TOQUE_MINUS     0x02           /* 电机速度较低时，反向降低换相引起的转矩波动标志 */

#define HALL_SPEED_FIFO_SIZE   ((u8)6)        /* Hall信号处理Buffer大小，6次求平均 */


/* -----------------------Hall信号速度及锁相环相关宏定义 -------------------------------- */

#define ROTOR_SPEED_FACTOR_1US         (MCU_MCLK/6.0*65536)          /* 1个时钟节拍速度值标幺，360度对应65536 */
/* 最低速标幺值，对应一个Hall周期只计数1个时钟周期的情况 */
/* 角速度计数系数,为计算角速度的被除数: 一个PWM周期内角度累加值 = ROTOR_SPEED_FACTOR/Hall变化周期, 用以Hall PLL锁相和当前转子速度值计算 */
#define ROTOR_SPEED_FACTOR             (u32)(ROTOR_SPEED_FACTOR_1US/PWM_FREQ) 

#define ROTOR_SPEED_FACTOR_SAT         (u32)(ROTOR_SPEED_FACTOR - 100)        /* 角速度计数系数饱和处理 */
#define ROTOR_SPEED_LOW_FACTOR         (u32)(ROTOR_SPEED_FACTOR*2/3)         /* 低速标幺值 */
#define ROTOR_SPEED_FACTOR_REDUCE      (u32)(ROTOR_SPEED_FACTOR*3/2)         /* 极低速时，为降低扭矩波动，速度系数标幺值 */
#define ROTOR_SPEED_FACTOR_PER_UNIT    (u16)(MCU_MCLK/PWM_FREQ)              /* PWM周期对应主时钟周期，角速度系数关系 */
#define ROTOR_SPEED_FACTOR_PER_UNIT1   (u16)(ROTOR_SPEED_FACTOR_PER_UNIT/3) 
#define ROTOR_FACTOR_REDUCE_PER_UNIT   (u16)(ROTOR_SPEED_FACTOR_PER_UNIT)    /* 低速时, 角速度系数处理关系 */
#define ROTOR_FACTOR_ECC_UNIT          (u16)(ROTOR_SPEED_FACTOR_PER_UNIT*3/5)/* 冗余处理时, 角速度系数处理关系 */

#define RPM_TO_DPP                     (65536/PWM_FREQ/60.0)                 /* 1Rpm对就Dpp值，DPP:每个PWM周期下角度增量值，0~65536对应0~360度 */
#define DPP_TO_RPM                     (1.0/RPM_TO_DPP)                        /* 1单位角度增量对应Rpm值 */
#define MOTOR_POLES                    (u16)(35)                             /* 电机极对数 */
#define MAX_SPEED_RPM                  (u32)(2000)                           /* 电机最大Rpm值 */
#define MAX_SPEED_DPP                  (u16)(MAX_SPEED_RPM * MOTOR_POLES * RPM_TO_DPP)/* 系统限制的最大Dpp值 */ 

#define RPM_TO_PERIOD(rpm)             (u32)(MCU_MCLK/(rpm/60.0*6))          /* RPM转Hall周期值，单位主时钟计数值  */

#define MOROR_SPEED_RPM_150            (150)                                 /* 电机速度， 150RPM 1对极 */
#define MOROR_SPEED_RPM_160            (160)                                 /* 电机速度， 160RPM 1对极 */
#define MOROR_SPEED_RPM_130            (130)                                 /* 电机速度， 130RPM 1对极 */
#define MOROR_SPEED_RPM_1250           (1250)                                /* 电机速度， 750RPM 1对极 */
#define MOROR_SPEED_RPM_750            (750)                                 /* 电机速度， 750RPM 1对极 */
#define MOROR_SPEED_RPM_640            (640)                                 /* 电机速度， 640RPM 1对极 */
#define MOROR_SPEED_RPM_550            (550)                                 /* 电机速度， 550RPM 1对极 */

#define HALL_PLL_OPEN2CLOSE_THD        RPM_TO_PERIOD(MOROR_SPEED_RPM_160)    /* HALL PLL 开环切闭环速度门限值 */
#define HALL_PLL_CLOSE2OPEN_THD        RPM_TO_PERIOD(MOROR_SPEED_RPM_130)    /* HALL PLL 开环切闭环速度门限值 */

#define LOW_CLOSE_SPEED_THD            RPM_TO_PERIOD(MOROR_SPEED_RPM_1250)   /* Hall锁相环进入闭环门限 */
#define LOW_CLOSE_SPEED_THD1           RPM_TO_PERIOD(MOROR_SPEED_RPM_750)    /* Hall锁相环进入闭环门限1 */
#define LOW_CLOSE_SPEED_THD2           RPM_TO_PERIOD(MOROR_SPEED_RPM_550)    /* Hall锁相环进入闭环门限2 */

#define HALL_FAST_ADD_DPP              (u16)(MOROR_SPEED_RPM_150 * RPM_TO_DPP)/* 异常处理，角度快速累加Dpp值 */ 

typedef const struct
{
    s32 RotorSpeed_FACTOR;           /* 角速度计数系数,为计算角速度的被除数 */
    s32 RotorSpeed_FACTOR_33;        /* 1/3角速度计数系数 */
    s32 RotorSpeed_FACTOR_SAT;       /* 角速度计数系数饱和处理 */
    s16 RotorSpeed_FACTOR_PER_UNIT;  /* PWM周期对应主时钟周期，角速度系数关系 */
    s16 RotorSpeed_FACTOR_PER_UNIT1; /* PWM周期对应主时钟周期，角速度系数关系 */
    s16 RotorSpeed_REDUCE_PER_UNIT;  /* 低速时, 角速度系数处理关系 */
    s16 RotorSpeed_ECC_UNIT;         /* 冗余处理时, 角速度系数处理关系 */

    u32 LowCloseSpeed_THD;           /* Hall锁相环进入闭环门限 */
    u32 LowCloseSpeed_THD1;          /* Hall锁相环进入闭环门限1 */
    u32 LowCloseSpeed_THD2;          /* Hall锁相环进入闭环门限2 */

    u16 hPWM_PERIOD;                 /* PWM周期值 */

    u16 hPWM_TIME_1MS;               /* 以PWM周期值为时期，计数1ms的周期数 */
    u16 hPWM_TIME_2MS;               /* 以PWM周期值为时期，计数1ms的周期数 */
    u16 hPWM_TIME_4MS;               /* 以PWM周期值为时期，计数4ms的周期数 */
    u16 hPWM_TIME_10MS;              /* 以PWM周期值为时期，计数10ms的周期数 */
    u16 hPWM_TIME_20MS;              /* 以PWM周期值为时期，计数20ms的周期数 */

    u32 hHALL_LEARN_ADDR;            /* Hall学习Flash存储地址 */

} Stru_HallParaConstDef, *pStru_HallParama;

typedef struct
{
    u8 bLast_hall;           /* 上一个Hall状态 */
    u8 bBeforeLast_hall;     /* 上上一个Hall状态 */
    u8 bCheckCnt;            /* 滤波计数 */
    u8 bBrokenHallFlg;       /* 损坏Hall标志 BIT0 BIT1 BIT2对应3个Hall, 置位的代表故障Hall位置 */
    u8 bOldBrokenHallFlg;    /* 上一次损坏标志 */
    u8 bBrokenTypeFlg;       /* 损坏类型 */
    u8 bNeed_DelayComm;      /* 需要延时换相标志 */
    u8 bHallState_org;       /* Hall原始信号 */
    u8 bRepareDirect;        /* 修复方向，正向预测或反向预测 */
    u16 bDelay_timeCnt;      /* 延时时间计数器 */
    u8 bChangedHall;         /* 当前变化的Hall状态 */

    u16 nFocusCommCnt;       /* 强制换相计数器 */

    u8 bCommErrCnt;          /* 换相错误次数计数器 */
    u16 nSeekAngleLowThd;    /* 60度搜索下限和下限 */
    u16 nSeekAngleHighThd;   /* 60度搜索下限和上限 */

    u8 nSeekAngleFlg;        /* 正确角度搜索标志 */

    u16 nSeekAngleIncValue;  /* 角度搜索时的角度增量 */


} stru_HallRepairDef;

typedef struct
{
    u8  bFstHallSigFlg;                      /* 第一次Hall变化标志，用来Hall信号去抖 */
    u32 wOldSensorPeriod;                    /* 上次Hall信号滤波后周期值 */
    u32 wMotorSpeedAvgCnt;                   /* Hall信号滤波后周期值, 以HAll模块时钟周期计数 */
    u32 wSensorPeriod[HALL_SPEED_FIFO_SIZE]; /* Hall信号周期变化数组，用来求信号平均值 */
    u8  bSpeedFIFO_Index;                    /* Hall信号周期变化数组指针 */
    volatile u32 wHallPWMTimerCnt;           /* Hall变化周期以PWM周期冗余计数，计数器值 */
    u16 nElectrical_Angle;                   /* 当前转子位置角度值，0~65535对应0~360度 */
    u16 nOldElectrical_Angle;                /* 上一次转子位置角度值，0~65535对应0~360度 */  
    s16 nEndElectricalAngle;                 /* 最大角度增量限制值，在Hall处理错误时，用来快速找回正确角度 */
    u16 nTabElectrical_Angle;                /* 直接查表等到的当前转子电角度值 */
    u16 nRotorFreqDpp;                       /* 当前角速度值, 一个PWM周期下的角度增量值。0~65536对应0~360度 */
    u16 nRotorFreqDppFir;                    /* 当前角速度值(滤波后), 一个PWM周期下的角度增量值。0~65536对应0~360度 */
	  u16 nReduceToqueDpp;                     /* 极低速模式下当前角速度值, 一个PWM周期下的角度增量值。0~65536对应0~360度 */
    u16 nReduceToqueAngle;                   /* 低速模式下, 角度增量 */
    u32 wHallCapCnt;                         /* Hall模块捕捉累计值 */
    s16 nMaxReduceIncAngle;                  /* 在Reduce Toque模式下，角度最大增量 0~65536对应0~360度 */
  
    u8 bHallState;                           /* 当前滤波后的Hall状态 */
    u8 bOldHallState;                        /* 上一次滤波后的Hall状态 */ 
    u8 bHallStateOrg;                        /* 滤波前Hall 原始值 */

    u16 nCommErrType1Cnt;                    /* Hall滤波类型时间设定 */
    u16 nCommErrType2Cnt;                    /* Hall滤波类型时间设定 */
    u16 nHallCheckCnt;                       /* Hall滤波计数器*/
    u16 nHallCheckCnt2;                      /* Hall滤波计数器*/

    u8 bMotorDirtionCtrl;                    /* 期望控制的电机运行方向 */

    u16 nOverAngleCnt;                       /* 角度累加超过正常限制值 */
    u8 bHallType;                            /* Hall类型；60度Hall或120度Hall */ 
    u8 bHallRunFlg;                          /* Hall运行状态标志，故障、方向、超时等状态 HALL_COM_TIMEOUT，
                                                  HALL_COM_FLG，HALL_COM_ERR，HALL_DIR_FLG*/
    u8 bHallLearnFlg;                        /* Hall相序自学习标志 非零代表进入学习状态*/                                                  
    u16 nLearnHallTimeCnt;                   /* Hall学习，计数器值  */ 
    u16 nLearnHallStep;                      /* 当前Hall自学习运行步数 */
    u8 bHallLearnStateFlg;                   /* Hall学习状态 */
    u8 bMotorTypeLearnAngle;                 /* Hall自学习到的偏移角 */

    u8 bLearn_AngleOffset;                   /* 电流环模式学习Hall角度偏移标志 */
    u8 bLearn_AngleCnt;                      /* 电流环模式学习Hall角度偏移计数 */
    u8 bFirHallState;                        /* Hall自学习时Hall滤波状态 */
    s32 wHallErrTimeMark;                    /* Hall自学习时Hall滤波时间计数 */

    
    u8 bReduceToqueFlg;                      /* Reduce Toque模式工作标志，电机速度过低，工作在此模式 */ 
    u8 bFastAddElecAngleFlg;                 /* 在Hall处理错误时，用来快速累加到正确的角度 */
    u8 bCloseLoopCnt;                        /* Hall闭环运行计数，是否启动锁相环 */
    u8 bCloseLoopAngleFlg;                   /* Hall角度拟合，PLL闭环工作标志  */
    u16 nHallChangeCnt;                      /* Hall变化一次计数器值加1,错误清0 取值：0~65536 */  
    s16 nHallQEP_Cnt;                        /* Hall模拟编码器模式工作，计数器值, 取值：S16_MIN~S16_MAX */
  
    u16 nPhaseShift;                         /* 当前Hall角度计算角度偏移值 */
    u16 nPhaseShiftOffset;                   /* 当前Hall角度偏移值设置 */
    
    u32 wCloseLoopPeriodThd;                 /* HALL PLL闭环工作门限值 */
    u32 wOpenLoopPeriodThd;                  /* HALL PLL开环工作门限值 */
    u8  bOpenStartUpCntThd;                  /* 开环状态运行转闭环工作门限值 */
    
    u16 nMaxIncAngle;                        /* 最大角度增量 */
    u16 nCloseLoopMaxAngle;                  /* PLL模式下最大角度限制 */
    
    stru_RC_Def stru_HallDppRC;              /* Hall角速度低通滤波 */
    
    enum_SystStatus eMotorState;             /* 当前电机运行状态 */
    u8 bHallErrorCnt;                        /* Hall运行错误计数 */
    
    u8 bCommErrType1;                        /* Hall滤波检查干扰类型标志 */
    u8 bCommErrType2;                        /* Hall滤波检查干扰类型标志 */  




    u16 nHallCaliCnt;                        /* Hall偏移角校正步骤计数 */
    u8  bCalibrationHallFlg;                 /* Hall偏移角校正标志 */
    u16 nHallOffsetArry[6];                  /* Hall偏移角修正，锁相环输出，修正Hall安装引起的机械安装偏移角 */
    s16 nHallCaliArray[6];                   /* Hall偏移角校正值数组　*/
    
    u8  bHallPossableTab[8];                 /* Hall换相表，存储下一相Hall值 */
    u8  bHallPossableTab2[8];                /* Hall换相表，存储上一相Hall值 */
    u8  bHallCommTab[8];                     /* Hall换相表 */
    
    pStru_HallParama pStru_HallParama;       /* Hall处理相关Const变量 */
    
    stru_HallRepairDef stru_HallRepare;      /* Hall修复结构体 */
		
		s32 wSpeedfbkHall;                       /* 当前角速度值(滤波后),经过关系系数转化为闭环core速度 ADD*/

		//u8 gHall_ExpHallEdge;                   /* 当前hall上一时刻的hall状态的异或数值 ADD*/
		 s8 bHallSpeedDirFlag;                    /* 当前hall上一时刻的hall状态的异或数值 ADD*/
 
} stru_HallProcessDef;

/*******************************************************************************
 函数名称：    u8 ReadHallState(HALL_TypeDef *HALLx)
 功能描述：    读出当前Hall实时状态
 输入参数：    Hall通道号
 输出参数：    无
 返 回 值：    当前Hall实时状态
 其它说明：    静态函数，加速程序执行速度
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
static __inline u8 ReadHallState(HALL_TypeDef *HALLx)
{
    volatile u8 t_ReadValue;

    t_ReadValue = HALLx->INFO & 0x07;
	
	  //add 120°不需要异或;60°需要异或
//	  t_ReadValue ^= (BIT0 | BIT2);

    return(t_ReadValue);   
}

/*******************************************************************************
 函数名称：    u8 ReadHallCapState(HALL_TypeDef *HALLx)
 功能描述：    读出上次Hall捕捉到的状态值
 输入参数：    Hall通道号
 输出参数：    无
 返 回 值：    当前Hall实时状态
 其它说明：    静态函数，加速程序执行速度
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
static __inline u8 ReadHallCapState(HALL_TypeDef *HALLx)
{
    volatile u8 t_ReadValue;

    t_ReadValue = (HALLx->INFO >> 8) & 0x07;
	
	//add 120°不需要异或;60°需要异或
//	  t_ReadValue ^= (BIT0 | BIT2);

    return(t_ReadValue);
}         

u32 GetAvrgHallPeriod(stru_HallProcessDef *struHallProcess);
u8 check_hall_state(u8 t_hall, stru_HallProcessDef *struHallProcess);
void HALL_Init_Electrical_Angle(stru_HallProcessDef *struHallProcess);
void Angle_Init_Process(stru_HallProcessDef *struHallProcess);
void HALL_IRQProcess(HALL_TypeDef *HALLx, stru_HallProcessDef *struHallProcess);
void Verify_Hall_State(HALL_TypeDef *HALLx, stru_HallProcessDef *struHallProcess);
void GetHall_edgeAngle(stru_HallProcessDef *struHallProcess);
s16 lowPass_filter(stru_RC_Def *rc,s16 signal);
void closeLoopAnglePLLInit(u16 t_hElectrical_Angle, stru_HallProcessDef *struHallProcess);
void calc_first_ElectAngle(u16 angle,stru_HallProcessDef *struHallProcess);
void HALL_InitHallMeasure(stru_HallProcessDef *struHallProcess);
void Hall_FastAdd_ElecAngle(stru_HallProcessDef *struHallProcess);

void closeLoopAnglePLL(stru_HallProcessDef *struHallProcess);
void reduceToqueAnglePll(stru_HallProcessDef *struHallProcess);

void Hall_ElecAnglePWM_Process(stru_HallProcessDef *struHallProcess);

void learn_hall_proc(stru_HallProcessDef *this);

void Hall_learn_Process(void);

#define GET_HALL_LEARN_STATE()          (struHallProcess.bHallLearnFlg)     /* 得到HALL学习状态 */
#define HALL_LEARN_FINISH               3                                   /* HALL自学习结束 */

#endif /* __HALL_H */


/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
