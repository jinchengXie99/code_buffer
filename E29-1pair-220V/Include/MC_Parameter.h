/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： parameter.h
 * 文件标识：
 * 内容摘要： parameter config
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： Andrew kong, Howlet Li
 * 完成日期： 2020年8月18日
 *
 *******************************************************************************/
 
/*------------------------------prevent recursive inclusion -------------------*/ 
#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "basic.h"
#include "MC_Type.h"

/* -----------------------------Hardware Parameter---------------------------- */
#define ADC_SUPPLY_VOLTAGE             (3.6)              /* 单位: V  ADC基准电压，3.6或者2.4,大部分应用选择3.6 */
#define AMPLIFICATION_GAIN             (18.18)            /* 运放放大倍数 */   //18.18
#define RSHUNT                         (0.200)             /* 单位: Ω  采样电阻阻值 */
#define VOLTAGE_SHUNT_RATIO            (6.8/(470.0*2+6.8)) /* 母线电压分压比 */
#define BEMF_SHUNT_RATIO               (1.0/(100.0*2 + 1.0)) /* 反电势电压分压比 */

#define CURRENT_SAMPLE_TYPE            CURRENT_SAMPLE_2SHUNT /* 电流采样方式选择 */

#define ROTOR_SENSOR_TYPE              ROTOR_SENSORLESS  /* 电机位置传感器类型选择 */

/* ------------------------------Rated Parameter------------------------------ */

#define U_RATED_VOLTAGE                (310.0)               /* 单位:V, 电机额定工作电压，设置为正常工作电压 */
#define U_RATED_CURRENT                (0.8)                /* 单位:A, 电机额定工作电流，相电流最大值 */
#define U_MAX_FREQ                     (3000.0)            /* 单位:Hz,  电机最高运行转速 */

/* ------------------------------Motor Parameter------------------------------ */
#define U_MOTOR_PP                     (1.0)              //电机极对数   徕芬电机 1对极 220V
#define U_MOTOR_RS                     (24.5)           //单位: Ω 电机相电阻
#define U_MOTOR_LD                     (1900.3)           //单位: uH 电机d轴电感
#define U_MOTOR_LQ                     (2300.4)           //单位: uH 电机q轴电感


//#define U_MOTOR_PP                     (1.0)              //电机极对数   拆机电机 1对极 220V
//#define U_MOTOR_RS                     (24.321)           //单位: Ω 电机相电阻
//#define U_MOTOR_LD                     (2008.6)           //单位: uH 电机d轴电感
//#define U_MOTOR_LQ                     (2557.8)           //单位: uH 电机q轴电感

//#define U_MOTOR_PP                     (1.0)              //电机极对数   今鸿星辉电机 1对极 220V
//#define U_MOTOR_RS                     (21.887)           //单位: Ω 电机相电阻
//#define U_MOTOR_LD                     (1951.985)           //单位: uH 电机d轴电感
//#define U_MOTOR_LQ                     (2569.775)           //单位: uH 电机q轴电感

//#define U_MOTOR_PP                     (1.0)              //电机极对数   海和电机 1对极 110V
//#define U_MOTOR_RS                     (6.055)           //单位: Ω 电机相电阻
//#define U_MOTOR_LD                     (453.29)           //单位: uH 电机d轴电感
//#define U_MOTOR_LQ                     (693.62)           //单位: uH 电机q轴电感

/* 电机磁链常数 计算公式：Vpp/2/sqrt(3)/(2*PI)/f，其中Vpp为电压峰峰值，f为电频率 
   此参数仅影响顺逆风启动的速度检测，角度估算不使用些参数 */
#define U_MOTOR_FLUX_CONST             (0.00142328)

/* ----------------------IPD 转子初始位置检测 脉冲注入时间设置---------------- */
#define SEEK_POSITION_STATUS           (FALSE)             /* 初始位置检测状态 TRUE为使能, FALSE为不使能 */
#define U_START_ANGLE_COMP             (0)                /* 单位:度 初始位置检测补偿角度 */

#define IPD_PLUS_TIME_SETTING          (1)              /* 脉冲注入时间宽度设置 单位us */
#define IPD_WAIT_TIME_SETTING          (50)              /* 脉冲空闲等待时间宽度设置 单位us */

#define IPD_PLUS_TIME_WIDTH            (u32)(IPD_PLUS_TIME_SETTING*(MCU_MCLK/1000000))   /* 脉冲注入时间宽度设置 单位clk */
#define IPD_PLUS_WAIT_TIME             (u32)(IPD_WAIT_TIME_SETTING*(MCU_MCLK/1000000))   /* 脉冲空闲等待时间宽度设置 单位clk */

/* -------------------------------HALL自学习设置------------------------------- */
#define HALL_LEARN_CURRENT_SETTINT     (1.0)              /* HALL自学习电流设定,单位: A */

/* ----------------------------direction check Parameter----------------------- */
#define CW                             (0)                /* 电机转向：顺时针 */ 
#define CCW                            (1)                /* 电机转向：逆时针*/

/* ------------------------------ADC校准相关参数设置---------------------------- */
#define CALIB_SAMPLES                  (512)              /* ADC偏置校准次数，不可修改 */
#define OFFSET_ERROR                   (3500)             /* ADC偏置误差阈值，不用修改 */

/* ----------------------------预驱自举电容预充电参数--------------------------- */
#define CHARGE_TIME                    (100)              /* 每相预充电时间，根据实际硬件参数修改 */

/* -------------------------------- 顺逆风检测参数------------------------------ */
#define SPEED_TRACK_DELAYTIME          (100)              /* 单位: ms 顺逆风检测时间  */
#define SPEED_TRACK_ON_FREQ_THH        (10)               /* 单位: Hz 顺风切闭环频率 */
#define EBRK_ON_FREQ_THH               (20)               /* 单位: Hz 逆风刹车频率 */

#define MOTOR_STOP_CUR_THD             (120)              /* 电机停止检测电流阈值 */
#define MOTOR_STOP_CUR_DIF_THD         (12)               /* 电机停止检测电流差阈值 */

#define STOP_TIME                      (250)               /* 单位：ms 电机停止检测滤波时间，根据实际负载修改 */
#define STOP_DELAY_TIME                (200)              /* 单位: ms 电机停止后延迟时间，根据实际负载修改。修改依据：电机在判定为停止后还在转动就加大延迟时间 */

#define CLOSE_VOLT                     (1000)              /**/
/*------------------------------------预定位参数---------------------------------*/
#define ALIGN_ANGLE                    (0)                /* 单位:度 预定位角度 */
#define U_START_CUR_SET_F              (0.0)              /* 单位: A 第一段定位电流 */
#define U_START_CUR_SET_S              (0.55)              /* 单位: A 第二段定位电流 */
#define DC_HOLDTIME_TIME_LENTH         (10)               /* 单位: ms 第一段定位时间 */
#define DC_HOLDTIME_TIME_LENTH_STAGE1  (10)               /* 单位: ms 第二段定位时间 */
#define DC_ALIGN_TOTAL_LENTH            \
       (DC_HOLDTIME_TIME_LENTH + DC_HOLDTIME_TIME_LENTH_STAGE1)/* 定位总时长 */

#define ALIGN_CURRENT_ACC              (0.01)              /* 单位: (1/8)A/ms  定位电流加速调整值  初始位置检测使能后给到最大值，不能超过30，否则数据会溢出。 */
#define ALIGN_CURRENT_DEC              (0.01)              /* 单位: (1/8)A/ms  定位电流减速调整值  初始位置检测使能后给到最大值，不能超过30，否则数据会溢出。 */

/*---------------------------------开环参数------------------------------------*/
#define OPEN_ANGLE_TAG_FREQ            (60.0)             /* 单位：Hz 开环拖动最终频率 */
#define FREQ_ACC                       (0.0)              /* 单位：(1/128)Hz/ms 开环拖动频率加速调整值 */
#define FREQ_DEC                       (20.0)              /* 单位：(1/128)Hz/ms 开环拖动频率减速调整值 */

#define OPEN_RUN_STATUS                (FALSE)            /* 开环状态 TRUE = 开环运行, FALSE = 闭环运行 */
  
#define MATCH_TIME                     (5)                /* 估算和给定电流匹配次数 */ 

/*---------------------- -----开环闭环切换过渡参数------------------------------*/
#define OPEN2CLOSE_RUN_COV_TIME        (2)               /* 开环闭环切换过渡时间：单位：mS */
#define OPEN2CLOSE_RUN_CURRENT_RAMP    (0.1)             /* 开环闭环切换过渡内，D,Q轴电流变化斜率。单位：A/ms */

/*-----------------------------------环路选择----------------------------------*/
#define CURRENT_LOOP                   (0)                /* 电流环 */
#define SPEED_LOOP                     (1)                /* 速度环 */
#define POWER_LOOP                     (2)                /* 功率环 */
#define CLOSE_LOOP                     (SPEED_LOOP)      /* 环路选择 */

/* ------------------------------启动参数----------------------------- */
#define IQ_START                       (0.1)               /* 单位 */

#define PLL_KP_GAIN                    (0)                 /* PLL_Kp 估算器Kp  start*/ //800
#define PLL_KI_GAIN                    (0)                  /* PLL_Ki 估算器Ki */  //300

#define PLL_KP_GAIN_L                  (1600)                  /* PLL_Kp 估算器Kp */ //800
#define PLL_KI_GAIN_L                  (100)                  /* PLL_Ki 估算器Ki */ //50

//#define PLL_KP_GAIN_H                  (1000)                 /* PLL_Kp 估算器Kp */ //400
//#define PLL_KI_GAIN_H                  (200)                  /* PLL_Ki 估算器Ki */ //10

#define PI_MATCH_FREQ                  (30.0)              /* 单位：Hz PI参数调整频率 */
#define CLOSE_FREQ                     (500.0)              /* 单位：Hz 切入功率环的频率 */

#define OBS_MIN_OUT_SPEED              (150)                  /* 观测器最小输出电频率，单位：0.1Hz*/  //50
#define OBS_MAX_OUT_SPEED              0//(3000)                /* 观测器最小输出电频率，单位：0.1Hz*/  //2000

#define START_DIVISOR                  (5.0)               /*启动参数--移位*/  //3
#define START_OBSFACTOR                (30.0)              /*启动参数--系数*/ //65

#define OBS_COEF                        FRAC16(0.05)        /* 观测器滤波系数 0~1  */  //0.25

/*----------------------------------电流环参数---------------------------------*/
#define IQ_SET                         (0.40)              /* 单位：A IqRef，Iq给定值 */
 
#define VQMAX                          (6000)             /* Q轴最大输出限制，Q15格式，取值范围0~6000 */
#define VQMIN                          (-VQMAX)            /* Q轴最小输出限制，Q15格式，取值范围0~-6000 */

#define VDMAX                          (VQMAX)             /* D轴最大输出限制，Q15格式，取值范围0~6000 */
#define VDMIN                          (-VQMAX)            /* D轴最小输出限制，Q15格式，取值范围0~6000 */

#define P_CURRENT_KP                   4500               /* 电流环Kp，实际运用的Kp会根据这个值和电机参数计算出最终的Kp */
#define P_CURRENT_KI                   700                /* 电流环Ki，实际运用的Kp会根据这个值和电机参数计算出最终的Ki */

#define P_CURRENT_KP_H                 3338              /* 高速电流环Kp， 1500= 1.5倍*/  //1200
#define P_CURRENT_KI_H                 140000               /* 高速 电流环Ki，1500= 1.5倍 */ //1200

#define AUTO_FW_LIM                    ((s16)0)        /* 自动弱磁D轴电流限制，Q12格式，最大值 4096 */

#define TORQUE_MODE_CURRENT_CHANGE_ACC (0.1)              /* 单位：A/ms 电流加速调整值 */
#define TORQUE_MODE_CURRENT_CHANGE_DEC (0.1)              /* 单位：A/ms 电流减速调整值 */

/*----------------------------------速度环参数-------------------------------*/
#define POWER_LIMIT_STATUS             (FALSE)            /* 限功率状态，TRUE = 使能，FALSE = 不使能 */
#define POWER_LIMIT_VALUE              (10.0)             /* 单位：W   限制功率的大小 */
#define POWER_LIMIT_TIME               (5)                /* 单位：速度环周期，  限功率计算周期 */
#define POWER_LIMIT_SPEED              (10)               /* 单位：Hz  限功率转速给定，根据实际应用来设置， */


#define POWER_CHECK                    (23)               /* 单位：功率校正值 */
#define POWER_CHECK1                   (34)               /* 单位：功率校正值 */
#define SPEED_SET                      (1834)               /* 单位：Hz  速度给定值  1450 1834 */
#define SPEED_LOOP_CNTR                (0)                /* 单位：ms  速度环路计算周期 */

#define STATE04_WAITE_TIME             (2)              /* Unit: ms 速度变量初始化时间 */

#define P_ASR_KP                      4000 //4000//2000//(3000)             /* 速度环Kp */
#define P_ASR_KI                      6000//6000//3000//(5000)              /* 速度环Ki */

#define IQMAX                          (0.6)                /* 单位:A, 速度环输出最大值 */
#define IQMIN                          (-0.4)               /* 单位:A, 速度环输出最小值 */

#define SPEED_RUN_ACC                  (3.0)            /* 单位 (1/128)Hz 速度加速调整值 */
#define SPEED_RUN_DEC                  (3.0)              /* 单位 (1/128)Hz 速度减速调整值 */

/* 刹车参数 */
#define BRAKE_IQ_EN                    (1)             /* IQ刹车 TRUE为使能, FALSE为不使能 */ 
#define BRAKE_CURRENT                  (-0.55)             /* 单位：A 刹车电流 */
#define BRAKE_SPEED                    (350)              /* 单位：Hz 刹车转速 */
#define BRAKE_P_CURRENT_KP              100                /* 刹车电流环Kp */
#define BRAKE_P_CURRENT_KI              44                /* 刹车电流环Ki */

/*------------------------------------功率环参数-------------------------------*/
#define SPPED_LIMIT_STATUS             (FALSE)            /*  限转速状态，TRUE = 使能，FALSE = 不使能 */
#define SPEED_LIMIT_VALUE              (200.0)            /* 单位：Hz  限制转速的大小 */
#define SPEED_LIMIT_TIME               (5)                /* 单位：ms  功率环周期， 限转速计算周期 */
#define SPEED_LIMIT_POWER_VALUE        (10)               /* 单位：W   限转速功率给定 */

#define POWER_SET                      (30.0)               /* 单位：W  功率给定值 */
#define POWER_LOOP_CNTR                (1)                /* 单位：ms  功率环路计算周期 */
                                                                  
#define POWER_KP                       (6000)             /* 功率环Kp */
#define POWER_KI                       (600)              /* 功率环Ki */

#define POWER_IQMAX                    (3.8)              /* 单位:A, 功率环输出最大值 */
#define POWER_IQMIN                    (-3.8)             /* 单位:A, 功率环输出最小值 */
                                                                   
#define POWER_RUN_ACC                  (2.0)              /* 单位 w 功率加速调整值 注意POWER_RUN_ACC和POWER_RUN_DEC不能太小，结合拟合参数设置，实际程序中的加减速值不能能小于0. */
#define POWER_RUN_DEC                  (2.0)              /* 单位 w 功率减速调整值 */

/*------------------------------------FaultDetection---------------------------*/
/* 过流检测参数 */
#define I_PH_OVERCURRENT_FAULT         (2.0)              /* 单位：A 软件过流检测设定值 */

/* 过欠压检测参数 */
#define U_DCB_OVERVOLTAGE_FAULT        (360.0)               /* 单位：V 过压检测设定值 */
#define U_DCB_OVERVOLTAGE_RECOVER      (350.0)               /* 单位：V 过压恢复设定值 */
#define U_DCB_UNDERVOLTAGE_FAULT       (254.0)               /* 单位：V 欠压检测设定值 */
#define U_DCB_UNDERVOLTAGE_RECOVER     (270.0)               /* 单位：V 欠压恢复设定值 */

/* 离水空转参数 */
#define I_PH_EMPTY_FAULT               (0.15)              /* 单位：A 空转检测电流设定值 */
#define SPEED_EMPTY_FAULT              (180.0)             /* 单位：Hz 空转检测转速设定值  */

/* 温度检测参数 */
#define TEMP_FAULT                     (140)              /* 过温检测设定值 */
#define TEMP_RECOVER                   (70)              /* 过温恢复设定值 */
#define TEMP_BREAK                     (120)             /* NTC开路设定值 */
                                                                  
/* 堵转检测参数 */
#define SPEED_STALL_MAX_FAULT          (2200.0)           /* 单位：Hz 堵转检测转速最大值 */
#define SPEED_STALL_MIN_FAULT          (50.0)             /* 单位：Hz 堵转检测转速最小值 */

#define I_PH_STALL_FAULT               (1.4)              /* 单位：A 堵转检测电流设定值 */

#define SPEED_STALL_FAULT              (30.0)             /* 单位：Hz 堵转检测转速设定值 */
#define IQ_STALL_FAULT                 (0.1)              /* 单位：A 堵转检测电流设定值 */

#define STALL_CHCEK_TIME               (150)               /* 单位：5ms 堵转检测时间*/   //100

#define  STALL_POWER1                  (78)               /* 单位：w 堵孔功率1*/   //100
#define  STALL_POWER2                  (117)               /* 单位：W 堵孔功率2*/   //100
#define  RESTART_CNT                    (2)                  // 允许重启次数

/* 二次启动检测参数 */
#define START_TIME_FAULT               (100)              /* 单位：5ms 开环之后1s内还不进入闭环就重启，1s这个时间根据实际应用调整 */
#define STARTUP_FAILED_TIME_5MS        (100)
/* 缺相检测参数 */
#define I_PHASE_LOSS_FAULT             (3000)             /* 数字量，根据实际测试中struCurrentAmplitude.nPhA/struCurrentAmplitude.nPhB/struCurrentAmplitude.nPhC */
                                                          /* 的计算值来设定，在缺相和正常运行的中间取值。 */
#define PHASE_CHCEK_TIME               (40)               /* 单位：5ms 缺相检测时间*/

/* 故障恢复时间 */
#define VOLT_FAULT_RECOVER_TIME        (300)             /* 单位：ms  过欠压恢复时间 */
#define CURRENT_FAULT_RECOVER_TIME     (300)             /* 单位：ms  过流恢复时间 */
#define STALL_FAULT_RECOVER_TIME       (300)             /* 单位：ms  堵转恢复时间 */
#define PHASELOSS_FAULT_RECOVER_TIME   (300)             /* 单位：ms  缺相恢复时间 */
#define TEMP_FAULT_RECOVER_TIME        (2000)             /* 单位：ms  过温恢复时间 */
#define START_FAULT_RECOVER_TIME       (300)             /* 单位：ms  二次启动恢复时间 */
#define EMPTY_FAULT_RECOVER_TIME       (300)             /* 单位：ms  离水空转恢复时间 */



#endif  /* __PARAMETER_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* -----------------------------------END OF FILE------------------------------- */

