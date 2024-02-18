/*******************************************************************************
 * 版权所有 (C)2022, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： HeatControl.h
 * 文件标识：
 * 内容摘要： 发热丝控制
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： Robin Chen
 * 完成日期： 2022年12月13日
 *
 *******************************************************************************/
#ifndef __HEATCONTORL_H
#define __HEATCONTORL_H

#include "basic.h"
#include "lks32mc03x.h"
#include "lks32mc03x_gpio.h"
#include "basic.h"
#include "lks32mc03x.h"
#include "lks32mc03x_gpio.h"
//#include "LED.h"
#include "KeyScan.h"
#include "state_machine.h"
#include "Global_Variable.h"
#include "fault_detection.h"
typedef struct
{
	u8  ACFrequencyDetectFinishFlag;  //交流电50Hz或者60Hz检测完成标志   1：检测完成  0：未检测完成
  u16 ACFrequencyDetectCnt;         //交流电50Hz或者60Hz检测计数
	u8  ACFrequency;                  //交流电50Hz或者60Hz频率
  u16 AC50HzFrequencyDetectCntMax;  //交流电50Hz或者60Hz检测计数最大值
  u16 AC50HzFrequencyDetectCntMin;  //交流电50Hz或者60Hz检测计数最小值
  u16 AC60HzFrequencyDetectCntMax;  //交流电50Hz或者60Hz检测计数最大值
  u16 AC60HzFrequencyDetectCntMin;  //交流电50Hz或者60Hz检测计数最小值
	
	u8  ACDetectErrorFlag;            //交流电频率检测错误标志   1：错误  0：正常
	u8  CrossZeroErrorFlag;           //交流电过零检测错误标志   1：错误  0：正常
	
	u16 TurnOnDelayCnt;          //发热丝延迟工作计数
	u8 TurnOnOffFlag;            //发热丝可以允许工作标志    1：发热丝允许正常工作  0：发热丝关闭
	u8 CrossZeroState;           //当前交流电过零状态
	u8 CrossZeroStateLast;       //上次交流电过零状态
	s16 CrossZeroVoltage;				//当前ADC电压值
	s16 CrossZeroVoltageLast;		//上次ADC电压值
	u16 CrossZeroVoltageCnt1;			//ADC计数
	u16 CrossZeroVoltageCnt2;			//ADC计数
	u8 CrossDownAcState;			//掉电状态,
	u8 CrossDownAcStateCnt;			//掉电状态,
	u8 ThrowWaveFlag;            //丢波标志   1：可以正常丢波  0：不能丢波
	u8 CrossZeroCnt;             //过零计数
  u16 AC50HzThrowWaveDelayCnt; //交流电50Hz丢波延迟值
  u16 AC60HzThrowWaveDelayCnt; //交流电60Hz丢波延迟值
  u16 activecnt;
	u8 ThrowWaveMode;          //丢波模式
	u8 ThrowWaveCnt;           //丢波计数
	u8 ThrowWaveTabIndex;      //丢波表索引
	u32 ThrowWaveDelayCnt;     //丢波延时计数
	
	u8  CrossZeroPosedge;  //过零信号   1：上升沿  0：不确定
	u8  CrossZeroPosedgeFlag;  //过零信号标志   1：上升沿  0：不确定
	s16 TargetTemperature;     //发热丝目标温度值
	s16 HEATER_NTCTemperature; //采集发热丝实际温度值
	s16 IPM_OTPTemperature;    //采集IPM(MOS)实际温度值	

	u32 ThrowWaveBase;         //丢波基础值
	u32 ThrowWaveLimitMax;     //最大丢波限制值
	u32 ThrowWaveLimitMin;     //最小丢波限制值

} HEAT_CONTROL;

#define   ZERO_PORT    GPIO0        //ZERO端口
#define   ZERO_PIN     GPIO_Pin_6   //ZERO引脚

#define  ZERO_SCR()    GPIO_ReadInputDataBit(ZERO_PORT, ZERO_PIN)//(GET_AC_ZERO_CHEAK_CHANNEL)//GPIO_ReadInputDataBit(GPIO1, GPIO_Pin_4)      //读取过零信号电平

#define   HEATER_CONTROL_PORT    GPIO0        //温控发热丝端口
#define   HEATER_CONTROL_PIN     GPIO_Pin_4   //温控发热丝引脚

#define  HEATER_CONTROL_ON()     GPIO_ResetBits(HEATER_CONTROL_PORT, HEATER_CONTROL_PIN)   //开发热丝  置为低电平
#define  HEATER_CONTROL_OFF()    GPIO_SetBits(HEATER_CONTROL_PORT, HEATER_CONTROL_PIN)     //关发热丝  置为高电平

#define  TEM_SET1        (50)      //一档温度设定
#define  TEM_SET2        (60)     //二档温度设定
#define  TEM_SET3        (80)     //三档温度设定
#define  TEM_SET4        (100)     //三档温度设定

#define  TEMP_CONSTANT_FUNCTION   (FUNCTION_ON)   // FUNCTION_ON：发热丝恒温功能使能，FUNCTION_OFF：发热丝恒温功能不使能
#define  PullUp            1     //接上拉电阻到5V   
#define  PullUpParall      2     //接上拉电阻到5V+NTC并联电阻
#define  PullDown          3     //接下拉和上拉电阻到5V
#define  NTCSAMPLE     (PullUpParall)

#define  PullUpResistor    (5.1)    //上拉电阻，KΩ
#define  PullDownResistor  (10.0)    //下拉电阻，KΩ
#define  ParallResistor    (39.0)    //并联电阻，KΩ

#define AD_TEMP_MAX_VAL			32767

#define  ThrowFullWave   0     //发热温度控制丢全波
#define  ThrowHalfWave   1     //发热温度控制丢半波
#define  HEATER_DELAY_ON_TIME     (2000)          // 发热丝延迟开启时间 单位ms


#define CROSSZERO_CNT1			15			//L// ms
#define CROSSZERO_CNT2			50			//L// ms
#define AC_ZERO_ON				0
#define AC_ZERO_OFF				1

//50Hz/60Hz交流电的过零信号检测区间值（单位 us：1s/22K = 45.4545us）
#define  AC50HzMax          240    //50Hz交流电检测最大值  
#define  AC50HzMin          200    //50Hz交流电检测最小值 
#define  AC60HzMax          200    //60Hz交流电检测最大值 
#define  AC60HzMin          170    //60Hz交流电检测最小值 

#define  AC50Hz          50    //50Hz交流电  标准值200 = 10ms/45.4545
#define  AC60Hz          60    //60Hz交流电  标准值183 = 8.33msms/45.4545

#define  Delay50Hz       11    //50Hz可控硅延迟导通 10 11 12   
#define  Delay60Hz       9     //60Hz可控硅延迟导通 8  9  10


#define  NTC210C   0.506     //NTC温度对应的阻值
#define  NTC200C   0.604     //NTC温度对应的阻值
#define  NTC190C   0.738     //NTC温度对应的阻值
#define  NTC180C   0.911     //NTC温度对应的阻值
#define  NTC170C   1.131     //NTC温度对应的阻值
#define  NTC160C   1.450     //NTC温度对应的阻值
#define  NTC150C   1.736     //NTC温度对应的阻值
#define  NTC140C   2.229     //NTC温度对应的阻值
#define  NTC130C   2.884     //NTC温度对应的阻值
#define  NTC120C   3.767     //NTC温度对应的阻值
#define  NTC110C   4.977     //NTC温度对应的阻值
#define  NTC100C   6.67      //NTC温度对应的阻值
#define  NTC90C    9.09     //NTC温度对应的阻值
#define  NTC80C    12.53     //NTC温度对应的阻值
#define  NTC70C    17.50     //NTC温度对应的阻值
#define  NTC60C    24.83     //NTC温度对应的阻值
#define  NTC50C    35.89     //NTC温度对应的阻值
#define  NTC40C    52.98     //NTC温度对应的阻值
#define  NTC30C    80.29     //NTC温度对应的阻值
#define  NTC20C    125.86    //NTC温度对应的阻值
#define  NTC10C    198.2     //NTC温度对应的阻值
#define  NTC00C    324.1    //NTC温度对应的阻值
#define  NTCn10C   541.9     //NTC温度对应的阻值
#define  NTCn20C   930.5    //NTC温度对应的阻值
#define  NTCn30C   1665.5  //NTC温度对应的阻值
#define  NTCn40C   3176.7  //NTC温度对应的阻值

extern HEAT_CONTROL FRSControl;                      /* 发热丝控制参数 */
extern void HeaterTemperature_Control(void);
void HeaterControlProcess(void);
extern void HeaterControl(void);
extern s16 NTCTempAD(s16 TargetTemp);
extern s32 NTCResistor10(s16 TargetTemp);
#endif

