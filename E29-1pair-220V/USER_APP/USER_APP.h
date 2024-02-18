/*******************************************************************************
 * 版权所有 (C)2020, Bright Power Semiconductor Co.ltd
 *
 * 文件名称： USER_APP.h
 * 文件标识：
 * 内容摘要： 
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： BPS IOT TEAM.
 * 完成日期： 2020年1月26日
 *
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __USE_APP__
#define __USE_APP__

 /****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "basic.h"
#include "lks32mc03x_gpio.h"
extern void USER_GPIO_Init(void);
extern void cheak_50hz(void);
extern void USER_Init(void);
extern void USER_APP_vTick1ms(void);
extern void USER_APP_vTick10ms(void);
extern void USER_APP_vTick05ms(void);
extern void GetVdcSpeed(void);
extern void vLoadForFlashing(void);
extern void PWMLED1(void);
extern void PWMLED2(void);
uint16_t GetHeartNTCTemp(void);
#define hzcheck_port              GPIO0
#define hzcheck_port_pin          GPIO_Pin_8
extern  u8 Flag_50HZ ;
extern u8 iTargetGear;
extern s16 t_VdcSpdCoef;
#define maxgear                              3
extern s32 powerfreqcmd[maxgear];

extern u32 Memorymsg;
extern u8 TempGear;
extern u8 TempGearAdd;
extern u8 TempGearOld;
struct appcmdkey 
{
	u8 cmd_exe;
	u8 cmd_exeorg;
	u8 sts;
	//u8 longshrotpress;
};//app cmd key..

typedef struct
{
	volatile uint8_t ucUserSysState;
	volatile uint16_t uiUserSysMsec;
	volatile uint16_t uiKey1ADValue;
	volatile uint16_t uiKey2ADValue;
	volatile uint8_t ucCleanFunCheckEnable;
	volatile uint8_t ucCleanFunCheckActived;
	volatile uint8_t ucUserFunState;
	volatile uint16_t uiHeartNTCTem;
	volatile uint8_t speedset;
	volatile uint8_t tempset;
	volatile uint8_t cleanset;
	volatile uint8_t cleanfinish;
	volatile uint8_t ucMotorDir;
	volatile uint16_t RunTime;
} stru_UserSys;
extern stru_UserSys UserSys;

#define USERFUN_STATE_WAIT		0x00
#define USERFUN_STATE_NORMAL	0x01
#define USERFUN_STATE_CLEAN		0x02
#define USERFUN_STATE_FAULT	    0x03

#define   LED1_PORT    GPIO1        //LED1端口
#define   LED1_PIN     GPIO_Pin_4   //LED1引脚

#define   LED2_PORT    GPIO0        //LED2端口
#define   LED2_PIN     GPIO_Pin_1   //LED2引脚

#define   LED3_PORT    GPIO0        //LED3端口
#define   LED3_PIN     GPIO_Pin_3   //LED3引脚

#define   LED1B_OFF()    GPIO_ResetBits(LED1_PORT, LED1_PIN) //红灯置低电平
#define   LED1B_ON()   GPIO_SetBits(LED1_PORT, LED1_PIN)   //红灯置高电平

#define   LED2Y_OFF()    GPIO_ResetBits(LED2_PORT, LED2_PIN) //橙灯置低电平
#define   LED2Y_ON()   GPIO_SetBits(LED2_PORT, LED2_PIN)   //橙灯置高电平

#define   LED3R_OFF()    GPIO_ResetBits(LED3_PORT, LED3_PIN) //蓝灯置低电平
#define   LED3R_ON()   GPIO_SetBits(LED3_PORT, LED3_PIN)   //蓝灯置高电平

#define 	KEY_WindTemp_port              GPIO0
#define 	KEY_WindTemp_port_pin          GPIO_Pin_8
#define     KEY_WindTemp()        GPIO_ReadInputDataBit(GPIO0, GPIO_Pin_8) //P0.6  风温按键

#define 	KEY_WindSpeed_port              GPIO0
#define 	KEY_WindSpeed_port_pin          GPIO_Pin_10
#define     KEY_WindSpeed()       GPIO_ReadInputDataBit(GPIO0, GPIO_Pin_10) //P0.10  风速按键



#define Gear_Speed_01                      (90000/60)                              // 1档转速               单位：转/min  
#define Gear_Speed_02                      (102700/60)//(1600)                     // 2档转速               单位：转/min 
#define Gear_Speed_03                      (1750)                                  // 3档转速               单位：转/min 

#define RUN_TIME_LIMIT                      (30)                                  // 开机时间限制   单位：分钟  

#define RUN_TIME                         RUN_TIME_LIMIT*60*2

#define VDC_SPEED_REDUCTION_COEF                      900

/* Macro Definitions--------------------------------------------------------*/
#define RemoteAddrNum                     (3)
#define NVR_TYPE                          (0)                                                     /* 0x800 NVM,  0 FLASH */
#define Rmt_NVM_Sector_StartAddr          (0x7E00)

#define MemoryMode                                           //屏蔽记忆使能




#define KEY_WindTemp_S                             1							//短按
#define KEY_WindTemp_L                             2							//长按


#endif 





