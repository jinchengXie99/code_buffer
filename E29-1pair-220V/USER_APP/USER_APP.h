/*******************************************************************************
 * ��Ȩ���� (C)2020, Bright Power Semiconductor Co.ltd
 *
 * �ļ����ƣ� USER_APP.h
 * �ļ���ʶ��
 * ����ժҪ�� 
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� BPS IOT TEAM.
 * ������ڣ� 2020��1��26��
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

#define   LED1_PORT    GPIO1        //LED1�˿�
#define   LED1_PIN     GPIO_Pin_4   //LED1����

#define   LED2_PORT    GPIO0        //LED2�˿�
#define   LED2_PIN     GPIO_Pin_1   //LED2����

#define   LED3_PORT    GPIO0        //LED3�˿�
#define   LED3_PIN     GPIO_Pin_3   //LED3����

#define   LED1B_OFF()    GPIO_ResetBits(LED1_PORT, LED1_PIN) //����õ͵�ƽ
#define   LED1B_ON()   GPIO_SetBits(LED1_PORT, LED1_PIN)   //����øߵ�ƽ

#define   LED2Y_OFF()    GPIO_ResetBits(LED2_PORT, LED2_PIN) //�ȵ��õ͵�ƽ
#define   LED2Y_ON()   GPIO_SetBits(LED2_PORT, LED2_PIN)   //�ȵ��øߵ�ƽ

#define   LED3R_OFF()    GPIO_ResetBits(LED3_PORT, LED3_PIN) //�����õ͵�ƽ
#define   LED3R_ON()   GPIO_SetBits(LED3_PORT, LED3_PIN)   //�����øߵ�ƽ

#define 	KEY_WindTemp_port              GPIO0
#define 	KEY_WindTemp_port_pin          GPIO_Pin_8
#define     KEY_WindTemp()        GPIO_ReadInputDataBit(GPIO0, GPIO_Pin_8) //P0.6  ���°���

#define 	KEY_WindSpeed_port              GPIO0
#define 	KEY_WindSpeed_port_pin          GPIO_Pin_10
#define     KEY_WindSpeed()       GPIO_ReadInputDataBit(GPIO0, GPIO_Pin_10) //P0.10  ���ٰ���



#define Gear_Speed_01                      (90000/60)                              // 1��ת��               ��λ��ת/min  
#define Gear_Speed_02                      (102700/60)//(1600)                     // 2��ת��               ��λ��ת/min 
#define Gear_Speed_03                      (1750)                                  // 3��ת��               ��λ��ת/min 

#define RUN_TIME_LIMIT                      (30)                                  // ����ʱ������   ��λ������  

#define RUN_TIME                         RUN_TIME_LIMIT*60*2

#define VDC_SPEED_REDUCTION_COEF                      900

/* Macro Definitions--------------------------------------------------------*/
#define RemoteAddrNum                     (3)
#define NVR_TYPE                          (0)                                                     /* 0x800 NVM,  0 FLASH */
#define Rmt_NVM_Sector_StartAddr          (0x7E00)

#define MemoryMode                                           //���μ���ʹ��




#define KEY_WindTemp_S                             1							//�̰�
#define KEY_WindTemp_L                             2							//����


#endif 





