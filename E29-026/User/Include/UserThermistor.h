/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
* File Name          : .h
* Author             : Fortiortech  Appliction Team
* Version            : V1.0
* Date               : 
* Description        : This file contains all the user interface parameter used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/ 

typedef signed char       s8;
typedef signed int        s16;
typedef signed long       s32;
typedef unsigned char     u8;
typedef unsigned int      u16;
typedef unsigned long     u32;

//extern uint16 Power_Currt;
//extern bit    PWM_EN;
//extern uint8  Sleep_En;

//// UserFrame.c
//extern u16 ADC2Smp,ADC3Smp,ADC6Smp,ADC7Smp;

//extern u8  Loop1msFlag;
//extern u8  Loop1msCnt;
//extern u8  Loop10msFlag;
//extern u8  Loop10msCnt;
//extern u16 StartX1ms;

//extern u16 IBusSmp;
//extern u16 UBusSmp;
//extern u16 UBusX10;

//extern u32 idata MotorRPM;

//extern u8  idata FlagHardBeark;            // Ӳ������
//extern u8  idata FlagOverCurrent;          // �������
//extern u8  idata FlagHighVolt;             // ��ѹ
//extern u8  idata FlagLowVolt;              // ��ѹ
//extern u8  idata FlagHighTemper;           // ����
//extern u8  idata FlagHighSpeed;            // ����
//extern u8  idata FlagStuck;                // ��ס
//extern u8  idata FlagPhaseLoss;            // ȱ��

//extern u8  idata AlarmHardBeark;           // 0 Ӳ������
//extern u8  idata AlarmOverCurrent;         // 1 �������
//extern u8  idata AlarmHighVolt;            // 2 ��ѹ
//extern u8  idata AlarmLowVolt;             // 3 ��ѹ
//extern u8  idata AlarmHighTemper;          // 4 ����
//extern u8  idata AlarmHighSpeed;           // 5 ����
//extern u8  idata AlarmStuck;               // 6 ��ס
//extern u8  idata AlarmPhaseLoss;           // 7 ȱ��
//extern u8  idata AlarmStartLose;
//extern u8  idata AlarmOverWind;            // 9 ˳�������쳣

//extern u8  idata AlarmFlag;                // ����ֵ

//extern u8  OCLv;                     // �����ȼ�
//extern u8  HiSpdLv;                  // ���ٵȼ�
//extern u8  StuckLv;                  // ��ת�ȼ�
//extern u8  StartFailLv;              // �����쳣�ȼ�
//extern u8  ReStartCnt;
//extern u8  StartFltFlag;
//extern u16 ReStartDlyX10ms;

//extern u8  HighSpdInhibitLv;
//extern uint8  HighSpdRestartCnt;
//extern uint16 HighSpdInhibitCnt;

//struct KeyIn{
//  uint8  Sample: 1;
//  uint8  Lv:     1;
//  uint8  PreLv:  1;
//  uint8  Rise:   1;
//  uint8  Fall:   1;
//};
//extern struct  KeyIn  KeyOn;

//extern s16 Vsp_Q12;

//extern u16 IbusADCOffset;

//extern u8  DrvEnFlag;
//extern s16 PulseDuty_Q12;

//extern u16 IBusCommand_mA;
//extern u16 IBusCommandAlt;
//extern u16 IBusFilt_mA;


//extern u8  ReStartCnt;
//extern u16 RunningX1ms;
//extern u16 ReStartDlyX10ms;
//extern u8  ReStartDlyCmd;
//extern u16 DrvTempADCSmpFilt;

//extern s32 VspRPMCmd;
//extern s32 VspRPMAlt;

//extern u16 KeyGoDuty;

//extern u8  KeyOnOffTrig;
//extern u8  KeyOnOffCmd;
//extern u16 PwrOnX1ms;
//extern u8  QuickStartFlag;

//// UserDryer.c
//extern s16 VspSpdAlt;
//extern s16 VspSpdCmd;
//extern s16 VspSpdCmdMax;
//extern u16 VspPwrDutyQ12;

//extern u32 idata UxISmp;
//extern u32 idata VspPwrCmd;

//// UserFlash.c
//extern u8  FlashSaveFlag;
