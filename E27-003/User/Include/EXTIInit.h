/**************************** (C) COPYRIGHT 2017 Fortiortech shenzhen *****************************
* File Name          : EXTIInit.h
* Author             : Fortiortech Appliction Team
* Version            : V1.0
* Date               : 10-Apr-2017
* Description        : This file contains all the common data types used for
*                      Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/

/* Define to prevent recursive inclusion --------------------------------------------------------*/
#ifndef __EXTIINIT_H_
#define __EXTIINIT_H_

#if (HallMode == HallIC)
  #define  HA  P14
  #define  HB  P16
  #define  HC  P21

#elif (HallMode == HallSensor)
  #define  HA  ReadBit(CMP_SR, CMP0_OUT)
  #define  HB  ReadBit(CMP_SR, CMP1_OUT)
  #define  HC  ReadBit(CMP_SR, CMP2_OUT)

#endif

typedef struct
{  
    uint16 ScrOnAngleTimeDatum;  //�������� ��ͨ�ӳ�ʱ��  ��λΪ  1/�ز�Ƶ��
    uint16 ScrOnAngleTime;       //ʵ�ʸ��� ��ͨ�ӳ�ʱ��  ��λΪ  1/�ز�Ƶ��
  
    uint16 Temperature;                //�¶�ֵ
    uint16 TemperatureDatum;           //�¶�����ֵ
    uint32 TemperatureSum;             //�¶��ڵ�λʱ�����¶����
    uint16 TemperatureAverage;         //�¶��ڵ�λʱ�����¶�ƽ��ֵ
    uint16 TemperatureCount[10];         //�¶��ڵ�λʱ�����¶�ƽ��ֵ
  
    int16  TemperatureEkL;        //ǰһ��  �¶����
}USER_TYPEDEF;

/* Exported variables ---------------------------------------------------------------------------*/
extern USER_TYPEDEF  User;

/* Exported functions ---------------------------------------------------------------------------*/
extern void EXTI_Init(void);
extern void Sleepmode_Init(void);
extern void ZeroCrossing_Init(void);
extern void TemperaturePID(void);
extern void TempAverage(void);
extern uint8 const code HW_Gear[][2];
extern uint8 Get_Gcd(uint8 mm,uint8 nn);

#endif


