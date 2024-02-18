/*******************************************************************************
 * ��Ȩ���� (C)2022, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� HeatControl.h
 * �ļ���ʶ��
 * ����ժҪ�� ����˿����
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� Robin Chen
 * ������ڣ� 2022��12��13��
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
	u8  ACFrequencyDetectFinishFlag;  //������50Hz����60Hz�����ɱ�־   1��������  0��δ������
  u16 ACFrequencyDetectCnt;         //������50Hz����60Hz������
	u8  ACFrequency;                  //������50Hz����60HzƵ��
  u16 AC50HzFrequencyDetectCntMax;  //������50Hz����60Hz���������ֵ
  u16 AC50HzFrequencyDetectCntMin;  //������50Hz����60Hz��������Сֵ
  u16 AC60HzFrequencyDetectCntMax;  //������50Hz����60Hz���������ֵ
  u16 AC60HzFrequencyDetectCntMin;  //������50Hz����60Hz��������Сֵ
	
	u8  ACDetectErrorFlag;            //������Ƶ�ʼ������־   1������  0������
	u8  CrossZeroErrorFlag;           //���������������־   1������  0������
	
	u16 TurnOnDelayCnt;          //����˿�ӳٹ�������
	u8 TurnOnOffFlag;            //����˿������������־    1������˿������������  0������˿�ر�
	u8 CrossZeroState;           //��ǰ���������״̬
	u8 CrossZeroStateLast;       //�ϴν��������״̬
	s16 CrossZeroVoltage;				//��ǰADC��ѹֵ
	s16 CrossZeroVoltageLast;		//�ϴ�ADC��ѹֵ
	u16 CrossZeroVoltageCnt1;			//ADC����
	u16 CrossZeroVoltageCnt2;			//ADC����
	u8 CrossDownAcState;			//����״̬,
	u8 CrossDownAcStateCnt;			//����״̬,
	u8 ThrowWaveFlag;            //������־   1��������������  0�����ܶ���
	u8 CrossZeroCnt;             //�������
  u16 AC50HzThrowWaveDelayCnt; //������50Hz�����ӳ�ֵ
  u16 AC60HzThrowWaveDelayCnt; //������60Hz�����ӳ�ֵ
  u16 activecnt;
	u8 ThrowWaveMode;          //����ģʽ
	u8 ThrowWaveCnt;           //��������
	u8 ThrowWaveTabIndex;      //����������
	u32 ThrowWaveDelayCnt;     //������ʱ����
	
	u8  CrossZeroPosedge;  //�����ź�   1��������  0����ȷ��
	u8  CrossZeroPosedgeFlag;  //�����źű�־   1��������  0����ȷ��
	s16 TargetTemperature;     //����˿Ŀ���¶�ֵ
	s16 HEATER_NTCTemperature; //�ɼ�����˿ʵ���¶�ֵ
	s16 IPM_OTPTemperature;    //�ɼ�IPM(MOS)ʵ���¶�ֵ	

	u32 ThrowWaveBase;         //��������ֵ
	u32 ThrowWaveLimitMax;     //��󶪲�����ֵ
	u32 ThrowWaveLimitMin;     //��С��������ֵ

} HEAT_CONTROL;

#define   ZERO_PORT    GPIO0        //ZERO�˿�
#define   ZERO_PIN     GPIO_Pin_6   //ZERO����

#define  ZERO_SCR()    GPIO_ReadInputDataBit(ZERO_PORT, ZERO_PIN)//(GET_AC_ZERO_CHEAK_CHANNEL)//GPIO_ReadInputDataBit(GPIO1, GPIO_Pin_4)      //��ȡ�����źŵ�ƽ

#define   HEATER_CONTROL_PORT    GPIO0        //�¿ط���˿�˿�
#define   HEATER_CONTROL_PIN     GPIO_Pin_4   //�¿ط���˿����

#define  HEATER_CONTROL_ON()     GPIO_ResetBits(HEATER_CONTROL_PORT, HEATER_CONTROL_PIN)   //������˿  ��Ϊ�͵�ƽ
#define  HEATER_CONTROL_OFF()    GPIO_SetBits(HEATER_CONTROL_PORT, HEATER_CONTROL_PIN)     //�ط���˿  ��Ϊ�ߵ�ƽ

#define  TEM_SET1        (50)      //һ���¶��趨
#define  TEM_SET2        (60)     //�����¶��趨
#define  TEM_SET3        (80)     //�����¶��趨
#define  TEM_SET4        (100)     //�����¶��趨

#define  TEMP_CONSTANT_FUNCTION   (FUNCTION_ON)   // FUNCTION_ON������˿���¹���ʹ�ܣ�FUNCTION_OFF������˿���¹��ܲ�ʹ��
#define  PullUp            1     //���������赽5V   
#define  PullUpParall      2     //���������赽5V+NTC��������
#define  PullDown          3     //���������������赽5V
#define  NTCSAMPLE     (PullUpParall)

#define  PullUpResistor    (5.1)    //�������裬K��
#define  PullDownResistor  (10.0)    //�������裬K��
#define  ParallResistor    (39.0)    //�������裬K��

#define AD_TEMP_MAX_VAL			32767

#define  ThrowFullWave   0     //�����¶ȿ��ƶ�ȫ��
#define  ThrowHalfWave   1     //�����¶ȿ��ƶ��벨
#define  HEATER_DELAY_ON_TIME     (2000)          // ����˿�ӳٿ���ʱ�� ��λms


#define CROSSZERO_CNT1			15			//L// ms
#define CROSSZERO_CNT2			50			//L// ms
#define AC_ZERO_ON				0
#define AC_ZERO_OFF				1

//50Hz/60Hz������Ĺ����źż������ֵ����λ us��1s/22K = 45.4545us��
#define  AC50HzMax          240    //50Hz�����������ֵ  
#define  AC50HzMin          200    //50Hz����������Сֵ 
#define  AC60HzMax          200    //60Hz�����������ֵ 
#define  AC60HzMin          170    //60Hz����������Сֵ 

#define  AC50Hz          50    //50Hz������  ��׼ֵ200 = 10ms/45.4545
#define  AC60Hz          60    //60Hz������  ��׼ֵ183 = 8.33msms/45.4545

#define  Delay50Hz       11    //50Hz�ɿع��ӳٵ�ͨ 10 11 12   
#define  Delay60Hz       9     //60Hz�ɿع��ӳٵ�ͨ 8  9  10


#define  NTC210C   0.506     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC200C   0.604     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC190C   0.738     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC180C   0.911     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC170C   1.131     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC160C   1.450     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC150C   1.736     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC140C   2.229     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC130C   2.884     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC120C   3.767     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC110C   4.977     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC100C   6.67      //NTC�¶ȶ�Ӧ����ֵ
#define  NTC90C    9.09     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC80C    12.53     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC70C    17.50     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC60C    24.83     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC50C    35.89     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC40C    52.98     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC30C    80.29     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC20C    125.86    //NTC�¶ȶ�Ӧ����ֵ
#define  NTC10C    198.2     //NTC�¶ȶ�Ӧ����ֵ
#define  NTC00C    324.1    //NTC�¶ȶ�Ӧ����ֵ
#define  NTCn10C   541.9     //NTC�¶ȶ�Ӧ����ֵ
#define  NTCn20C   930.5    //NTC�¶ȶ�Ӧ����ֵ
#define  NTCn30C   1665.5  //NTC�¶ȶ�Ӧ����ֵ
#define  NTCn40C   3176.7  //NTC�¶ȶ�Ӧ����ֵ

extern HEAT_CONTROL FRSControl;                      /* ����˿���Ʋ��� */
extern void HeaterTemperature_Control(void);
void HeaterControlProcess(void);
extern void HeaterControl(void);
extern s16 NTCTempAD(s16 TargetTemp);
extern s32 NTCResistor10(s16 TargetTemp);
#endif

