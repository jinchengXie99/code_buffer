/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� Time_Process.h
 * �ļ���ʶ��
 * ����ժҪ�� ��ʱ��غ���
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet Li
 * ������ڣ� 2020��8��16��
 *
 * �޸ļ�¼1��
 * �޸����ڣ�2020��8��16��
 * �� �� �ţ�V 1.0
 * �� �� �ˣ�Howlet Li
 * �޸����ݣ�����
 *
 *******************************************************************************/
/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __TIME_PROCESS_H
#define __TIME_PROCESS_H

#include "basic.h"
#include "Global_Variable.h"

#define PWM_TIME_500uS                    (u16)(PWM_FREQ/1000/2)
#define PWM_TIME_1MS                      (u16)(PWM_FREQ/1000)
#define PWM_TIME_2MS                      (u16)(2*PWM_FREQ/1000)
#define PWM_TIME_4MS                      (u16)(4*PWM_FREQ/1000)
#define PWM_TIME_10MS                     (u16)(20*PWM_FREQ/1000)
#define PWM_TIME_20MS                     (u16)(20*PWM_FREQ/1000)

#define TASK_SCHEDU_05MS                  (1)                                      /* �������0.5ms����ʱ�� */
#define TASK_SCHEDU_1MS                   (10)                                      /* �������1ms����ʱ�� */
#define TASK_SCHEDU_10MS                  (100)                                     /* �������1ms����ʱ�� */
#define TASK_SCHEDU_100MS                 (1000)                                    /* �������100ms����ʱ�� */
#define TASK_SCHEDU_500MS                 (5000)                                   /* �������500ms����ʱ�� */

#define TIMER_PERIOD                      (u32)500                                 /* ��ʱ500us, ��λus */
#define TIMER_500US_TH                    (u32)(MCU_MCLK/1000000*TIMER_PERIOD - 1) /* MCU_MCLK/1000000 * PERIOD  */

#define HALL_OVERTIME_1MS                 (MCU_MCLK/1000)                          /* Hallģ�鳬ʱ1ms, ��Ƶϵ��Ϊ1 */
#define HALL_OVERTIME_100MS               (u32)(HALL_OVERTIME_1MS*100)             /* Hallģ�鳬ʱ100ms  */

#define STATE_MACHINE_200MS               (200)                                    /* ״̬��200ms����ʱ�� */
#define STATE_MACHINE_10MS                (10)                                     /* ״̬��10ms����ʱ�� */

/* Includes ------------------------------------------------------------------*/
/*******************************************************************************
 �������ƣ�    void SetTimeOut_Counter(u16 hTimeout)
 ����������    ���õȴ������ȴ�ʱ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
#define SetTimeOut_Counter(SetTimeLeftCnt, hTimeout)  {SetTimeLeftCnt = hTimeout;}


/*******************************************************************************
 �������ƣ�    u8 SetTime_IsElapsed(void)
 ����������    �жϵȴ�ʱ���Ƿ�
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
#define SetTime_IsElapsed(SetTimeLeftCnt)     (SetTimeLeftCnt)

/*******************************************************************************
 �������ƣ�    void SetTime_CountDown(void)
 ����������    ������������ʱ����
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
static __inline void SetTime_CountDown(void)
{
    if(struFOC_CtrProc.nSetTimeLeftCnt)
    {   /* ״̬���ȴ������ȴ�ʱ�䰴1mS�ݼ� */
        struFOC_CtrProc.nSetTimeLeftCnt--;
    }
}

void SoftDelay(u32 cnt);

#endif
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
