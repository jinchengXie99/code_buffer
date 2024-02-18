/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� parameter.h
 * �ļ���ʶ��
 * ����ժҪ�� parameter config
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� Andrew kong, Howlet Li
 * ������ڣ� 2020��8��18��
 *
 *******************************************************************************/
 
/*------------------------------prevent recursive inclusion -------------------*/ 
#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "basic.h"
#include "MC_Type.h"

/* -----------------------------Hardware Parameter---------------------------- */
#define ADC_SUPPLY_VOLTAGE             (3.6)              /* ��λ: V  ADC��׼��ѹ��3.6����2.4,�󲿷�Ӧ��ѡ��3.6 */
#define AMPLIFICATION_GAIN             (18.18)            /* �˷ŷŴ��� */   //18.18
#define RSHUNT                         (0.200)             /* ��λ: ��  ����������ֵ */
#define VOLTAGE_SHUNT_RATIO            (6.8/(470.0*2+6.8)) /* ĸ�ߵ�ѹ��ѹ�� */
#define BEMF_SHUNT_RATIO               (1.0/(100.0*2 + 1.0)) /* �����Ƶ�ѹ��ѹ�� */

#define CURRENT_SAMPLE_TYPE            CURRENT_SAMPLE_2SHUNT /* ����������ʽѡ�� */

#define ROTOR_SENSOR_TYPE              ROTOR_SENSORLESS  /* ���λ�ô���������ѡ�� */

/* ------------------------------Rated Parameter------------------------------ */

#define U_RATED_VOLTAGE                (310.0)               /* ��λ:V, ����������ѹ������Ϊ����������ѹ */
#define U_RATED_CURRENT                (0.8)                /* ��λ:A, ����������������������ֵ */
#define U_MAX_FREQ                     (3000.0)            /* ��λ:Hz,  ����������ת�� */

/* ------------------------------Motor Parameter------------------------------ */
#define U_MOTOR_PP                     (1.0)              //���������   ��ҵ�� 1�Լ� 220V
#define U_MOTOR_RS                     (24.5)           //��λ: �� ��������
#define U_MOTOR_LD                     (1900.3)           //��λ: uH ���d����
#define U_MOTOR_LQ                     (2300.4)           //��λ: uH ���q����


//#define U_MOTOR_PP                     (1.0)              //���������   ������ 1�Լ� 220V
//#define U_MOTOR_RS                     (24.321)           //��λ: �� ��������
//#define U_MOTOR_LD                     (2008.6)           //��λ: uH ���d����
//#define U_MOTOR_LQ                     (2557.8)           //��λ: uH ���q����

//#define U_MOTOR_PP                     (1.0)              //���������   ����ǻԵ�� 1�Լ� 220V
//#define U_MOTOR_RS                     (21.887)           //��λ: �� ��������
//#define U_MOTOR_LD                     (1951.985)           //��λ: uH ���d����
//#define U_MOTOR_LQ                     (2569.775)           //��λ: uH ���q����

//#define U_MOTOR_PP                     (1.0)              //���������   ���͵�� 1�Լ� 110V
//#define U_MOTOR_RS                     (6.055)           //��λ: �� ��������
//#define U_MOTOR_LD                     (453.29)           //��λ: uH ���d����
//#define U_MOTOR_LQ                     (693.62)           //��λ: uH ���q����

/* ����������� ���㹫ʽ��Vpp/2/sqrt(3)/(2*PI)/f������VppΪ��ѹ���ֵ��fΪ��Ƶ�� 
   �˲�����Ӱ��˳����������ٶȼ�⣬�Ƕȹ��㲻ʹ��Щ���� */
#define U_MOTOR_FLUX_CONST             (0.00142328)

/* ----------------------IPD ת�ӳ�ʼλ�ü�� ����ע��ʱ������---------------- */
#define SEEK_POSITION_STATUS           (FALSE)             /* ��ʼλ�ü��״̬ TRUEΪʹ��, FALSEΪ��ʹ�� */
#define U_START_ANGLE_COMP             (0)                /* ��λ:�� ��ʼλ�ü�ⲹ���Ƕ� */

#define IPD_PLUS_TIME_SETTING          (1)              /* ����ע��ʱ�������� ��λus */
#define IPD_WAIT_TIME_SETTING          (50)              /* ������еȴ�ʱ�������� ��λus */

#define IPD_PLUS_TIME_WIDTH            (u32)(IPD_PLUS_TIME_SETTING*(MCU_MCLK/1000000))   /* ����ע��ʱ�������� ��λclk */
#define IPD_PLUS_WAIT_TIME             (u32)(IPD_WAIT_TIME_SETTING*(MCU_MCLK/1000000))   /* ������еȴ�ʱ�������� ��λclk */

/* -------------------------------HALL��ѧϰ����------------------------------- */
#define HALL_LEARN_CURRENT_SETTINT     (1.0)              /* HALL��ѧϰ�����趨,��λ: A */

/* ----------------------------direction check Parameter----------------------- */
#define CW                             (0)                /* ���ת��˳ʱ�� */ 
#define CCW                            (1)                /* ���ת����ʱ��*/

/* ------------------------------ADCУ׼��ز�������---------------------------- */
#define CALIB_SAMPLES                  (512)              /* ADCƫ��У׼�����������޸� */
#define OFFSET_ERROR                   (3500)             /* ADCƫ�������ֵ�������޸� */

/* ----------------------------Ԥ���Ծٵ���Ԥ������--------------------------- */
#define CHARGE_TIME                    (100)              /* ÿ��Ԥ���ʱ�䣬����ʵ��Ӳ�������޸� */

/* -------------------------------- ˳��������------------------------------ */
#define SPEED_TRACK_DELAYTIME          (100)              /* ��λ: ms ˳�����ʱ��  */
#define SPEED_TRACK_ON_FREQ_THH        (10)               /* ��λ: Hz ˳���бջ�Ƶ�� */
#define EBRK_ON_FREQ_THH               (20)               /* ��λ: Hz ���ɲ��Ƶ�� */

#define MOTOR_STOP_CUR_THD             (120)              /* ���ֹͣ��������ֵ */
#define MOTOR_STOP_CUR_DIF_THD         (12)               /* ���ֹͣ����������ֵ */

#define STOP_TIME                      (250)               /* ��λ��ms ���ֹͣ����˲�ʱ�䣬����ʵ�ʸ����޸� */
#define STOP_DELAY_TIME                (200)              /* ��λ: ms ���ֹͣ���ӳ�ʱ�䣬����ʵ�ʸ����޸ġ��޸����ݣ�������ж�Ϊֹͣ����ת���ͼӴ��ӳ�ʱ�� */

#define CLOSE_VOLT                     (1000)              /**/
/*------------------------------------Ԥ��λ����---------------------------------*/
#define ALIGN_ANGLE                    (0)                /* ��λ:�� Ԥ��λ�Ƕ� */
#define U_START_CUR_SET_F              (0.0)              /* ��λ: A ��һ�ζ�λ���� */
#define U_START_CUR_SET_S              (0.55)              /* ��λ: A �ڶ��ζ�λ���� */
#define DC_HOLDTIME_TIME_LENTH         (10)               /* ��λ: ms ��һ�ζ�λʱ�� */
#define DC_HOLDTIME_TIME_LENTH_STAGE1  (10)               /* ��λ: ms �ڶ��ζ�λʱ�� */
#define DC_ALIGN_TOTAL_LENTH            \
       (DC_HOLDTIME_TIME_LENTH + DC_HOLDTIME_TIME_LENTH_STAGE1)/* ��λ��ʱ�� */

#define ALIGN_CURRENT_ACC              (0.01)              /* ��λ: (1/8)A/ms  ��λ�������ٵ���ֵ  ��ʼλ�ü��ʹ�ܺ�������ֵ�����ܳ���30���������ݻ������ */
#define ALIGN_CURRENT_DEC              (0.01)              /* ��λ: (1/8)A/ms  ��λ�������ٵ���ֵ  ��ʼλ�ü��ʹ�ܺ�������ֵ�����ܳ���30���������ݻ������ */

/*---------------------------------��������------------------------------------*/
#define OPEN_ANGLE_TAG_FREQ            (60.0)             /* ��λ��Hz �����϶�����Ƶ�� */
#define FREQ_ACC                       (0.0)              /* ��λ��(1/128)Hz/ms �����϶�Ƶ�ʼ��ٵ���ֵ */
#define FREQ_DEC                       (20.0)              /* ��λ��(1/128)Hz/ms �����϶�Ƶ�ʼ��ٵ���ֵ */

#define OPEN_RUN_STATUS                (FALSE)            /* ����״̬ TRUE = ��������, FALSE = �ջ����� */
  
#define MATCH_TIME                     (5)                /* ����͸�������ƥ����� */ 

/*---------------------- -----�����ջ��л����ɲ���------------------------------*/
#define OPEN2CLOSE_RUN_COV_TIME        (2)               /* �����ջ��л�����ʱ�䣺��λ��mS */
#define OPEN2CLOSE_RUN_CURRENT_RAMP    (0.1)             /* �����ջ��л������ڣ�D,Q������仯б�ʡ���λ��A/ms */

/*-----------------------------------��·ѡ��----------------------------------*/
#define CURRENT_LOOP                   (0)                /* ������ */
#define SPEED_LOOP                     (1)                /* �ٶȻ� */
#define POWER_LOOP                     (2)                /* ���ʻ� */
#define CLOSE_LOOP                     (SPEED_LOOP)      /* ��·ѡ�� */

/* ------------------------------��������----------------------------- */
#define IQ_START                       (0.1)               /* ��λ */

#define PLL_KP_GAIN                    (0)                 /* PLL_Kp ������Kp  start*/ //800
#define PLL_KI_GAIN                    (0)                  /* PLL_Ki ������Ki */  //300

#define PLL_KP_GAIN_L                  (1600)                  /* PLL_Kp ������Kp */ //800
#define PLL_KI_GAIN_L                  (100)                  /* PLL_Ki ������Ki */ //50

//#define PLL_KP_GAIN_H                  (1000)                 /* PLL_Kp ������Kp */ //400
//#define PLL_KI_GAIN_H                  (200)                  /* PLL_Ki ������Ki */ //10

#define PI_MATCH_FREQ                  (30.0)              /* ��λ��Hz PI��������Ƶ�� */
#define CLOSE_FREQ                     (500.0)              /* ��λ��Hz ���빦�ʻ���Ƶ�� */

#define OBS_MIN_OUT_SPEED              (150)                  /* �۲�����С�����Ƶ�ʣ���λ��0.1Hz*/  //50
#define OBS_MAX_OUT_SPEED              0//(3000)                /* �۲�����С�����Ƶ�ʣ���λ��0.1Hz*/  //2000

#define START_DIVISOR                  (5.0)               /*��������--��λ*/  //3
#define START_OBSFACTOR                (30.0)              /*��������--ϵ��*/ //65

#define OBS_COEF                        FRAC16(0.05)        /* �۲����˲�ϵ�� 0~1  */  //0.25

/*----------------------------------����������---------------------------------*/
#define IQ_SET                         (0.40)              /* ��λ��A IqRef��Iq����ֵ */
 
#define VQMAX                          (6000)             /* Q�����������ƣ�Q15��ʽ��ȡֵ��Χ0~6000 */
#define VQMIN                          (-VQMAX)            /* Q����С������ƣ�Q15��ʽ��ȡֵ��Χ0~-6000 */

#define VDMAX                          (VQMAX)             /* D�����������ƣ�Q15��ʽ��ȡֵ��Χ0~6000 */
#define VDMIN                          (-VQMAX)            /* D����С������ƣ�Q15��ʽ��ȡֵ��Χ0~6000 */

#define P_CURRENT_KP                   4500               /* ������Kp��ʵ�����õ�Kp��������ֵ�͵��������������յ�Kp */
#define P_CURRENT_KI                   700                /* ������Ki��ʵ�����õ�Kp��������ֵ�͵��������������յ�Ki */

#define P_CURRENT_KP_H                 3338              /* ���ٵ�����Kp�� 1500= 1.5��*/  //1200
#define P_CURRENT_KI_H                 140000               /* ���� ������Ki��1500= 1.5�� */ //1200

#define AUTO_FW_LIM                    ((s16)0)        /* �Զ�����D��������ƣ�Q12��ʽ�����ֵ 4096 */

#define TORQUE_MODE_CURRENT_CHANGE_ACC (0.1)              /* ��λ��A/ms �������ٵ���ֵ */
#define TORQUE_MODE_CURRENT_CHANGE_DEC (0.1)              /* ��λ��A/ms �������ٵ���ֵ */

/*----------------------------------�ٶȻ�����-------------------------------*/
#define POWER_LIMIT_STATUS             (FALSE)            /* �޹���״̬��TRUE = ʹ�ܣ�FALSE = ��ʹ�� */
#define POWER_LIMIT_VALUE              (10.0)             /* ��λ��W   ���ƹ��ʵĴ�С */
#define POWER_LIMIT_TIME               (5)                /* ��λ���ٶȻ����ڣ�  �޹��ʼ������� */
#define POWER_LIMIT_SPEED              (10)               /* ��λ��Hz  �޹���ת�ٸ���������ʵ��Ӧ�������ã� */


#define POWER_CHECK                    (23)               /* ��λ������У��ֵ */
#define POWER_CHECK1                   (34)               /* ��λ������У��ֵ */
#define SPEED_SET                      (1834)               /* ��λ��Hz  �ٶȸ���ֵ  1450 1834 */
#define SPEED_LOOP_CNTR                (0)                /* ��λ��ms  �ٶȻ�·�������� */

#define STATE04_WAITE_TIME             (2)              /* Unit: ms �ٶȱ�����ʼ��ʱ�� */

#define P_ASR_KP                      4000 //4000//2000//(3000)             /* �ٶȻ�Kp */
#define P_ASR_KI                      6000//6000//3000//(5000)              /* �ٶȻ�Ki */

#define IQMAX                          (0.6)                /* ��λ:A, �ٶȻ�������ֵ */
#define IQMIN                          (-0.4)               /* ��λ:A, �ٶȻ������Сֵ */

#define SPEED_RUN_ACC                  (3.0)            /* ��λ (1/128)Hz �ٶȼ��ٵ���ֵ */
#define SPEED_RUN_DEC                  (3.0)              /* ��λ (1/128)Hz �ٶȼ��ٵ���ֵ */

/* ɲ������ */
#define BRAKE_IQ_EN                    (1)             /* IQɲ�� TRUEΪʹ��, FALSEΪ��ʹ�� */ 
#define BRAKE_CURRENT                  (-0.55)             /* ��λ��A ɲ������ */
#define BRAKE_SPEED                    (350)              /* ��λ��Hz ɲ��ת�� */
#define BRAKE_P_CURRENT_KP              100                /* ɲ��������Kp */
#define BRAKE_P_CURRENT_KI              44                /* ɲ��������Ki */

/*------------------------------------���ʻ�����-------------------------------*/
#define SPPED_LIMIT_STATUS             (FALSE)            /*  ��ת��״̬��TRUE = ʹ�ܣ�FALSE = ��ʹ�� */
#define SPEED_LIMIT_VALUE              (200.0)            /* ��λ��Hz  ����ת�ٵĴ�С */
#define SPEED_LIMIT_TIME               (5)                /* ��λ��ms  ���ʻ����ڣ� ��ת�ټ������� */
#define SPEED_LIMIT_POWER_VALUE        (10)               /* ��λ��W   ��ת�ٹ��ʸ��� */

#define POWER_SET                      (30.0)               /* ��λ��W  ���ʸ���ֵ */
#define POWER_LOOP_CNTR                (1)                /* ��λ��ms  ���ʻ�·�������� */
                                                                  
#define POWER_KP                       (6000)             /* ���ʻ�Kp */
#define POWER_KI                       (600)              /* ���ʻ�Ki */

#define POWER_IQMAX                    (3.8)              /* ��λ:A, ���ʻ�������ֵ */
#define POWER_IQMIN                    (-3.8)             /* ��λ:A, ���ʻ������Сֵ */
                                                                   
#define POWER_RUN_ACC                  (2.0)              /* ��λ w ���ʼ��ٵ���ֵ ע��POWER_RUN_ACC��POWER_RUN_DEC����̫С�������ϲ������ã�ʵ�ʳ����еļӼ���ֵ������С��0. */
#define POWER_RUN_DEC                  (2.0)              /* ��λ w ���ʼ��ٵ���ֵ */

/*------------------------------------FaultDetection---------------------------*/
/* ���������� */
#define I_PH_OVERCURRENT_FAULT         (2.0)              /* ��λ��A �����������趨ֵ */

/* ��Ƿѹ������ */
#define U_DCB_OVERVOLTAGE_FAULT        (360.0)               /* ��λ��V ��ѹ����趨ֵ */
#define U_DCB_OVERVOLTAGE_RECOVER      (350.0)               /* ��λ��V ��ѹ�ָ��趨ֵ */
#define U_DCB_UNDERVOLTAGE_FAULT       (254.0)               /* ��λ��V Ƿѹ����趨ֵ */
#define U_DCB_UNDERVOLTAGE_RECOVER     (270.0)               /* ��λ��V Ƿѹ�ָ��趨ֵ */

/* ��ˮ��ת���� */
#define I_PH_EMPTY_FAULT               (0.15)              /* ��λ��A ��ת�������趨ֵ */
#define SPEED_EMPTY_FAULT              (180.0)             /* ��λ��Hz ��ת���ת���趨ֵ  */

/* �¶ȼ����� */
#define TEMP_FAULT                     (140)              /* ���¼���趨ֵ */
#define TEMP_RECOVER                   (70)              /* ���»ָ��趨ֵ */
#define TEMP_BREAK                     (120)             /* NTC��·�趨ֵ */
                                                                  
/* ��ת������ */
#define SPEED_STALL_MAX_FAULT          (2200.0)           /* ��λ��Hz ��ת���ת�����ֵ */
#define SPEED_STALL_MIN_FAULT          (50.0)             /* ��λ��Hz ��ת���ת����Сֵ */

#define I_PH_STALL_FAULT               (1.4)              /* ��λ��A ��ת�������趨ֵ */

#define SPEED_STALL_FAULT              (30.0)             /* ��λ��Hz ��ת���ת���趨ֵ */
#define IQ_STALL_FAULT                 (0.1)              /* ��λ��A ��ת�������趨ֵ */

#define STALL_CHCEK_TIME               (150)               /* ��λ��5ms ��ת���ʱ��*/   //100

#define  STALL_POWER1                  (78)               /* ��λ��w �¿׹���1*/   //100
#define  STALL_POWER2                  (117)               /* ��λ��W �¿׹���2*/   //100
#define  RESTART_CNT                    (2)                  // ������������

/* �������������� */
#define START_TIME_FAULT               (100)              /* ��λ��5ms ����֮��1s�ڻ�������ջ���������1s���ʱ�����ʵ��Ӧ�õ��� */
#define STARTUP_FAILED_TIME_5MS        (100)
/* ȱ������� */
#define I_PHASE_LOSS_FAULT             (3000)             /* ������������ʵ�ʲ�����struCurrentAmplitude.nPhA/struCurrentAmplitude.nPhB/struCurrentAmplitude.nPhC */
                                                          /* �ļ���ֵ���趨����ȱ����������е��м�ȡֵ�� */
#define PHASE_CHCEK_TIME               (40)               /* ��λ��5ms ȱ����ʱ��*/

/* ���ϻָ�ʱ�� */
#define VOLT_FAULT_RECOVER_TIME        (300)             /* ��λ��ms  ��Ƿѹ�ָ�ʱ�� */
#define CURRENT_FAULT_RECOVER_TIME     (300)             /* ��λ��ms  �����ָ�ʱ�� */
#define STALL_FAULT_RECOVER_TIME       (300)             /* ��λ��ms  ��ת�ָ�ʱ�� */
#define PHASELOSS_FAULT_RECOVER_TIME   (300)             /* ��λ��ms  ȱ��ָ�ʱ�� */
#define TEMP_FAULT_RECOVER_TIME        (2000)             /* ��λ��ms  ���»ָ�ʱ�� */
#define START_FAULT_RECOVER_TIME       (300)             /* ��λ��ms  ���������ָ�ʱ�� */
#define EMPTY_FAULT_RECOVER_TIME       (300)             /* ��λ��ms  ��ˮ��ת�ָ�ʱ�� */



#endif  /* __PARAMETER_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* -----------------------------------END OF FILE------------------------------- */

