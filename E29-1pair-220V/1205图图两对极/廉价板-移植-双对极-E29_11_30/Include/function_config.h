/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： function_config.h
 * 文件标识：
 * 内容摘要： 控制参数相关头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet
 * 完成日期： 2020年8月5日
 *
 * 修改记录1：
 * 修改日期： 2020年8月5日
 * 版 本 号： V 1.0
 * 修 改 人： Howlet
 * 修改内容： 创建
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __FUNCTION_CFG_PARAM_H
#define __FUNCTION_CFG_PARAM_H

#include "hardware_config.h"

/* -------------------------------ADC 读偏置次数——---------------------------- */
#define ADC_GET_OFFSET_SAMPLES         (9)           /* 偏置采样次数2的9次方, 512次 */
#define ADC_OFFSET_ERROR               (3500)        /* ADC读偏置错误 */

/* -------------------------------存储地址相关定义---------------------------- */ 
#define HALL_LEARN_ADDR                0x7600       /* Hall学习存储存地址 */


/* ------------------------------FOC功能相关定义------------------------------ */
#define MAX_MODULE_VALUE               18919    /* PWM调制比 1/sqrt(3)*32767 */ 

/* ---------------------------------功能相关定义------------------------------ */
#define TEST_ON                        1
#define TEST_OFF                       0

#define DEBUG_PWM_OUTPUT               TEST_OFF       /* PWM在调试的时候输出一定占空比 */

#define ALTERNATE_OUT_PWM              TEST_OFF      /* PWM输出调试，交替输出Ch012 CH3 */

#define OPEN_LOOP_FORCE_TEST           TEST_OFF       // 强制电压输出


#define FUNCTION_ON                    1
#define FUNCTION_OFF                   0

#define RTT_FUNCTION                   FUNCTION_OFF   /* RTT 调试功能 */


#endif /* __FUNCTION_CFG_PARAM_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
