#ifndef LKSVIEWER_H_
#define LKSVIEWER_H_

#include "basic.h"
#include "SEGGER_RTT.h"

typedef enum
{
	REC_IDLE,
	REC_WAITCOND,
	REC_SAMPLING,
	REC_SAMPLED,
	REC_FINISH,
}enumRecState;

typedef enum
{
	CONDOPER_NULL,
	CONDOPER_GREATER,
	CONDOPER_LESSER,
}enumCondOper;

typedef struct
{
	void *var;											// 为方便处理，只认int32类型
	int  oper;
	int  thresValue;
}LKS_REC_COND;

typedef struct
{
	char name[12];										// mark as "LKS_VIEWER"
	u32  maxChannel;
	
	u32   chTimeBase;                                   // 时基   
	void* chAddr[5];									// 通道要存入的对象地址，最大支持5个通道
	u32   chLen[5];										// 通道要存入数据的宽度，最大支持5个通道
	
	// recorder参数，需要上位机配置的参数
	LKS_REC_COND recCond;								// 触发条件
	u32 postNum;										// 触发后采样的最大数量	
	u32 recStart;										// 上位机写1，启动Recorder
	u32 recReset;										// 上位机写1，复位Recorder，完全退出Recorder状态

	// 下位机使用的状态管理参数
	u32 recState;										// recorder的当前状态
	u32 sampleNum;										// 触发后采样的实际数量
	int lastValue;										// 条件变量的上一个值
}LKS_VIEWER_CB;

extern LKS_VIEWER_CB _LKS_VIEWER;


#ifdef __cplusplus
extern "C" {
#endif

/*
  启动时配置_LKS_VIEWER的原始值
*/
void LKS_VIEWER_DeInit(void);

/*
  初始化一个通道，指定通道使用缓冲区的地址和最大长度
*/
void LKS_VIEWER_InitBuffer(u32 channel, void *pBuffer, u32 bufferLen);

/*
  初始化一个通道，指定要写入的对象地址和对象长度
  调用LKS_VIEWER_Write()时，向该通道写入指定对象的数据
*/ 
void LKS_VIEWER_InitVar(u32 channel, void *pVar, u32 varLen);

/*
  从指定地址写数据到RTT中
  地址可以通过上位机修改
*/
void LKS_VIEWER_Write(u32 channel);
void LKS_VIEWER_WriteWithOverWrite(u32 channel);

/*
  执行LKS_VIEWER周期事件，根据上位机所发指令，
  写入缓冲区数据，或执行Recorder状态管理
*/
void LKS_VIEWER_Sched(void);

/*
  private函数，无需外部使用
*/
//void _RecInitMembers(void);
void _RecSaveBuffer(bool overWrite);	
bool _RecCondMatched(LKS_REC_COND *recCond);

#ifdef __cplusplus
}
#endif	  

#endif
