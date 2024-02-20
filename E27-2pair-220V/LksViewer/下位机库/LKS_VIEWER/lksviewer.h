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
	void *var;											// Ϊ���㴦��ֻ��int32����
	int  oper;
	int  thresValue;
}LKS_REC_COND;

typedef struct
{
	char name[12];										// mark as "LKS_VIEWER"
	u32  maxChannel;
	
	u32   chTimeBase;                                   // ʱ��   
	void* chAddr[5];									// ͨ��Ҫ����Ķ����ַ�����֧��5��ͨ��
	u32   chLen[5];										// ͨ��Ҫ�������ݵĿ�ȣ����֧��5��ͨ��
	
	// recorder��������Ҫ��λ�����õĲ���
	LKS_REC_COND recCond;								// ��������
	u32 postNum;										// ������������������	
	u32 recStart;										// ��λ��д1������Recorder
	u32 recReset;										// ��λ��д1����λRecorder����ȫ�˳�Recorder״̬

	// ��λ��ʹ�õ�״̬�������
	u32 recState;										// recorder�ĵ�ǰ״̬
	u32 sampleNum;										// �����������ʵ������
	int lastValue;										// ������������һ��ֵ
}LKS_VIEWER_CB;

extern LKS_VIEWER_CB _LKS_VIEWER;


#ifdef __cplusplus
extern "C" {
#endif

/*
  ����ʱ����_LKS_VIEWER��ԭʼֵ
*/
void LKS_VIEWER_DeInit(void);

/*
  ��ʼ��һ��ͨ����ָ��ͨ��ʹ�û������ĵ�ַ����󳤶�
*/
void LKS_VIEWER_InitBuffer(u32 channel, void *pBuffer, u32 bufferLen);

/*
  ��ʼ��һ��ͨ����ָ��Ҫд��Ķ����ַ�Ͷ��󳤶�
  ����LKS_VIEWER_Write()ʱ�����ͨ��д��ָ�����������
*/ 
void LKS_VIEWER_InitVar(u32 channel, void *pVar, u32 varLen);

/*
  ��ָ����ַд���ݵ�RTT��
  ��ַ����ͨ����λ���޸�
*/
void LKS_VIEWER_Write(u32 channel);
void LKS_VIEWER_WriteWithOverWrite(u32 channel);

/*
  ִ��LKS_VIEWER�����¼���������λ������ָ�
  д�뻺�������ݣ���ִ��Recorder״̬����
*/
void LKS_VIEWER_Sched(void);

/*
  private�����������ⲿʹ��
*/
//void _RecInitMembers(void);
void _RecSaveBuffer(bool overWrite);	
bool _RecCondMatched(LKS_REC_COND *recCond);

#ifdef __cplusplus
}
#endif	  

#endif
