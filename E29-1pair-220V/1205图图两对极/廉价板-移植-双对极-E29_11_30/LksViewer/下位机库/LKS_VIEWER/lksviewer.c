#include "lksviewer.h"
#include <string.h>


LKS_VIEWER_CB _LKS_VIEWER;

void LKS_VIEWER_DeInit(void)
{
	int i = 0;
	
	strncpy(_LKS_VIEWER.name, "LKS_VIEWER", 12);
	_LKS_VIEWER.maxChannel = SEGGER_RTT_MAX_NUM_UP_BUFFERS;
	_LKS_VIEWER.chTimeBase = 0;
	for (i = 0; i < SEGGER_RTT_MAX_NUM_UP_BUFFERS; i++)
	{
		_LKS_VIEWER.chAddr[i] = NULL;
		_LKS_VIEWER.chLen[i] = 0;
	}
}

void LKS_VIEWER_InitBuffer(u32 channel, void *pBuffer, u32 bufferLen)
{
	// flag����ΪSEGGER_RTT_MODE_NO_BLOCK_SKIP
	// ��������ʣ��ռ䲻��ʱ��������д�������ݣ�����������ݲ�����
	if (channel < SEGGER_RTT_MAX_NUM_UP_BUFFERS)
	{
		SEGGER_RTT_ConfigUpBuffer(channel, "", pBuffer, bufferLen, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	}
}

void LKS_VIEWER_InitVar(u32 channel, void *pVar, u32 varLen)
{
	if (channel < SEGGER_RTT_MAX_NUM_UP_BUFFERS)
	{
		_LKS_VIEWER.chAddr[channel] = pVar;
		_LKS_VIEWER.chLen[channel] = varLen;
	}
}

bool _RecCondMatched(LKS_REC_COND *recCond)
{
	int curValue = 0;
	if (recCond == NULL || recCond->var == NULL || recCond->oper == CONDOPER_NULL)
	{
		return false;
	}

	curValue = *(int*)recCond->var;
	if (recCond->oper == CONDOPER_GREATER)			// ���򴥷�
	{
		if (_LKS_VIEWER.lastValue < recCond->thresValue && curValue >= recCond->thresValue)
		{
			return true;
		}
		else if (_LKS_VIEWER.lastValue == recCond->thresValue && curValue > recCond->thresValue)
		{
			return true;
		}
		else
		{
			_LKS_VIEWER.lastValue = curValue;
		}
	}
	else if (recCond->oper == CONDOPER_LESSER)		// ���򴥷�
	{
		if (_LKS_VIEWER.lastValue > recCond->thresValue && curValue <= recCond->thresValue)
		{
			return true;
		}
		else if (_LKS_VIEWER.lastValue == recCond->thresValue && curValue < recCond->thresValue)
		{
			return true;
		}
		else
		{
			_LKS_VIEWER.lastValue = curValue;
		}
	}
	
	return false;
}

void _RecSaveBuffer(bool overWrite)
{
	int i = 0;
	static u32 timeBaseCnt = 0;
	timeBaseCnt++;
	// ʱ������δ�ﵽ����д������
	if (_LKS_VIEWER.chTimeBase > 1 && timeBaseCnt < _LKS_VIEWER.chTimeBase)
	{
		return;
	}
	
	timeBaseCnt = 0;
	for (i = 0; i < SEGGER_RTT_MAX_NUM_UP_BUFFERS; i++)
	{
		if (overWrite)
		{
			LKS_VIEWER_WriteWithOverWrite(i);
		}
		else
		{
			LKS_VIEWER_Write(i);
		}
	}
}

void LKS_VIEWER_Write(u32 channel)
{
	if (channel >= SEGGER_RTT_MAX_NUM_UP_BUFFERS 
		|| _LKS_VIEWER.chAddr[channel] == NULL
		|| _LKS_VIEWER.chLen[channel] == 0)
	{
		return;
	}

	SEGGER_RTT_Write(channel, _LKS_VIEWER.chAddr[channel], _LKS_VIEWER.chLen[channel]);
}

void LKS_VIEWER_WriteWithOverWrite(u32 channel)
{
	if (channel >= SEGGER_RTT_MAX_NUM_UP_BUFFERS 
		|| _LKS_VIEWER.chAddr[channel] == NULL
		|| _LKS_VIEWER.chLen[channel] == 0)
	{
		return;
	}
	
	SEGGER_RTT_WriteWithOverwriteNoLock(channel, _LKS_VIEWER.chAddr[channel], _LKS_VIEWER.chLen[channel]);
}

void LKS_VIEWER_Sched(void)
{
	int i = 0;
	bool bufferEmpty = true;
	// �ж�ǿ�Ƹ�λ����
	if (_LKS_VIEWER.recReset)
	{
		_LKS_VIEWER.sampleNum = 0;
		_LKS_VIEWER.recReset = 0;
		_LKS_VIEWER.recState = REC_IDLE;
	}
	
	switch (_LKS_VIEWER.recState)
	{
	case REC_IDLE:
		// ����״̬��������������ΪRTT����ʹ�ã��������ǣ��������ݻ��ҡ��ȴ������ź�recStart
		_RecSaveBuffer(false);
		if (_LKS_VIEWER.recStart)
		{
			_LKS_VIEWER.recStart = 0;
			_LKS_VIEWER.recState = REC_WAITCOND;
			_LKS_VIEWER.lastValue = *(int*)_LKS_VIEWER.recCond.var;
		}
		break;
	case REC_WAITCOND:
		// Recorder���������жϴ�������������ʽ���»��������ݣ���ʱ��λ����ֹͣ��ȡ��
		_RecSaveBuffer(true);
		if (_RecCondMatched(&_LKS_VIEWER.recCond))
		{
			_LKS_VIEWER.recState = REC_SAMPLING;
			_LKS_VIEWER.sampleNum = 0;
		}
		break;
	case REC_SAMPLING:
		// ��������������ݲ�������������������
		if (_LKS_VIEWER.sampleNum < _LKS_VIEWER.postNum)
		{
			_RecSaveBuffer(true);
			_LKS_VIEWER.sampleNum++;
		}
		else
		{
			_LKS_VIEWER.recState = REC_SAMPLED;
		}
		break;
	case REC_SAMPLED:
		// ������ɣ��ȴ�������������������������ȫ�������󣬱��REC_FINISH
		for (i = 0; i < SEGGER_RTT_MAX_NUM_UP_BUFFERS; i++)
		{
			if (_LKS_VIEWER.chAddr[i] == NULL || _LKS_VIEWER.chLen[i] == 0)
			{
				continue;
			}
			if (SEGGER_RTT_HasDataUp(i) > 0)
			{
				bufferEmpty = false;
				break;
			}
		}
		if (bufferEmpty)
		{
			_LKS_VIEWER.recState = REC_FINISH;
		}
		break;
	case REC_FINISH:
		// ������ɣ��ȴ���λ���ĸ�λ�����λRecorder״̬
		if (_LKS_VIEWER.recReset)
		{
			_LKS_VIEWER.sampleNum = 0;
			_LKS_VIEWER.recReset = 0;
			_LKS_VIEWER.recState = REC_IDLE;
		}
		break;
	default:
		break;
	}
}
