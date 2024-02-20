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
	// flag设置为SEGGER_RTT_MODE_NO_BLOCK_SKIP
	// 当缓冲区剩余空间不足时，不允许写入半个数据，避免造成数据不完整
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
	if (recCond->oper == CONDOPER_GREATER)			// 正向触发
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
	else if (recCond->oper == CONDOPER_LESSER)		// 反向触发
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
	// 时基计数未达到，不写缓冲区
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
	// 判断强制复位命令
	if (_LKS_VIEWER.recReset)
	{
		_LKS_VIEWER.sampleNum = 0;
		_LKS_VIEWER.recReset = 0;
		_LKS_VIEWER.recState = REC_IDLE;
	}
	
	switch (_LKS_VIEWER.recState)
	{
	case REC_IDLE:
		// 空闲状态，持续采样，作为RTT缓冲使用，不允许覆盖，避免数据混乱。等待启动信号recStart
		_RecSaveBuffer(false);
		if (_LKS_VIEWER.recStart)
		{
			_LKS_VIEWER.recStart = 0;
			_LKS_VIEWER.recState = REC_WAITCOND;
			_LKS_VIEWER.lastValue = *(int*)_LKS_VIEWER.recCond.var;
		}
		break;
	case REC_WAITCOND:
		// Recorder已启动，判断触发条件。覆盖式更新缓冲区数据，此时上位机已停止读取。
		_RecSaveBuffer(true);
		if (_RecCondMatched(&_LKS_VIEWER.recCond))
		{
			_LKS_VIEWER.recState = REC_SAMPLING;
			_LKS_VIEWER.sampleNum = 0;
		}
		break;
	case REC_SAMPLING:
		// 触发条件后的数据采样，限制最大采样点数
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
		// 采样完成，等待缓冲区被读出。缓冲区数据全部读出后，标记REC_FINISH
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
		// 采样完成，等待上位机的复位命令，复位Recorder状态
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
