#include "comdata.h"
#include "lks32mc08x_uart.h"
#include <string.h>
#include "rtt_debug.h"


u8 recvData[MAXRECV_LEN] = {0};
u8 recvLen = 0;

void InitRecvData(void)
{
	memset(recvData, 0, MAXRECV_LEN);
	recvLen = 0;
}

void AddRecv(u8 data)
{
	if (recvLen == 0 && data != DATA_HEAD0)
	{
		return;
	}
	if (recvLen >= MAXRECV_LEN)
	{
		return;
	}

	recvData[recvLen] = data;
	recvLen++;							//已接收数据长度
}

void CheckRecv(void)
{
    if(recvLen < MINRECV_LEN)			//收集的数据不够最短的有效报文长度
    {
        return;
    }

    if(recvData[INDEX_HEAD] == DATA_HEAD0)
    {
		u8 byGramLen = recvData[INDEX_LEN] + PACKHEAD_LEN;
		if (byGramLen > MAXRECV_LEN)	// 长度标识异常
		{
			NVIC_DisableIRQ(UART0_IRQn);
			HandleError();			
			NVIC_EnableIRQ(UART0_IRQn);
			PRINTF("err length\r\n");
			return;
		}
		
        if(recvLen >= byGramLen)		// 收够一包数据
        {
			u8 crc = recvData[INDEX_CRC];
			u8 crc2 = CalcCRC(recvData+INDEX_CMD, byGramLen-PACKHEAD_LEN);
			if (crc == crc2)
			{
				NVIC_DisableIRQ(UART0_IRQn);
				HandleCmd();
				NVIC_EnableIRQ(UART0_IRQn);
			}
			else
			{
				NVIC_DisableIRQ(UART0_IRQn);
				HandleError();
				NVIC_EnableIRQ(UART0_IRQn);
				PRINTF("err crc\r\n");
			}
        }
    }
    else
    {
		NVIC_DisableIRQ(UART0_IRQn);
		HandleError();
		NVIC_EnableIRQ(UART0_IRQn);
		PRINTF("err head\r\n");
    }
}

void HandleCmd(void)
{
	u8 byCMD = recvData[INDEX_CMD];
	switch (byCMD)
	{
	case CMD_READMEM:
		HandleReadMem();
		break;
	case CMD_WRITEMEM:
		HandleWriteMem();
		break;
	default:
		PRINTF("unsupported cmd: %d\r\n", byCMD);
		break;
	}
	InitRecvData();
}

void HandleError(void)
{
	InitRecvData();
}

void HandleReadMem(void)
{
	u32 readAddr = CharToU32(&recvData[INDEX_ADDR]);
	u8 readLen = recvData[INDEX_RLEN];

	// 组织回复
	u8 sendData[MAXSEND_LEN] = {0};
	u8 index = 0;

	sendData[index++] = DATA_HEAD0;
	sendData[index++] = 0;
	sendData[index++] = 0;
	sendData[index++] = CMD_READMEM;
	memcpy(&sendData[index], (const void*)readAddr, readLen);
	index += readLen;

	sendData[INDEX_LEN] = index - PACKHEAD_LEN;
	sendData[INDEX_CRC] = CalcCRC(&sendData[INDEX_CMD], index - PACKHEAD_LEN);

	SendData(sendData, index);
}

void HandleWriteMem(void)
{
	u32 writeAddr = CharToU32(&recvData[INDEX_ADDR]);
	u8 writeLen = recvLen - INDEX_WDATA;
	memcpy((void*)writeAddr, &recvData[INDEX_WDATA], writeLen);
	
	// 组织回复
	u8 sendData[MAXSEND_LEN] = {0};
	u8 index = 0;
	
	sendData[index++] = DATA_HEAD0;
	sendData[index++] = 0;
	sendData[index++] = 0;
	sendData[index++] = CMD_WRITEMEM;
	sendData[index++] = 1;						// 成功

	sendData[INDEX_LEN] = index - PACKHEAD_LEN;
	sendData[INDEX_CRC] = CalcCRC(&sendData[INDEX_CMD], index - PACKHEAD_LEN);

	SendData(sendData, index);
}

void SendData(u8 *pData, u8 len)
{
    for (u8 i = 0; i < len; i++)
    {
		u16 dealy = 60000;
		
        while(!(UART0_IF & BIT2))
		{
			if(dealy > 0)
			{
				dealy--;
			}
			else
			{
				break;
			}
		}
		
        UART0_IF = BIT2;
        UART0_BUFF = pData[i];
    }
}

u8 CalcCRC(u8 *pData, int len)
{
    u8 byCRC = 0;
    for (int i = 0; i < len; i++)
    {
        byCRC ^= pData[i];
    }
    return byCRC;
}

u32 CharToU32(u8* pSrc)
{
	u32 v = *(u32*)pSrc;
	return v;
}

void U32ToChar(u32 v, u8 *pDst)
{
	memcpy(pDst, &v, sizeof(u32));
}
