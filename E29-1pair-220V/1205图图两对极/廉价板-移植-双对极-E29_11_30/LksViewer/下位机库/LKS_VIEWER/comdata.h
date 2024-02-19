#ifndef COMDATA_H_
#define COMDATA_H_

#include "basic.h"


#define MAXRECV_LEN   255
#define MINRECV_LEN   4

#define MAXSEND_LEN   255
#define PACKHEAD_LEN  3

// 报文头定义
#define DATA_HEAD0      0x68

#define INDEX_HEAD      0
#define INDEX_LEN       1
#define INDEX_CRC       2
#define INDEX_CMD       3

// CMD类型 READ_WRITE
// 返回值中，如果CMD的高位为1，表示错误
// 对于PRINTF的内容，也是通过READMEM读取的
#define CMD_READMEM     0x01
#define CMD_WRITEMEM    0x02

// 不同命令后续字节不同
// CMD_READMEM后的数据: 起始地址+长度
// 请求帧
#define INDEX_ADDR      4           // 4字节地址的下标
#define INDEX_RLEN      8           // 读数据长度的下标

// CMD_WRITEMEM
#define INDEX_WDATA     8            // 写数据的下标

// 应答帧
#define INDEX_MEMDATA   4

// 初始化接收数据缓冲区
void InitRecvData(void);

// 处理接收数据，添加缓冲区并解析
void AddRecv(u8 data);
void CheckRecv(void);

// 处理完整的数据包
void HandleCmd(void);
void HandleError(void);
void HandleReadMem(void);
void HandleWriteMem(void);

// 发送组织好的报文内容。可修改通过中断或DMA实现
void SendData(u8 *pData, u8 len);

// 协议处理工具类函数
u8 CalcCRC(u8 *pData, int nLen);
u32 CharToU32(u8* pSrc);
void U32ToChar(u32 v, u8 *pDst);

#endif
