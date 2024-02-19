#ifndef COMDATA_H_
#define COMDATA_H_

#include "basic.h"


#define MAXRECV_LEN   255
#define MINRECV_LEN   4

#define MAXSEND_LEN   255
#define PACKHEAD_LEN  3

// ����ͷ����
#define DATA_HEAD0      0x68

#define INDEX_HEAD      0
#define INDEX_LEN       1
#define INDEX_CRC       2
#define INDEX_CMD       3

// CMD���� READ_WRITE
// ����ֵ�У����CMD�ĸ�λΪ1����ʾ����
// ����PRINTF�����ݣ�Ҳ��ͨ��READMEM��ȡ��
#define CMD_READMEM     0x01
#define CMD_WRITEMEM    0x02

// ��ͬ��������ֽڲ�ͬ
// CMD_READMEM�������: ��ʼ��ַ+����
// ����֡
#define INDEX_ADDR      4           // 4�ֽڵ�ַ���±�
#define INDEX_RLEN      8           // �����ݳ��ȵ��±�

// CMD_WRITEMEM
#define INDEX_WDATA     8            // д���ݵ��±�

// Ӧ��֡
#define INDEX_MEMDATA   4

// ��ʼ���������ݻ�����
void InitRecvData(void);

// ����������ݣ���ӻ�����������
void AddRecv(u8 data);
void CheckRecv(void);

// �������������ݰ�
void HandleCmd(void);
void HandleError(void);
void HandleReadMem(void);
void HandleWriteMem(void);

// ������֯�õı������ݡ����޸�ͨ���жϻ�DMAʵ��
void SendData(u8 *pData, u8 len);

// Э�鴦�����ຯ��
u8 CalcCRC(u8 *pData, int nLen);
u32 CharToU32(u8* pSrc);
void U32ToChar(u32 v, u8 *pDst);

#endif
