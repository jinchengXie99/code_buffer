
/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

#include <UserGlobal.h>
#include <UserDefine.h>

/* Private typedef ------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------*/

#define UTX_LENTH         6
#define URX_LENTH         6

extern u16 KeyGoDuty;
extern u8  idata AlarmFlag;

u8  UTXMark;
u8  URXMark;

u8  idata UartTxLine[UTX_LENTH];
u8  idata UartRxLine[URX_LENTH];
u8  idata URXSum;
u8  idata UTXSum;


u8  idata UartTxTrig;
u8  idata UartTxData;
u8  idata UartTxCnt;
u8  idata UartTxFlag;

u8  idata UartRxData;
u8  idata UartRxCnt;
u8  idata UartRxFlag;

u8  idata URXCnt;     // �����ã����ռ���
u8  idata UTXCnt;     // �����ã����ͼ���

u8  idata UartFrameErr;

u8  idata UartHeadErr;
u8  idata UartSumErr;
u8  idata UartNoRxErr;
u8  idata UartTimeOutErr;
u8  idata UartErr;

u16 UartRxIdleX1ms;         // ͨ�ſ��еȴ�
u16 UartRxInvalidX1ms;      // ͨ����Ч�ȴ�

u8  URXCmdVal;
u16 URXGoDuty;
u32 URXCmdSpd;

u16 URXCmdPwr;
u32 URXCmdRpm;

//----------------------------------------------------------------------------//
// ˵����UART���ݳ�ʼ��
void UARTDataInit(void)
{
  //  
  UartTxLine[0]  = 0x55;
  UartTxLine[1]  = 0xAA;
  UartTxLine[5]  = 0x55;
  
  UTXMark = 1;
}
//----------------------------------------------------------------------------//
// ˵������������
void UARTGetData(void)
{
  UartRxData = UT_DR;                       // ����������
  
  if(UartRxCnt == 0)                        // ֡���ֽ�
  {
    if(UartRxData == 0x55)
    {
      UartRxLine[0]= 0x55;
      UartRxCnt++;
    }
  }
  else
  {
    
    UartRxLine[UartRxCnt] = UartRxData;   // ���ν��ո�����
//    if(UartRxCnt < (URX_LENTH-1))
//      URXSum += UartRxData;               // �ۼ�У��
    
    if(++UartRxCnt >= URX_LENTH)
    {
      URXSum = UartRxLine[2] + UartRxLine[3];

      // �ж�֡�������п�ȥ�����ж�
      if((UartRxLine[1] != 0xAA)||(UartRxLine[5] != 0x55)||(UartRxLine[4] != URXSum))
        UartFrameErr = 1;

      UartRxFlag = 1;                     // �յ�6 byte��������
      UartRxCnt = 0;
      URXCnt++;
    }
    
  }
  
  UartRxIdleX1ms = 0;                       // ����ȴ�������
  //UartRxByteFlag = 1;                       // ���յ�1byte
}
//----------------------------------------------------------------------------//
// ˵������������
void UARTSetData(void)
{

  // ��ǰ�޷�������
  if(UartTxFlag == 0)
  {
    if(UartTxTrig == 1)
    {
      UartTxTrig = 0;
      UartTxFlag = 1;
    }
  }
  // ��ǰ�з�������
  else//if(UartTxFlag == 1)
  {
    if(UartTxCnt < UTX_LENTH)
    {
      if(UTXMark == 1)                  // �����ж��Ѵ������
      {
        UTXMark = 0;                    // ��־λ����
        UT_DR = UartTxLine[UartTxCnt];  // ���͵�ǰ�ֽ�
        UartTxCnt++;                    // �����ۼ�
      }
    }
    if(UartTxCnt >= UTX_LENTH)                  // �������
    {
      UartTxFlag = 0;                   // ������ͱ�־
      UartTxCnt = 0;                    // ���������
      //UartTxFrameCnt++;
      UTXCnt++; 
    }
  }
}

void UARTPer1ms(void)
{
  
  UARTSetData();                        // ��ʱ��������
  //UARTGetData();                      // ��ʱ�������ݣ��˴�1ms����ʱ�����㣬�����жϴ���
  
  // ͨ�ſ��м�ʱ�����Կ���ʱ�μ�ʱ���ڽ��յ�ͨ������ʱ���
  if(++UartRxIdleX1ms >= 1000)
  {
    UartRxIdleX1ms = 1000;
  }

  // ���нϳ��������ֽڼ������
  if(UartRxCnt > 0)
  {
    if(UartRxIdleX1ms >= 50)
    {
      UartRxCnt = 0;
    }
  }
  
  if(UartRxFlag == 1)
  {
    UartRxFlag = 0;

    if(UartFrameErr == 0)
    {
    
      if(UartRxLine[2] == 0xFF)           // ֻ��0xFF����,����һ�ɹػ�
      {
        URXCmdVal = UartRxLine[3];
      }
      else // if(UartRxLine[2] == 0x00)      // ͣ��
      {
        URXCmdVal = 0;
      }
      
      if(mcState == mcReady)  
        UartTxLine[2]  = 0x00;            // δ����
      else
        UartTxLine[2]  = 0xFF;            // ����0
      
      if(URXCmdVal == 0)
        URXGoDuty = 0;
      else
        URXGoDuty = _Q12(1);
      
      // URXCmdVal: ����ƹ��ʣ���Ϊ����ֵ�������ת�� ��*30 ��Ӧת�١�
      // ͨ������ֵ ����300 ��ʾʵ��Ԥ��ֵ
      // 32767 ��Ӧ 180000����ת��Ϊ  ((Cmd * 30)/ 180000) * 32767 
      // ��Ч���� Cmd *109
      
      //URXCmdPwr = URXCmdVal;
      URXCmdRpm = (u32)URXCmdVal*30;
      URXCmdSpd = (u32)URXCmdVal *11;
      
      if(URXCmdRpm < 1000)
      {
        URXGoDuty = 0;
        URXCmdSpd = 0;
      }
    }
    else
    {
      UartFrameErr = 0;
      URXGoDuty = 0;
      URXCmdSpd = 0;
    
      UartTxLine[2]  = 0x00;
    }
    
    //UartTxLine[3]  = 0x00;
    UartTxLine[3]  = AlarmFlag;
    UartTxLine[4]  = UartTxLine[2] + UartTxLine[3];

    UartTxTrig = 1;
  }
}
