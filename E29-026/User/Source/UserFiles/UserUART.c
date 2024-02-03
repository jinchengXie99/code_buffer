
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

u8  idata URXCnt;     // 调试用，接收计数
u8  idata UTXCnt;     // 调试用，发送计数

u8  idata UartFrameErr;

u8  idata UartHeadErr;
u8  idata UartSumErr;
u8  idata UartNoRxErr;
u8  idata UartTimeOutErr;
u8  idata UartErr;

u16 UartRxIdleX1ms;         // 通信空闲等待
u16 UartRxInvalidX1ms;      // 通信无效等待

u8  URXCmdVal;
u16 URXGoDuty;
u32 URXCmdSpd;

u16 URXCmdPwr;
u32 URXCmdRpm;

//----------------------------------------------------------------------------//
// 说明：UART数据初始化
void UARTDataInit(void)
{
  //  
  UartTxLine[0]  = 0x55;
  UartTxLine[1]  = 0xAA;
  UartTxLine[5]  = 0x55;
  
  UTXMark = 1;
}
//----------------------------------------------------------------------------//
// 说明：接收数据
void UARTGetData(void)
{
  UartRxData = UT_DR;                       // 读接收数据
  
  if(UartRxCnt == 0)                        // 帧首字节
  {
    if(UartRxData == 0x55)
    {
      UartRxLine[0]= 0x55;
      UartRxCnt++;
    }
  }
  else
  {
    
    UartRxLine[UartRxCnt] = UartRxData;   // 依次接收各数据
//    if(UartRxCnt < (URX_LENTH-1))
//      URXSum += UartRxData;               // 累加校验
    
    if(++UartRxCnt >= URX_LENTH)
    {
      URXSum = UartRxLine[2] + UartRxLine[3];

      // 判断帧，调试中可去除此判断
      if((UartRxLine[1] != 0xAA)||(UartRxLine[5] != 0x55)||(UartRxLine[4] != URXSum))
        UartFrameErr = 1;

      UartRxFlag = 1;                     // 收到6 byte，待处理
      UartRxCnt = 0;
      URXCnt++;
    }
    
  }
  
  UartRxIdleX1ms = 0;                       // 清除等待计数器
  //UartRxByteFlag = 1;                       // 接收到1byte
}
//----------------------------------------------------------------------------//
// 说明：发送数据
void UARTSetData(void)
{

  // 当前无发送任务
  if(UartTxFlag == 0)
  {
    if(UartTxTrig == 1)
    {
      UartTxTrig = 0;
      UartTxFlag = 1;
    }
  }
  // 当前有发送任务
  else//if(UartTxFlag == 1)
  {
    if(UartTxCnt < UTX_LENTH)
    {
      if(UTXMark == 1)                  // 发送中断已处理完成
      {
        UTXMark = 0;                    // 标志位清零
        UT_DR = UartTxLine[UartTxCnt];  // 发送当前字节
        UartTxCnt++;                    // 计数累计
      }
    }
    if(UartTxCnt >= UTX_LENTH)                  // 计数完成
    {
      UartTxFlag = 0;                   // 清除发送标志
      UartTxCnt = 0;                    // 清除计数器
      //UartTxFrameCnt++;
      UTXCnt++; 
    }
  }
}

void UARTPer1ms(void)
{
  
  UARTSetData();                        // 定时发送数据
  //UARTGetData();                      // 定时查收数据，此处1ms周期时长不足，置于中断处理
  
  // 通信空闲计时，仅对空闲时段计时，在接收到通信数据时清除
  if(++UartRxIdleX1ms >= 1000)
  {
    UartRxIdleX1ms = 1000;
  }

  // 空闲较长，接收字节计数清除
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
    
      if(UartRxLine[2] == 0xFF)           // 只有0xFF开机,其他一律关机
      {
        URXCmdVal = UartRxLine[3];
      }
      else // if(UartRxLine[2] == 0x00)      // 停机
      {
        URXCmdVal = 0;
      }
      
      if(mcState == mcReady)  
        UartTxLine[2]  = 0x00;            // 未运行
      else
        UartTxLine[2]  = 0xFF;            // 运行0
      
      if(URXCmdVal == 0)
        URXGoDuty = 0;
      else
        URXGoDuty = _Q12(1);
      
      // URXCmdVal: 如控制功率，即为功率值；如控制转速 需*30 对应转速。
      // 通信命令值 乘以300 表示实际预期值
      // 32767 对应 180000，则转换为  ((Cmd * 30)/ 180000) * 32767 
      // 等效运算 Cmd *109
      
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
