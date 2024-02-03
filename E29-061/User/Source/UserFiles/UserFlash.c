/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
* File Name          : Main.c
* Author             : Andrew Kong Fortiortech  Market Dept
* Version            : V1.0
* Date               : 11/09/2015
* Description        : This file contains all the common data types used for
*                      Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/

/*Include ---------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>

#include <UserGlobal.h>
#include <UserDefine.h>

#define  USER_FALSH_ADDRESS 0X3E00             // 15616

extern uint8 Flash_Sector_Erase(uint8 xdata *FlashAddress);
extern uint8 Flash_Sector_Write(uint8 xdata *FlashAddress, uint8 FlashData);

extern u8  PowerLose;

// UserDryer.c
extern u8  idata SpeedShiftStep;
extern u8  idata SpeedCmdLv;

extern u8  idata HeatShiftStep;
extern u8  idata HeatCmdLv;

extern u8  idata CoolCmdLv;
extern u8  idata CoolShiftLv;

extern bit TestMode;

//----------------------------------------------------------------------------//
// 说明：用户操作flash
// 启动时读取存档，获得启动时的挡位指令，低4位是风速挡，高4位是热度挡
//u8  FaultCode=0;
//u8  FaultLine[8];
u8  FaultOffset;
u8  FlashInitFlag;

u8  FlashReadLine[4];

u8  FlashPieceNum;
u8  FlashAddrOffset;
u8  FlashSaveFlag;
u16 FlashWriteTimes;

//----------------------------------------------------------------------------//
// 说明：用户数据组写入存储区
void FlashWriteData(void)
{
  // 每4byte为一组，其中第一个byte写入0xA5，第四个byte写入本次高低档，第二三byte预留
  Flash_Sector_Write(USER_FALSH_ADDRESS+FlashAddrOffset+0,(uint8)0xA5);
  _nop_();
  Flash_Sector_Write(USER_FALSH_ADDRESS+FlashAddrOffset+1,(uint8)SpeedShiftStep);
  _nop_();
  Flash_Sector_Write(USER_FALSH_ADDRESS+FlashAddrOffset+2,(uint8)HeatShiftStep);
  _nop_();
  Flash_Sector_Write(USER_FALSH_ADDRESS+FlashAddrOffset+3,(uint8)CoolShiftLv);
  _nop_();
}

//----------------------------------------------------------------------------//
// 说明：擦除用户数据区，并写入前2个存储组
void FlashEarseAndWrite(void)
{
  // 擦除此存储区间
  Flash_Sector_Erase(USER_FALSH_ADDRESS);
  
  // 基地址设置标志0xA5，表明已写入数据
  Flash_Sector_Write(USER_FALSH_ADDRESS+0,(uint8)0xA5 );
  Flash_Sector_Write(USER_FALSH_ADDRESS+1,(uint8)0x5A );

  // 基地址偏移16位，记录擦除次数，次数为16位数据
  Flash_Sector_Write(USER_FALSH_ADDRESS+2,(uint8)FlashWriteTimes);
  Flash_Sector_Write(USER_FALSH_ADDRESS+3,(uint8)(FlashWriteTimes>>8));

  // 每4byte为一组，其中第一个byte写入0xA5，第二三四个byte写入本次数据
  Flash_Sector_Write(USER_FALSH_ADDRESS+4,(uint8)0xA5 );
  Flash_Sector_Write(USER_FALSH_ADDRESS+5,(uint8)SpeedShiftStep);
  Flash_Sector_Write(USER_FALSH_ADDRESS+6,(uint8)HeatShiftStep);
  Flash_Sector_Write(USER_FALSH_ADDRESS+7,(uint8)CoolShiftLv);

//    //bool TempEA;
//    //TempEA = EA;
//    EA = 0;

//    FLA_CR = 0x03;                  //使能自擦除
//    FLA_KEY = 0x5a;
//    FLA_KEY = 0x1f;                 //flash预编程解锁
//    _nop_();
//    
//    //写数据
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+0) = 0xA5;
//    _nop_();
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+1) = 0x5A;
//    _nop_();
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+2) = (uint8)FlashWriteTimes;
//    _nop_();
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+3) = (uint8)(FlashWriteTimes>>8);
//    _nop_();
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+4) = 0xA5;
//    _nop_();
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+5) = (uint8)SpeedShiftStep;
//    _nop_();
//    *(uint8 xdata *)(USER_FALSH_ADDRESS+6) = (uint8)HeatShiftStep;
//    _nop_();
////    *(uint8 xdata *)(USER_FALSH_ADDRESS+7) = (uint8)CoolShiftLv);
////    _nop_();
//    
//    FLA_CR = 0x08;                  //开始预编程，完成后Flash再次上锁
//    //EA = TempEA;

}
//----------------------------------------------------------------------------//
// 说明：Flash处理，上电即执行一次，后每1ms执行
void DryerFlashInit(void)
{
  u8 i;

  EA = 0;
      
  // 上电后读取Flash
  if(FlashInitFlag == 0)
  {
    FlashInitFlag = 1;
    
    // 读取前4byte
    for(i=0;i<4;i++)
      FlashReadLine[i] = *(uint8 code *)(USER_FALSH_ADDRESS + i);
    
    // 如第一个字节非0xA5,表明未设置过参数
    if(FlashReadLine[0] != 0xA5)
    {
      FlashWriteTimes = 1;                // 首次擦写
      
      DryerKeysDefault();                 // 
      
      FlashEarseAndWrite();               // 擦写

      FlashPieceNum = 2;                  // flash预备区编号
      FlashAddrOffset = 8;                // flash预备区偏移首地址
    }
    // 否则,表明已设置过参数，
    else
    {
      // 搜索最后一个已存储数据的数组
      FlashPieceNum = 1;                  // 
      for(i=1;i<=30;i++)
      {
        if((*(uint8 code *)(USER_FALSH_ADDRESS + i*4)) == 0xA5)
          FlashPieceNum=i;
      }
      
      // 计算上次存储组首地址偏移量
      FlashAddrOffset = FlashPieceNum*4;
      // 读取擦写次数并加一次
      FlashWriteTimes = FlashReadLine[2] + (FlashReadLine[3]<<8);
      
      // 读取开机挡
      SpeedShiftStep = *(uint8 code *)(USER_FALSH_ADDRESS + FlashAddrOffset + 1);
      HeatShiftStep = *(uint8 code *)(USER_FALSH_ADDRESS + FlashAddrOffset + 2);
      CoolShiftLv = *(uint8 code *)(USER_FALSH_ADDRESS + FlashAddrOffset + 3);
      
      // 若此区块写满，为便于下次存储，应将此区域擦除，开始新一轮flash操作，并记录擦写次数
      if(FlashPieceNum >= 30)        // 此处是 >=
      {
        FlashWriteTimes += 1;       // 擦写次数递增
        
        FlashEarseAndWrite();       // 擦写
        
        FlashPieceNum = 2;          // 更改待写入数组的存储区编号
        FlashAddrOffset = 8;        // 更改待写入数组的存储区偏移首地址
      }
      // 区块未写满，计算待操作区块的编号及首地址，用以下次写入
      else
      {
        FlashPieceNum += 1;
        FlashAddrOffset += 4;
      }
    }
  }
  
  //EA = 1;
}

//----------------------------------------------------------------------------//
// 说明：Flash处理，每1ms执行
void FlashLoopPro(void)
{
  // 1 当供电异常，保存数据
  if(PowerLose == 1)
  {
    if(mcState == mcReady)
    {
      if(FlashSaveFlag == 0)
      {
        FlashSaveFlag = 1;
      }
    }
  }

  // 2 存储标志置位，执行flash存储
  if(FlashSaveFlag == 1)
  {
    FlashSaveFlag = 2;
    if(TestMode == 0)
    {
//    FlashWriteData();
    EA = 0;
    if(FlashPieceNum > 30)           // 此处是 >=
    {
      FlashWriteTimes += 1;         // 擦写次数递增
      
      FlashEarseAndWrite();         // 擦写
      
      FlashPieceNum = 2;            // 更改待写入数组的存储区编号
      FlashAddrOffset = 8;          // 更改待写入数组的存储区偏移首地址
    }
    // 区块未写满，计算待操作区块的编号及首地址，用以下次写入
    else
    {
      FlashWriteData();
      
      FlashPieceNum += 1;
      FlashAddrOffset += 4;
    }
    EA = 1;
    }
  }
  
  // 若掉电时数据存储完毕，但未完全关闭时供电又恢复，此时需重新执行Flash初始化
  if(FlashSaveFlag == 2)
  {
    if(PowerLose == 0)
    {
      FlashSaveFlag = 0;
      if(TestMode == 1)
      {
        SpeedShiftStep = 1;
        SpeedCmdLv = 1;
        
        HeatShiftStep = 1;
        HeatCmdLv = 1;
        
        CoolShiftLv = 0;
        CoolCmdLv = 0;
      }
    }
  }
}
