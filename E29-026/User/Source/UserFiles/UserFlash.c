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
// ˵�����û�����flash
// ����ʱ��ȡ�浵���������ʱ�ĵ�λָ���4λ�Ƿ��ٵ�����4λ���ȶȵ�
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
// ˵�����û�������д��洢��
void FlashWriteData(void)
{
  // ÿ4byteΪһ�飬���е�һ��byteд��0xA5�����ĸ�byteд�뱾�θߵ͵����ڶ���byteԤ��
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
// ˵���������û�����������д��ǰ2���洢��
void FlashEarseAndWrite(void)
{
  // �����˴洢����
  Flash_Sector_Erase(USER_FALSH_ADDRESS);
  
  // ����ַ���ñ�־0xA5��������д������
  Flash_Sector_Write(USER_FALSH_ADDRESS+0,(uint8)0xA5 );
  Flash_Sector_Write(USER_FALSH_ADDRESS+1,(uint8)0x5A );

  // ����ַƫ��16λ����¼��������������Ϊ16λ����
  Flash_Sector_Write(USER_FALSH_ADDRESS+2,(uint8)FlashWriteTimes);
  Flash_Sector_Write(USER_FALSH_ADDRESS+3,(uint8)(FlashWriteTimes>>8));

  // ÿ4byteΪһ�飬���е�һ��byteд��0xA5���ڶ����ĸ�byteд�뱾������
  Flash_Sector_Write(USER_FALSH_ADDRESS+4,(uint8)0xA5 );
  Flash_Sector_Write(USER_FALSH_ADDRESS+5,(uint8)SpeedShiftStep);
  Flash_Sector_Write(USER_FALSH_ADDRESS+6,(uint8)HeatShiftStep);
  Flash_Sector_Write(USER_FALSH_ADDRESS+7,(uint8)CoolShiftLv);

//    //bool TempEA;
//    //TempEA = EA;
//    EA = 0;

//    FLA_CR = 0x03;                  //ʹ���Բ���
//    FLA_KEY = 0x5a;
//    FLA_KEY = 0x1f;                 //flashԤ��̽���
//    _nop_();
//    
//    //д����
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
//    FLA_CR = 0x08;                  //��ʼԤ��̣���ɺ�Flash�ٴ�����
//    //EA = TempEA;

}
//----------------------------------------------------------------------------//
// ˵����Flash�����ϵ缴ִ��һ�Σ���ÿ1msִ��
void DryerFlashInit(void)
{
  u8 i;

  EA = 0;
      
  // �ϵ���ȡFlash
  if(FlashInitFlag == 0)
  {
    FlashInitFlag = 1;
    
    // ��ȡǰ4byte
    for(i=0;i<4;i++)
      FlashReadLine[i] = *(uint8 code *)(USER_FALSH_ADDRESS + i);
    
    // ���һ���ֽڷ�0xA5,����δ���ù�����
    if(FlashReadLine[0] != 0xA5)
    {
      FlashWriteTimes = 1;                // �״β�д
      
      DryerKeysDefault();                 // 
      
      FlashEarseAndWrite();               // ��д

      FlashPieceNum = 2;                  // flashԤ�������
      FlashAddrOffset = 8;                // flashԤ����ƫ���׵�ַ
    }
    // ����,���������ù�������
    else
    {
      // �������һ���Ѵ洢���ݵ�����
      FlashPieceNum = 1;                  // 
      for(i=1;i<=30;i++)
      {
        if((*(uint8 code *)(USER_FALSH_ADDRESS + i*4)) == 0xA5)
          FlashPieceNum=i;
      }
      
      // �����ϴδ洢���׵�ַƫ����
      FlashAddrOffset = FlashPieceNum*4;
      // ��ȡ��д��������һ��
      FlashWriteTimes = FlashReadLine[2] + (FlashReadLine[3]<<8);
      
      // ��ȡ������
      SpeedShiftStep = *(uint8 code *)(USER_FALSH_ADDRESS + FlashAddrOffset + 1);
      HeatShiftStep = *(uint8 code *)(USER_FALSH_ADDRESS + FlashAddrOffset + 2);
      CoolShiftLv = *(uint8 code *)(USER_FALSH_ADDRESS + FlashAddrOffset + 3);
      
      // ��������д����Ϊ�����´δ洢��Ӧ���������������ʼ��һ��flash����������¼��д����
      if(FlashPieceNum >= 30)        // �˴��� >=
      {
        FlashWriteTimes += 1;       // ��д��������
        
        FlashEarseAndWrite();       // ��д
        
        FlashPieceNum = 2;          // ���Ĵ�д������Ĵ洢�����
        FlashAddrOffset = 8;        // ���Ĵ�д������Ĵ洢��ƫ���׵�ַ
      }
      // ����δд�����������������ı�ż��׵�ַ�������´�д��
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
// ˵����Flash����ÿ1msִ��
void FlashLoopPro(void)
{
  // 1 �������쳣����������
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

  // 2 �洢��־��λ��ִ��flash�洢
  if(FlashSaveFlag == 1)
  {
    FlashSaveFlag = 2;
    if(TestMode == 0)
    {
//    FlashWriteData();
    EA = 0;
    if(FlashPieceNum > 30)           // �˴��� >=
    {
      FlashWriteTimes += 1;         // ��д��������
      
      FlashEarseAndWrite();         // ��д
      
      FlashPieceNum = 2;            // ���Ĵ�д������Ĵ洢�����
      FlashAddrOffset = 8;          // ���Ĵ�д������Ĵ洢��ƫ���׵�ַ
    }
    // ����δд�����������������ı�ż��׵�ַ�������´�д��
    else
    {
      FlashWriteData();
      
      FlashPieceNum += 1;
      FlashAddrOffset += 4;
    }
    EA = 1;
    }
  }
  
  // ������ʱ���ݴ洢��ϣ���δ��ȫ�ر�ʱ�����ָֻ�����ʱ������ִ��Flash��ʼ��
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
