
/************************ (C) COPYRIGHT 2015 FT *******************************
* File Name          : 
* Author             : Application Team  Tom.wang 
* Version            : V2.0.0
* Date               : 06/15/2015
* Description        : 
********************************************************************************
/*******************************************************************************
* All Rights Reserved
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>
#include <Customer_Debug.h>
ROM_TypeDef xdata  Rom;

uint8 idata FLASHUsingStatus=0;
void Write2Byte2Flash(uint8 xdata *BlockStartAddr,uint16 NewData2Flash)
{
  uint8 xdata *FlashStartAddr = BlockStartAddr;
  uint16 tempofFlashData=0;
  uint16 tempofNewFlashData=0;
  uint8 i;
  
  if(FLASHUsingStatus == 0)
  {
    FLASHUsingStatus=1;      //FLASH����ʹ��
    tempofNewFlashData = NewData2Flash;
    for(i=0;i<64;i++)
    {
      tempofFlashData = *(uint16 code *)(FlashStartAddr+2*i);
      if(tempofFlashData==0)
      {
          tempofFlashData = tempofNewFlashData>>8;
          Flash_Sector_Write((FlashStartAddr+2*i),(uint8)tempofFlashData);
          _nop_();
          tempofFlashData = tempofNewFlashData&0x00ff;
          Flash_Sector_Write((FlashStartAddr+2*i+1),(uint8)tempofFlashData);
          _nop_();      
          break;
      }
      else
      {
          if(i==63)
          {
              Flash_Sector_Erase(FlashStartAddr);
              _nop_();
              tempofFlashData = tempofNewFlashData>>8;
              Flash_Sector_Write(FlashStartAddr,(uint8)tempofFlashData);
              _nop_();
              tempofFlashData = tempofNewFlashData&0x00ff;
              Flash_Sector_Write((FlashStartAddr+1),(uint8)tempofFlashData);
              _nop_();        
          }
      }
    }
    FLASHUsingStatus=0;      //FLASH����
  }
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  void Write1Byte2Flash(uint8 xdata *BlockStartAddr,uint8 NewData2Flash)
  Description   :  д��1���ֽڵ�FLASH
  Input         :  uint8 xdata *BlockStartAddr��Ŀ��FLASH��ַ  NewData2Flash����д������
  Output        :  ��
-------------------------------------------------------------------------------------------------*/
void Write1Byte2Flash(uint8 xdata *BlockStartAddr,uint8 NewData2Flash)
{
  uint8 xdata *FlashStartAddr = BlockStartAddr;
  uint8 tempofNewFlashData;
  uint8 tempofFlashData;
  uint8 i;
  
  if(FLASHUsingStatus == 0)
  {
    FLASHUsingStatus=1;      //FLASH����ʹ��
    tempofNewFlashData = NewData2Flash;
    for(i=0;i<128;i++)
    {
      tempofFlashData = *(uint8 code *)(FlashStartAddr + i);
      if(tempofFlashData==0)
      {
        Flash_Sector_Write((FlashStartAddr+i),(uint8)tempofNewFlashData);
        _nop_();    
        break;
      }
      else
      {
        if(i==127)
        {
          Flash_Sector_Erase(FlashStartAddr);
          _nop_();
          Flash_Sector_Write(FlashStartAddr,(uint8)tempofNewFlashData);
          _nop_();    
        }
      }
    }
    FLASHUsingStatus=0;      //FLASH����
  }
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  uint8 Get1ByteFromFlash(uint8 xdata *BlockStartAddr)
  Description   :  ��Ŀ��FLASH������ȡ1�ֽ�����д�������
  Input         :  uint8 xdata *BlockStartAddr��Ŀ��FLASH����
  Output        :  ����������
-------------------------------------------------------------------------------------------------*/
uint8 Get1ByteFromFlash(uint8 xdata *BlockStartAddr)
{
  uint8 xdata *FlashStartAddr = BlockStartAddr;
  uint8 i;
  uint8 tempofFlashData;
  
  for(i=0;i<128;i++)
  {
    tempofFlashData = *(uint8 code *)(FlashStartAddr + i);
    if(tempofFlashData==0)
    {
      if(i!=0)
      {
        tempofFlashData = *(uint8 code *)(FlashStartAddr + (i-1));
        return tempofFlashData;
      }
      else
      {
        return 0;
      }
    }
    else
    {
      if(i==127)
      {
        return tempofFlashData;
      }
    }  
  }
  return 0;
}
