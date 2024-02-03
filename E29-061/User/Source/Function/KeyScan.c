/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
* File Name          : KeyScan.c
* Author             : Billy Long Fortiortech  Market Dept
* Version            : V1.0
* Date               : 01/07/2015
* Description        : This file contains key scan function used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/ 


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>

/* Private variables ----------------------------------------------------------------------------*/
KeyScanParam_TypeDef  KS;

/*-------------------------------------------------------------------------------------------------
  Function Name :  void KeyInit(void)
  Description   :  按键参数初始化
  Input         :  无
  Output        :  无
-------------------------------------------------------------------------------------------------*/
void KeyInit(void)
{
  memset(&KS,0, sizeof(KeyScanParam_TypeDef));    
  
  Rom.WriteValue = 0;
  Rom.ReadValue = Get1ByteFromFlash(STARTPAGEROMADDRESS);
  
  if((Rom.ReadValue == 0x11)||(Rom.ReadValue == 0x21) ||(Rom.ReadValue == 0x41) ||(Rom.ReadValue == 0x12) ||(Rom.ReadValue == 0x22)
  ||(Rom.ReadValue == 0x42)||(Rom.ReadValue == 0x14)||(Rom.ReadValue== 0x24)||(Rom.ReadValue == 0x44)||(Rom.ReadValue == 0x18)
  ||(Rom.ReadValue == 0x28)||(Rom.ReadValue == 0x48))
  {
    KS.KeyValuetotal   = Rom.ReadValue;
    KS.Key1Value = Rom.ReadValue & 0x0F;
    KS.Key2Value = Rom.ReadValue & 0x70;;
    KS.Key3Value = 0x00;                //上电读取内存按键值
  }
  else                                  
  {
    KS.Key1Value = 0x01;
    KS.Key2Value = 0x10;
    KS.Key3Value = 0x00;
    KS.KeyValuetotal    = KS.Key1Value ^ KS.Key2Value ^ KS.Key3Value; //读值不对给固定档位
    KS.OldKeyValuetotal = KS.KeyValuetotal;
  }
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  uint8 KeyValue(void)
  Description   :  功能函数，获取按键值，选择返回键位值还是管脚电平KeyValue
  Input         :  无
  Output        :  键位值或者管脚电平KeyValue
-------------------------------------------------------------------------------------------------*/
int KeyValue(void)
{
  /* SW1 Scan */
//  if(SW1==1)
//  {
//    if(KS.SW1_Flag==1)
//    {
//      KS.Key1PressCnt ++;
//      if(KS.Key1PressCnt >= KeyFilterTime)
//      {
//        KS.Key1PressCnt = KeyFilterTime;
//        KS.SW1_Flag=0;    
//        
//        if((KS.Key1Value < 0x08)&&(KS.Key1Valueflag == 0))
//        {
//           KS.Key1Value = KS.Key1Value <<1;
//        }
//        else if((KS.Key1Value > 0x01)&&(KS.Key1Valueflag==1))
//        {
//           KS.Key1Value = KS.Key1Value >>1;
//        }
//        else if(KS.Key1Value == 0x08)
//        {
//          KS.Key1Valueflag= 1;
//          KS.Key1Value = KS.Key1Value >>1;
//        }
//        else if(KS.Key1Value == 0x01)
//        {
//          KS.Key1Valueflag= 0;
//          KS.Key1Value = KS.Key1Value <<1;
//        }
//        else
//        {
//          KS.Key1Valueflag= 0;
//          KS.Key1Value = 0x01;
//        }    
//      }
//    }
//  }
//  else
//  {    
//     KS.Key1PressCnt --;
//     if(KS.Key1PressCnt <= 0)
//     {
//       KS.Key1PressCnt = 0;
//       KS.SW1_Flag=1;
//     }
//  }    

  /* SW4 Scan */
  if(SW2==1)
  {
    if(KS.SW2_Flag==1)
    {
      KS.Key2PressCnt ++;
      if(KS.Key2PressCnt >= KeyFilterTime)
      {
        KS.Key2PressCnt = KeyFilterTime;
        KS.SW2_Flag=0;
        
        if((KS.Key2Value < 0x40)&&(KS.Key2Valueflag == 0))
        {
           KS.Key2Value = KS.Key2Value <<1;
        }
        else if((KS.Key2Value > 0x10)&&(KS.Key2Valueflag == 1))
        {
           KS.Key2Value = KS.Key2Value >>1;
        }
        else if(KS.Key2Value == 0x40)
        {
          KS.Key2Valueflag= 1;
          KS.Key2Value = KS.Key2Value >>1;
        }
        else if(KS.Key2Value == 0x10)
        {
          KS.Key2Valueflag = 0;
          KS.Key2Value = KS.Key2Value <<1;
        }
        else
        {
          KS.Key2Valueflag = 0;
          KS.Key2Value = 0x20;
        }            
      }
    }
  }
  else
  {    
     KS.Key2PressCnt --;
     if(KS.Key2PressCnt <= 0)
     {
       KS.Key2PressCnt = 0;
       KS.SW2_Flag=1;
     }
  }  

  /* SW3 Scan */
  if(SW3==1)
  {
    KS.Key3PressCnt++;
    if(KS.Key3PressCnt >= KeyFilterTime)
    {
      KS.Key3PressCnt = KeyFilterTime;
      if(KS.SW3_Flag==1)
      {
        KS.SW3_Flag=0;
        if(KS.Key3Value == 0x80)
        {
           KS.Key3Value = 0x00; 
        }
        else
        {
          KS.Key3Value = 0x80;
        }          
      }
    }
  }
  else
  {    
     KS.Key3PressCnt --;
     if(KS.Key3PressCnt <= -KeyFilterTime)
     {
       KS.SW3_Flag=1;
     }
  }
  return (KS.Key1Value ^ KS.Key2Value ^ KS.Key3Value);  
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  void KeyScan(void)
  Description   :  功能函数，按键扫描，按键触发传递出按键命令
  Input         :  无
  Output        :  无
-------------------------------------------------------------------------------------------------*/
void KeyScan(void)
{
  KS.OldKeyValuetotal = KS.KeyValuetotal;
  
  KS.KeyValuetotal = KeyValue();
  
  if(KS.OldKeyValuetotal != KS.KeyValuetotal)
  {
    KS.ChangeKeyFlg = 1;
  }
}


