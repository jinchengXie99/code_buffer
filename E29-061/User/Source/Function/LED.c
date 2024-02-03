/**************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
* File Name          : LED.c
* Author             : Billy Long Fortiortech  Market Dept
* Version            : V1.0
* Date               : 01/07/2015
* Description        : This file contains LPF control function used for Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/ 


/* Includes -------------------------------------------------------------------------------------*/
#include <FU68xx_2.h>
#include <Myproject.h>

uint16 KeyCount_Up;
uint16 KeyCount_Dd;
uint16 KeyCount_Dd1;
uint16 KeyCount_Long;
bit  LH_Gear = 0;
extern uint16  POWER_VSP;
/*-------------------------------------------------------------------------------------------------
  Function Name :  void LEDControl(void)
  Description   :  ���ܺ�����LED���ƣ���ʾϵͳ״̬
                  Stop״̬--LED����
                  Normal״̬--LED����
                  Fault״̬--LED��˸        
  Input         :  ��
  Output        :  ��
-------------------------------------------------------------------------------------------------*/
void LEDControl(void)
{
//  if((mcState != mcFault))
//  {
//     switch(KS.KeyValuetotal & 0x70)     //ת���ж�
//     {
//       case 0x10:
//         D4 = LEDON;
//         D5 = LEDOFF;
//         D6 = LEDOFF;
////         mcSpeedRamp.TargetValue = Motor_Speed_Low;             // ת�ٸ�ֵ
//         POWER_VSP = POWER_LOW;
//         break;
//       case 0x20:
//         D4 = LEDON;
//         D5 = LEDON;
//         D6 = LEDOFF;  
//         POWER_VSP = POWER_gear;
////         mcSpeedRamp.TargetValue = Motor_Speed_Mid;            // ת�ٸ�ֵ     
//         break;
//       case 0x40:
//         D4 = LEDON;
//         D5 = LEDON;
//         D6 = LEDON;
//         POWER_VSP =POWER_HIGH;
////         mcSpeedRamp.TargetValue = Motor_Speed_HIgh;          // ת�ٸ�ֵ       
//         break;
//       default:
//         break;
//    }
//    
//   if((KS.KeyValuetotal & 0x80) == 0)                         //�¶ȸ�ֵ
//   { 
//    switch(KS.KeyValuetotal & 0x0F)
//     {
//       case 0x01:
//         D1 = LEDOFF;
//         D2 = LEDOFF;
//         D3 = LEDOFF;
//         mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_Off][1];
//         mcFocCtrl.HW_ZeroT = HW_Gear[Temp_Off][0];
//         HW1 = HWOFF;
//         HW2 = HWOFF;
//         break;
//       case 0x02:
//         D1 = LEDOFF;
//         D2 = LEDOFF;
//         D3 = LEDON;   
//         mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_LOW][1];
//         mcFocCtrl.HW_ZeroT = HW_Gear[Temp_LOW][0];
//         break;
//       case 0x04:
//         D1 = LEDOFF;
//         D2 = LEDON;
//         D3 = LEDON;
//         mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_Mid][1];
//         mcFocCtrl.HW_ZeroT = HW_Gear[Temp_Mid][0];
//         break;
//       case 0x08:
//         D1 = LEDON;
//         D2 = LEDON;
//         D3 = LEDON; 
//         mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_High][1];
//         mcFocCtrl.HW_ZeroT = HW_Gear[Temp_High][0];
//         break;
//       default:
//         break;
//     }  
//   }
//   else                                                        //һ����簴��
//   {
//     mcFocCtrl.HW_ZeroZ = HW_Gear[Temp_Off][1];
//     mcFocCtrl.HW_ZeroT = HW_Gear[Temp_Off][0];
//     D1 = LEDOFF;
//     D2 = LEDOFF;
//     D3 = LEDOFF;
//     HW1 = HWOFF;
//     HW2 = HWOFF;
//   }
// }
 
//   mcFocCtrl.HW_ZeroZ = Temp_HighZ;
//   mcFocCtrl.HW_ZeroT = Temp_HighT;
// 
//   mcFocCtrl.HW_interval = (uint8)(Temp_HighZ/Temp_HighT);
 
//  mcFocCtrl.HW_ZeroZ = Temp_HighZ;
//  mcFocCtrl.HW_ZeroT = Temp_HighT;
// 
//  mcFocCtrl.HW_Gcd =  Get_Gcd(mcFocCtrl.HW_ZeroT, mcFocCtrl.HW_ZeroZ);
//  mcFocCtrl.HW_ZeroZ = mcFocCtrl.HW_ZeroZ / mcFocCtrl.HW_Gcd;
//  mcFocCtrl.HW_ZeroT = mcFocCtrl.HW_ZeroT / mcFocCtrl.HW_Gcd;
}

/*-------------------------------------------------------------------------------------------------
  Function Name :  void LEDControl(void)
  Description   :  ���ܺ���,���ƿ���
                    
  Input         :  ��
  Output        :  ��
-------------------------------------------------------------------------------------------------*/
void Motor_EN(void)
{
   if(GP10==0) 
    {
      KeyCount_Up = 0;
      if(KeyCount_Dd < 50)
      {
        KeyCount_Dd++;
      }
      else
      {        
        LH_Gear = 1;
      }
    }    
    else 
    {
      KeyCount_Dd = 0;
      if(KeyCount_Up < 50)  
      {
        KeyCount_Up++;  
      }
      else
      {
        if(LH_Gear == 1)
        {
          LH_Gear = 0;
          if(POWER_VSP==POWER_LOW)  
          {
           POWER_VSP = POWER_gear;
          }
          else
          {
            POWER_VSP = POWER_LOW;
          }
    
        }  
      }
    }
}
/*-------------------------------------------------------------------------------------------------
  Function Name :  void LEDControl(void)
  Description   :  ���ܺ���,���ƿ���
                    
  Input         :  ��
  Output        :  ��
-------------------------------------------------------------------------------------------------*/
void KEYControl(void)
{
//  if(LOCK==1)                  //δ����
//  {
    if(KS.KeyLongPressFlag==0)
       {
        if(mcState == mcRun)
        {
          KeyScan();              //��ȡ����ֵ
        }                                                               
        if(KS.ChangeKeyFlg)      //
        {
          LEDControl();
          KS.ChangeKeyFlg = 0;
        }
      }
//  }
}
