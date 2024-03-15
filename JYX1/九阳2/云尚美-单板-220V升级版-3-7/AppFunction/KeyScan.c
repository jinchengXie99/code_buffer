/*******************************************************************************
 * 版权所有 (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： KeyScan.c
 * 文件标识：
 * 内容摘要： KeyScan
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： andrew kong
 * 完成日期： 2019年12月19日
 *
 *******************************************************************************/
#include "KeyScan.h"

KeyControl_t key_state;

/*****************************************************************************
 * Function:     void KeyScan(void)
 * Description:  key value  stop or run
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void KeyScan(KeyControl_t *pKeyState)
{
  if(KEY0)
  {
    pKeyState->nKey0PressCnt ++;
    if(pKeyState->nKey0PressCnt >= KEY_FLITER)
    {
      pKeyState->nKey0PressCnt = KEY_FLITER;
      if(pKeyState->nFlagKey0Press == 1)
      {
        pKeyState->nFlagKey0Press = 0;
        if(pKeyState->nKey0Value == START)
        {
          pKeyState->nKey0Value = STOP;
        }
        else
        {
          pKeyState->nKey0Value = START;
        }
      }
    }
  }
  else
  {
    pKeyState->nKey0PressCnt --;
    if(pKeyState->nKey0PressCnt <= -KEY_FLITER)
    {
      pKeyState->nKey0PressCnt = -KEY_FLITER;
      pKeyState->nFlagKey0Press = 1;
    }
  }
}

/*****************************************************************************
 * Function:     void KeyInit(void)
 * Description:  init
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void KeyInit(void)
{
  key_state.nFlagKey0Press = 0;
  key_state.nKey0PressCnt  = 0;
  key_state.nKey0Value     = STOP;
  LED_OFF;
}
