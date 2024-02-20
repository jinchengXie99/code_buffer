/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_IWDG.c
 * 文件标识：
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/
#include "lks32mc03x_iwdg.h"
#include "lks32mc03x.h"
#include "basic.h"

/*******************************************************************************
 函数名称：    void IWDG_Init(IWDG_InitTypeDef *this)
 功能描述：    看门狗初始化
 输入参数：    this 看门狗配置结构体
 返 回 值：    无
 其它说明：    休眠唤醒时间需要小于复位时间，否则休眠唤醒不生效
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
void IWDG_Init(IWDG_InitTypeDef *this)
{
    IWDG->CFG = (this->DWK_EN<<4) | (this ->WDG_EN);
    IWDG->PSW = PSW_IWDG_PRE;
    IWDG->RTH = this->RTH;
    if(this->RTH > this->WTH)
    {
        IWDG->WTH = this->RTH-this->WTH;
    }
    else
    {
        IWDG->WTH = 0x001000;
    }
    IWDG->PSW = PSW_IWDG_PRE;
    IWDG->CLR = PSW_IWDG_CLR;
}
/*******************************************************************************
 函数名称：    void IWDG_StrutInit(IWDG_InitTypeDef *this)
 功能描述：    看门狗配置结构体初始化
 输入参数：    this看门狗配置结构体指针
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/09    V1.0           Yangzj              创建
 *******************************************************************************/
void IWDG_StrutInit(IWDG_InitTypeDef *this)
{
    this->DWK_EN = DISABLE;             // 深度休眠定时唤醒使能
    this->WDG_EN = DISABLE;             // 独立看门狗使能
    this->WTH = SECOND2IWDGCNT(1);      // 看门狗定时唤醒时间,大于复位时间后无效
    this->RTH = SECOND2IWDGCNT(2);      // 看门狗超时复位时间（21位计数器，但低12恒位0）
                                        // SECOND2IWDGCNT输入参数，秒
}
/*******************************************************************************
 函数名称：    void IWDG_DISABLE(void)
 功能描述：    关闭看门狗
 输入参数：    state :看门狗状态 ：  ENABLE 开启  DISABLE 关闭
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/09    V1.0           Yangzj              创建
 *******************************************************************************/
void IWDG_DISABLE(void)
{
    IWDG->CFG = 0;
}
/*******************************************************************************
 函数名称：    void IWDG_Feed(void)
 功能描述：    看门狗喂狗
 输入参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/09    V1.0           Yangzj              创建
 *******************************************************************************/
void IWDG_Feed(void)
{
    IWDG->PSW = PSW_IWDG_PRE;
    IWDG->CLR = PSW_IWDG_CLR;
}
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
