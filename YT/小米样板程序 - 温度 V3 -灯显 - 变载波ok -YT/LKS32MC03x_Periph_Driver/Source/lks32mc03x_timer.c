/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_timer.c
 * 文件标识：
 * 内容摘要： Timer配置程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021/11/10
 *
 *******************************************************************************/
#include "lks32mc03x.h"
#include "lks32mc03x_timer.h"
#include "lks32mc03x_sys.h"

/*******************************************************************************
 函数名称：    void TIM_TimerInit(TIM_TimerTypeDef *TIMERx, TIM_TimerInitTypeDef *this)
 功能描述：    Timer初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
void TIM_TimerInit(TIM_TimerTypeDef *TIMERx, TIM_TimerInitTypeDef *this)
{
    if(TIMERx == TIMER0)
    {
        SYS_ModuleClockCmd(SYS_Module_TIMER0,ENABLE);     //打开Timer时钟
    }
    if(TIMERx == TIMER1)
    {
        SYS_ModuleClockCmd(SYS_Module_TIMER1,ENABLE);     //打开Timer时钟
    }
    TIMERx -> CFG  = (this->EN            << 31) | (this->CAP1_CLR_EN   << 27) |
                     (this->CAP0_CLR_EN   << 26) | (this->ETON          << 24) |
                     (this->CLK_DIV       << 20) | (this->CLK_SRC       << 16) |
                     (this->SRC1          << 12) | (this->CH1_POL       << 11) |
                     (this->CH1_MODE      << 10) | (this->CH1_FE_CAP_EN <<  9) |
                     (this->CH1_RE_CAP_EN <<  8) | (this->SRC0          <<  4) |
                     (this->CH0_POL       <<  3) | (this->CH0_MODE      <<  2) |
                     (this->CH0_FE_CAP_EN <<  1) | (this->CH0_RE_CAP_EN);
    TIMERx -> TH    = this->TH;
    TIMERx -> CNT   = this->CNT;
    TIMERx -> CMPT0 = this->CMP0;
    TIMERx -> CMPT1 = this->CMP1;
    TIMERx -> EVT   = this->EVT_SRC;
    TIMERx -> FLT   = this->FLT;
    TIMERx -> IE    = this->IE;
    TIMERx -> IF    = 0x7;
}
/*******************************************************************************
 函数名称：    void TIM_TimerStrutInit(TIM_TimerInitTypeDef *TIM_TimerInitStruct)
 功能描述：    Timer结构体初始化
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
#define DISABLE 0
void TIM_TimerStrutInit(TIM_TimerInitTypeDef *this)
{
    this -> EN            = DISABLE;               // Timer 模块使能
    this -> CAP1_CLR_EN   = DISABLE;      // 当发生 CAP1 捕获事件时，清零 Timer 计数器，高有效
    this -> CAP0_CLR_EN   = DISABLE;      // 当发生 CAP0 捕获事件时，清零 Timer 计数器，高有效
    this -> ETON          = DISABLE;             // Timer 计数器计数使能配置 0: 自动运行 1: 等待外部事件触发计数
    this -> CLK_DIV       = 0;                // Timer 计数器分频
    this -> CLK_SRC       = TIM_Clk_SRC_MCLK; // Timer 时钟源
    this -> TH            = 0;                     // Timer 计数器计数门限。计数器从 0 计数到 TH 值后再次回 0 开始计数
            
    this -> SRC1          = TIM_SRC1_1;       // Timer 通道 1 捕获模式信号来源
    this -> CH1_POL       = 1;             // Timer 通道 1 在比较模式下的输出极性控制，计数器回0后的输出值
    this -> CH1_MODE      = 0;            // Timer 通道 1 工作模式选择，0：比较模式，1：捕获模式
    this -> CH1_FE_CAP_EN = DISABLE; // Timer 通道 1 下降沿捕获事件使能。1:使能；0:关闭
    this -> CH1_RE_CAP_EN = DISABLE; // Timer 通道 1 上升沿捕获事件使能。1:使能；0:关闭
    this -> CMP1          = 0;    // Timer 通道 1 比较门限
            
    this -> SRC0          = TIM_SRC1_0;       // Timer 通道 0 捕获模式信号来源
    this -> CH0_POL       = 1;             // Timer 通道 0 在比较模式下的输出极性控制，计数器回0后的输出值
    this -> CH0_MODE      = 0;            // Timer 通道 0 工作模式选择，0：比较模式，1：捕获模式
    this -> CH0_FE_CAP_EN = DISABLE; // Timer 通道 0 下降沿捕获事件使能。1:使能；0:关闭
    this -> CH0_RE_CAP_EN = DISABLE; // Timer 通道 0 上升沿捕获事件使能。1:使能；0:关闭
    this -> CMP0          = 0;    // Timer 通道 0 比较门限
            
    this -> CNT           = 0;     // Timer 计数器当前计数值。写操作可以写入新的计数值。
    this -> EVT_SRC       = 0; // Timer 计数使能开始后，外部事件选择
    this -> FLT           = 0;     // 通道 0/1 信号滤波宽度选择，0-255
    this -> IE            = 0;      // Timer 中断使能
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
