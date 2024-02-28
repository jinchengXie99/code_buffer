/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_sys.c
 * 文件标识：
 * 内容摘要： SYS外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/
#include "lks32mc03x_sys.h"
#include "lks32mc03x.h"


/*******************************************************************************
 函数名称：    void SYS_Init(SYS_InitTypeDef* SYS_InitStruct)
 功能描述：    SYS模块初始化函数
 操作的表：    无
 输入参数：    SYS_InitTypeDef* SYS_InitStruct
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
void SYS_Init(SYS_InitTypeDef* SYS_InitStruct)
{
    uint32_t tmp1;

    SYS_WR_PROTECT = 0x7A83;    //解除系统寄存器写保护
    tmp1 = SYS_AFE_REG0;
    tmp1 &= 0x7FFF;
    tmp1 |= (SYS_InitStruct->PLL_SrcSel << 15);
    SYS_AFE_REG0 = tmp1;
    SYS_CLK_CFG = SYS_InitStruct->PLL_DivSel + (SYS_InitStruct->PLL_ReDiv << 8);
    
    SYS_CLK_DIV0 = SYS_InitStruct->Clk_DivI2C;
    SYS_CLK_DIV2 = SYS_InitStruct->Clk_DivUART;
    SYS_CLK_FEN = SYS_InitStruct->Clk_FEN;
    SYS_WR_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void SYS_StructInit(SYS_InitTypeDef* SYS_InitStruct)
 功能描述：    SYS结构体初始化
 操作的表：    无
 输入参数：    SYS_InitTypeDef* SYS_InitStruct
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
void SYS_StructInit(SYS_InitTypeDef* SYS_InitStruct)
{
    SYS_InitStruct -> PLL_SrcSel  = SYS_PLLSRSEL_RCH;
    SYS_InitStruct -> PLL_DivSel  = 0xFF;
    SYS_InitStruct -> PLL_ReDiv   = SYS_PLLREDIV_1;
                                  
    SYS_InitStruct -> Clk_DivI2C  = SYS_Clk_SPIDiv1;
    SYS_InitStruct -> Clk_DivUART = SYS_Clk_UARTDiv1;
    SYS_InitStruct -> Clk_FEN     = 0;
}

/*******************************************************************************
 函数名称：    void SYS_ClearRst(void)
 功能描述：    SYS清除复位标志
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    由于复位记录工作于低速时钟域，清除执行完成需要一定时间，不应清除后立即读记录状态
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
void SYS_ClearRst(void)
{
    AON_EVT_RCD = 0xCA40;
}

/*******************************************************************************
 函数名称：    uint32_t SYS_GetRstSource(void)
 功能描述：    获得SYS复位源信号
 操作的表：    无
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
uint32_t SYS_GetRstSource(void)
{
    return AON_EVT_RCD;
}

/*******************************************************************************
 函数名称：    void SYS_ModuleClockCmd(uint32_t nModule, FuncState state)
 功能描述：    模块时钟使能和停止
 操作的表：    无
 输入参数：    uint32_t nModule：对应的模块
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
void SYS_ModuleClockCmd(uint32_t nModule, FuncState state)
{
    SYS_WR_PROTECT = 0x7A83;    //解除系统寄存器写保护
    if (state != DISABLE)
    {
        SYS_CLK_FEN |= nModule;
    }
    else
    {
        SYS_CLK_FEN &= ~nModule;
    }
    SYS_WR_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void SYS_AnalogModuleClockCmd(uint32_t nModule, FuncState state)
 功能描述：    模拟模块使能和停止
 操作的表：    无
 输入参数：    uint32_t nModule：对应的模块
 输出参数：    无
 返 回 值：    无
 其它说明：    模拟部分时钟是使能：PGA、ADC、DAC
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              创建
 *******************************************************************************/
void SYS_AnalogModuleClockCmd(uint32_t nModule, FuncState state)
{
    SYS_WR_PROTECT = 0x7A83;    //解除系统寄存器写保护
    
    if (state == ENABLE)
    {
        if(nModule & SYS_AnalogModule_BGP)
        {//BGP与其他模块极性相反
            SYS_AFE_REG0 &= ~SYS_AnalogModule_BGP;
            nModule &= ~SYS_AnalogModule_BGP;
        }
        SYS_AFE_REG0 |= nModule;
    }
    else
    {
        if(nModule & SYS_AnalogModule_BGP)
        {//BGP与其他模块极性相反
            SYS_AFE_REG0 |= SYS_AnalogModule_BGP;
            nModule |= SYS_AnalogModule_BGP;
        }
        SYS_AFE_REG0 &= ~nModule;
    }
    SYS_WR_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void SYS_SoftResetModule(uint32_t nModule)
 功能描述：    模块软复位
 操作的表：    无
 输入参数：    uint32_t nModule：对应的模块
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
void SYS_SoftResetModule(uint32_t nModule)
{
    SYS_WR_PROTECT = 0x7A83;    //解除系统寄存器写保护
    SYS_SFT_RST = nModule;
    SYS_SFT_RST = 0;
    SYS_WR_PROTECT = 0;
}
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
