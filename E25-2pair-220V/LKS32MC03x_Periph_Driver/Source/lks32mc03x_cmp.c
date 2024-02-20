/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_cmp.c
 * 文件标识：
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/
 #include "lks32mc03x_cmp.h"
 #include "lks32mc03x_sys.h"
/*******************************************************************************
 函数名称：    void cmp_Init(CMP_InitTypeDef *this)
 功能描述：    比较器初始化
 输入参数：    this：I2C配置结构体
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj            ??
 2021/11/17    V1.1           Yangzj            增加CMP信号来源选择
 *******************************************************************************/
void CMP_Init(CMP_InitTypeDef *this)
{
    SYS_ModuleClockCmd(SYS_Module_CMP,ENABLE);

    CMP -> IE      = (this -> CMP1_RE           <<  9) | (this -> CMP0_RE           <<  8) |
                     (this -> CMP1_IE           <<  1) | (this -> CMP0_IE);            
    CMP -> TCLK    = (this -> FIL_CLK10_DIV16   <<  4) | (this -> CLK10_EN          <<  3) |
                     (this -> FIL_CLK10_DIV2);                                         
    CMP -> CFG     = (this -> CMP1_W_PWM_POL    <<  7) | (this -> CMP1_IRQ_TRIG     <<  6) |
                     (this -> CMP1_IN_EN        <<  5) | (this -> CMP1_POL          <<  4) |
                     (this -> CMP0_W_PWM_POL    <<  3) | (this -> CMP0_IRQ_TRIG     <<  2) |
                     (this -> CMP0_IN_EN        <<  1) | (this -> CMP0_POL);           
    CMP -> BLCWIN  = (this -> CMP1_CHN3P_WIN_EN <<  7) | (this -> CMP1_CHN2P_WIN_EN <<  6) |
                     (this -> CMP1_CHN1P_WIN_EN <<  5) | (this -> CMP1_CHN0P_WIN_EN <<  4) |
                     (this -> CMP0_CHN3P_WIN_EN <<  3) | (this -> CMP0_CHN2P_WIN_EN <<  2) |
                     (this -> CMP0_CHN1P_WIN_EN <<  1) | (this -> CMP0_CHN0P_WIN_EN);
    SYS_WR_PROTECT = 0x7A83;
    SYS_AFE_REG1   = (SYS_AFE_REG1 & (BIT15 | BIT6 | BIT1 | BIT0)) |
                     (this -> CMP1_SELP         << 12) | (this -> CMP_FT            << 11) |
                     (this -> CMP0_SELP         <<  8) | (this -> CMP_HYS           <<  7) |
                     (this -> CMP1_SELN         <<  4) | (this -> CMP0_SELN         <<  2);
    SYS_WR_PROTECT = 0;
    CMP -> IF = 3;
}
/*******************************************************************************
 函数名称：    void CMP_StructInit(CMP_InitTypeDef *this)
 功能描述：    CMP配置结构体初始化
 输入参数：    this CMP配置结构体
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj            创建
 2021/11/17    V1.1           Yangzj            增加CMP信号来源初始化
 *******************************************************************************/
void CMP_StructInit(CMP_InitTypeDef *this)
{
    // 比较器IO滤波
    this -> FIL_CLK10_DIV16   = 0 ;                     // 比较器 1/0 滤波
    this -> CLK10_EN          = ENABLE ;                // 比较器 1/0 滤波时钟使能
    this -> FIL_CLK10_DIV2    = 0 ;                     // 比较器 1/0 滤波时钟分频
    this -> CMP_FT            = ENABLE ;                // 比较器快速比较使能
    this -> CMP_HYS           = CMP_HYS_20mV ;          // 比较器回差选择
        
    //比较器1      
    this -> CMP1_RE           = DISABLE ;               // 比较器 1 DMA 请求使能
    this -> CMP1_IE           = DISABLE ;               // 比较器 1 中断使能
    this -> CMP1_W_PWM_POL    = 0 ;                     // 比较器 1 开窗 PWM 信号极性选择
    this -> CMP1_IRQ_TRIG     = DISABLE ;               // 比较器 1 边沿触发使能
    this -> CMP1_IN_EN        = 0 ;                     // 比较器 1 信号输入使能
    this -> CMP1_POL          = 0 ;                     // 比较器 1 极性选择
    this -> CMP1_SELP         = CMP1_SELP_CMP1_IP0 ;    // 比较器 1 信号正端选择
    this -> CMP1_SELN         = CMP1_SELN_CMP1_IN ;     // 比较器 1 信号负端选择
    this -> CMP1_CHN3P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN3_P 通道使能比较器 1 开窗
    this -> CMP1_CHN2P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN2_P 通道使能比较器 1 开窗
    this -> CMP1_CHN1P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN1_P 通道使能比较器 1 开窗
    this -> CMP1_CHN0P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN0_P 通道使能比较器 1 开窗

    //比较器0
    this -> CMP0_IE           = DISABLE ;               // 比较器 0 中断使能
    this -> CMP0_RE           = DISABLE ;               // 比较器 0 DMA 请求使能
    this -> CMP0_W_PWM_POL    = 0 ;                     // 比较器 1 开窗 PWM 信号极性选择
    this -> CMP0_IRQ_TRIG     = DISABLE ;               // 比较器 1 边沿触发使能
    this -> CMP0_IN_EN        = 0 ;                     // 比较器 1 信号输入使能
    this -> CMP0_POL          = 0 ;                     // 比较器 1 极性选择
    this -> CMP0_SELP         = CMP0_SELP_CMP0_IP0 ;    // 比较器 0 信号正端选择
    this -> CMP0_SELN         = CMP0_SELN_CMP0_IN ;     // 比较器 0 信号负端选择
    this -> CMP0_CHN3P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN3_P 通道使能比较器 0 开窗
    this -> CMP0_CHN2P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN2_P 通道使能比较器 0 开窗
    this -> CMP0_CHN1P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN1_P 通道使能比较器 0 开窗
    this -> CMP0_CHN0P_WIN_EN = DISABLE ;               // MCPWM 模块 CHN0_P 通道使能比较器 0 开窗
}
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
