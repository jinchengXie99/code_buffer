/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc03x_I2C.c
 * 文件标识：
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： YangZJ
 * 完成日期： 2021年11月09日
 *
 *******************************************************************************/
 #include "lks32mc03x_i2c.h"
 #include "lks32mc03x_sys.h"
/*******************************************************************************
 函数名称：    void I2C_Init(I2C_InitTypeDef *this)
 功能描述：    I2C初始化
 输入参数：    this I2C配置结构体
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
void I2C_Init(I2C_InitTypeDef *this)
{
    SYS_ModuleClockCmd(SYS_Module_I2C,ENABLE);

    I2C -> ADDR  = (this -> ADDR_CMP_EN << 7) | (this -> ADDR);
    I2C -> CFG   = (this -> IE          << 7) | (this -> TC_IE       << 6) | 
                   (this -> BUS_ERR_IE  << 5) | (this -> STOP_IE     << 4) |
                   (this -> MST_MODE    << 1) | (this -> SLV_MODE);
    I2C -> SCR   = 0;
    I2C -> DATA  = 0;
    I2C -> MSCR  = 0;
    I2C -> BCR   = (this -> NACK_IE     << 7) | (this -> ADDR_CMP_IE << 6) |
                   (this -> BUSRT_EN    << 5);
    I2C -> BSIZE = this -> SIZE;
}
/*******************************************************************************
 函数名称：    void I2C_StrutInit(I2C_InitTypeDef *this)
 功能描述：    I2C结构体初始化
 输入参数：    this I2C配置结构体
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
void I2C_StrutInit(I2C_InitTypeDef *this)
{
    this -> ADDR_CMP_EN = ENABLE;   // I2C 硬件地址比较使能开关
    this -> ADDR        = 0x00;     // 仅用于从模式下，I2C 设备硬件地址
                        
    this -> IE          = DISABLE;  // I2C 中断使能信号
    this -> TC_IE       = DISABLE;  // I2C 数据传输完成中断使能
    this -> BUS_ERR_IE  = DISABLE;  // I2C 总线错误事件中断使能信号
    this -> STOP_IE     = DISABLE;  // I2C STOP 事件中断使能信号
    this -> NACK_IE     = DISABLE;  // NACK 事件中断使能
    this -> ADDR_CMP_IE = DISABLE;  // 硬件地址匹配中断使能
                        
    this -> MST_MODE    = DISABLE;  // I2C 主模式使能信号
    this -> SLV_MODE    = DISABLE;  // I2C 从模式使能信号
                        
    this -> BUSRT_EN    = DISABLE;  // I2C 多数据传输使能
    this -> SIZE        = 0;        // I2C 数据传输长度
}
/*******************************************************************************
 函数名称：    void I2C_StrutInit(I2C_InitTypeDef *this)
 功能描述：    读I2C总线状态
 输入参数：    this I2C配置结构体
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
u8 Read_I2c_Bus_State(u16 n)
{
    if(I2C->SCR&n)
    {
        return ENABLE;
    }
    else
    {
        return DISABLE;
    }
}
/*******************************************************************************
 函数名称：    void I2C_StrutInit(I2C_InitTypeDef *this)
 功能描述：    I2C总线状态复位
 输入参数：    this I2C配置结构体
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           Yangzj              创建
 *******************************************************************************/
void Clear_I2c_Bus_State(u16 n)
{
    I2C_SCR &= ~n;
}
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
