/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： lks32mc081_flash.c
 * 文件标识：
 * 内容摘要： Flash外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet
 * 完成日期： 2019年3月5日
 *
 * 修改记录1：
 * 修改日期：2019年3月5日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet
 * 修改内容：创建
 *
 * 修改记录2：
 * 修改日期：
 * 版 本 号：
 * 修 改 人：
 * 修改内容：
 *
 *******************************************************************************/
#include "basic.h"
#include "lks32mc03x.h"
#include "lks32mc03x_Flash.h"
#include "lks32mc03x_nvr.h"

volatile u32 erase_flag = 0;
volatile u32 progm_flag = 0;



/*******************************************************************************
 函数名称：    void EraseSector(unsigned long adr, unsigned long nvr)
 功能描述：    Flash按扇区擦除操作
 操作的表：    无
 输入参数：    adr：       扇区地址  (一个扇区512字节)
               nvr：       为0x800的时候擦除NVR, 为0时擦除主Flash 
               erase_flag：调用函数前必须赋值0x9A0D361F，否则不执行擦除，用来防程序跑飞
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/12/15      V1.2           DengT            创建
 *******************************************************************************/
void EraseSector(u32 adr, u16 nvr, u32 erase_flag)
{
   u32 key1, key2, key3;

  key1 = 0x0;
  key2 = 0x0;
  key3 = 0x0;
	SYS_WR_PROTECT = 0x7a83;
  if (erase_flag == 0x9A0D361F)
  { //get flash_cfg addr
    SYS_FLSE = 0x8FCA;
    key1 = 0xB00C060A;
    key2 = 0x2A003015;
    key3 = erase_flag ^ key1 ^ key2;
    REG32(key3) &= ~0x80008000;
    FLASH_ADDR = adr;
    REG32(key3) |= (0x80000000);
    key1 = 0x0;
    key2 = 0x0;
    key3 = 0x0;
  }

  if (erase_flag == 0x9A0D361F)
  { //get flash_erase addr
    SYS_FLSE = 0x8FCA;
    key1 = 0xB001341A;
    key2 = 0x2A0D0215;
    key3 = erase_flag ^ key1 ^ key2;
    REG32(key3) = 0x7654DCBA; //trig sector erase, FLASH_ERASE
    FLASH_CFG &= ~0x80000000;
    erase_flag = 0x00000000;
    key1 = 0x0;
    key2 = 0x0;
    key3 = 0x0;
  }
  SYS_FLSE = 0x0;
  FLASH_CFG &= ~0x80000000;
  erase_flag = 0x00000000;
  key1 = 0x0;
  key2 = 0x0;
  key3 = 0x0;
	SYS_WR_PROTECT = 0;
}


/*******************************************************************************
 函数名称：    ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
 功能描述：    Flash编程操作
 操作的表：    无
 输入参数：    adr：扇区地址  
               sz : 编程字节数量(0~512字节) 
               buf: 要编程的数据数组指针；
               nvr：       为0x800的时候编程NVR, 为0时编程主Flash 
               erase_flag：调用函数前必须赋值0x9AFDA40C，否则不执行编程，用来防程序跑飞
 输出参数：    无
 返 回 值：    1：编程成功；  0: 编程失败
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/12/15      V1.2           DengT            创建
 *******************************************************************************/
int ProgramPage(u32 adr, u32 sz, u8 *buf, u16 nvr, u32 progm_flag)
{

 /* Add your Code */
  volatile u32 Addr, size;
  volatile u8 *p;
  volatile u8 t_rlt = 1;
  u32 key1, key2, key3;
  key1 = 0x0;
  key2 = 0x0;
  key3 = 0x0;
SYS_WR_PROTECT = 0x7a83;
  if (progm_flag == 0x9AFDA40C)
  {
    SYS_FLSP = 0x8F35;
    //get flash_cfg addr
    key1 = 0x6A5C040F;
    key2 = 0xF0A0A003;
    key3 = progm_flag ^ key1 ^ key2;
    REG32(key3) &= ~0x80008000;
    FLASH_ADDR = adr;
    REG32(key3) |= (0x08000000);
    progm_flag = 0;
    key1 = 0x0;
    key2 = 0x0;
    key3 = 0x0;

    p = buf;
    size = (sz + 3) & ~3; // align the word
    Addr = adr;
    while (size)
    {
      FLASH_ADDR = (Addr & ~0x00000003);                                        // address alignment
      FLASH_WDATA = *p + (*(p + 1) << 8) + (*(p + 2) << 16) + (*(p + 3) << 24); // write data to flash

      p += 4;
      size -= 4; // Go to next word
      Addr += 4;
    }

    p = buf;
    size = (sz + 3) & ~3; // align the word
    Addr = adr;
    while (size)
    {
      u32 t_RData;
      FLASH_ADDR = (Addr & ~0x00000003); // address alignment
      t_RData = *p + (*(p + 1) << 8) + (*(p + 2) << 16) + (*(p + 3) << 24);
      if (t_RData != FLASH_RDATA)
      {
        t_rlt = 0;
      }

      p += 4;
      size -= 4;
      Addr += 4;
    }
  }
  SYS_FLSP = 0x0;
  FLASH_CFG &= ~0x08000800;
  key1 = 0x0;
  key2 = 0x0;
  key3 = 0x0;
  progm_flag = 0;
	SYS_WR_PROTECT = 0;
  return (t_rlt);
}

/*******************************************************************************
 函数名称：    void Read_Flash(void)
 功能描述：    读取FLASH数据函数
 输入参数：    adr：要读取数据的地址
               buf:读取数据存储的地址
               sz:要读取的地址数
 输出参数：    读取的数据值
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/3/5      V1.0            HMG               创建
 *******************************************************************************/
void Read_Flash(uint32_t adr, u32 *buf, unsigned long sz, u16 nvr)
{
	uint32_t i;
	if(nvr==0x800){//nvm 
	  FLASH_CFG |= 0x00000800;
	}
	else if(nvr==0){//flash main
		FLASH_CFG &= ~0x00000800; //访问flash内MAIN区域
	}
	FLASH_CFG |= 0x1 << 23;	  //地址递增打开

	FLASH_ADDR = adr;         //写入FALSH访问地址
	for (i = 0; i < sz; i++)
	{
		buf[i] = FLASH_RDATA; //读取FLASH内容
	}
	FLASH_CFG &= ~(0x1 << 23); //地址递增关闭
}





u32  NVR_UserRoomRead(u32 addr)
{
    u32 readData;
	SYS_WR_PROTECT= 0x00007A83;//config key
    FLASH_CFG |= 0x00000800;
    //FLASH_ADDR     = 0x59000000;
	 FLASH_ADDR     = 0x95000000;
    FLASH_ADDR     = addr;
    readData = FLASH_RDATA;

    FLASH_ADDR     = 0x95000000;

    FLASH_CFG &=~0x00000800;
	SYS_WR_PROTECT= 1;//config key
    return readData;
}
void Nvr_UserRoomErase(UINT8 addr)
{
	  erase_flag = 0x9A0D361F;
		if(addr<128)
	{
		EraseSector(0x0000,0x800, 0x9A0D361F);
	}
	else
	{
		EraseSector(0x0200,0x800, 0x9A0D361F);
	}
	
}

void Nvr_UserRoomWrite(UINT8 addr,UINT32 data)
{
	UINT8 uArr[4],i;
	__disable_irq();
	for(i=0;i<4;i++)
	{
		uArr[i] = data&(0xff);
	  data>>=8;
	}



	progm_flag= 0x9AFDA40C;

	
	
		if( ProgramPage((UINT32)addr<<2,4,uArr,0x800, 0x9A0D361F) )
		{
		}
		else
		{
			//error
		}

	
	

}


void GetRchTrimVal(INT16 ax)
{
 UINT8 acc,bx;
 acc = SYS_AFE_REG6 & (0x3f);
 if((acc >= 0x20) && (acc <= (0x20 + ax)) )
 {
  acc = 0x20;
 }
 else if((acc <= 0x3f) && (acc > (0x20 + ax)) )
 {
  acc -= ax;
 }
 else if((acc <= 0x1f) && (acc >= (ax)) )
 {
  acc -= ax;
 }
 else
 {
  bx = (acc - ax)&(0x0f);
  acc = bx + 0x30;
 }
    
  SYS_AFE_REG6 = acc;
}


void SYS_VolSelModule(uint32_t Vol)
{
 if((Read_Trim(0x000001D4)>>16) >= 0x03)
 {
   if(Vol == 0) //3.3 Voltage
  {
    SYS_AFE_REG5 = (Read_Trim(0x00000198)>>16) & 0xffff;
     SYS_AFE_REG6 = (Read_Trim(0x0000019C)>>16) & 0xffff;  
  }
  else         // 5.0 Voltage
  {
     SYS_AFE_REG5 = Read_Trim(0x00000198) & 0xffff;
      SYS_AFE_REG6 = Read_Trim(0x0000019C) & 0xffff; 
  }
 }
  else
  {
  GetRchTrimVal(2);
  }
}

