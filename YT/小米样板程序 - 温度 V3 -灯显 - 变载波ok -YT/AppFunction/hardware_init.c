/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� hardware_init.c
 * �ļ���ʶ��
 * ����ժҪ�� Ӳ����ʼ������
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet Li
 * ������ڣ� 2015��11��5��
 *
 * �޸ļ�¼1��
 * �޸����ڣ�
 * �� �� �ţ�
 * �� �� �ˣ�
 * �޸����ݣ�
 *
 *******************************************************************************/
#include "basic.h"
#include "hardware_config.h"
#include "global_variable.h"

void SoftDelay(u32 cnt);
void UART_init(void);
void HALL_Perip_Init(void);
void TempSensor_Init(void);

/*******************************************************************************
 �������ƣ�    void Hardware_init(void)
 ����������    Ӳ�����ֳ�ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void Hardware_init(void)
{
  __disable_irq();                  /* �ر��ж� �ж��ܿ��� */
  SYS_WR_PROTECT = 0x7a83;

  IWDG_PSW = 0xA6B4;
  IWDG_CFG = 0x3C00;

  FLASH_CFG |= 0x00080000;          /* enable prefetch */

//  DSP_init();                       /* DSPģ���ʼ��*/

  SYS_ModuleClockCmd(SYS_Module_DIV, ENABLE);

//  UART_init();                      /* ���ڳ�ʼ��UART0*/
  ADC_init();                      /* ADC��ʼ�� */
  MCPWM_init();                     /* PWM��ʼ�� */
  UTimer_init();                    /* ͨ�ü�������ʼ�� */
  GPIO_init();                      /* GPIO��ʼ�� */
  DAC_Init();                       /* DAC ��ʼ�� */
  PGA_Init();                       /* PGA ��ʼ�� */
  CMP_init();                       /* �Ƚ�����ʼ�� */
  HALL_Perip_Init();                /* HALLģ���ʼ�� */
  TempSensor_Init();                /* �¶ȴ�������ʼ�� */

  SoftDelay(100);                   /* ��ʱ�ȴ�Ӳ����ʼ���ȶ� */


//  NVIC_SetPriority(UART_IRQn, 2);  /* ����UART0�ж����ȼ�Ϊ2 | ��0��1��2��3�ļ��ж����ȼ���0Ϊ���*/
  NVIC_SetPriority(ADC_IRQn, 1);
  NVIC_SetPriority(HALL_IRQn, 2);
  NVIC_SetPriority(MCPWM0_IRQn, 2);
  NVIC_SetPriority(CMP_IRQn, 0);    /* ����CMP_IRQn�ж����ȼ�Ϊ0 | ��0��1��2��3�ļ��ж����ȼ���0Ϊ���*/
  NVIC_SetPriority(TIMER0_IRQn, 2);
  NVIC_SetPriority(TIMER1_IRQn, 2);
    
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
  NVIC_SetPriority(MCPWM_IRQn, 1);
  NVIC_EnableIRQ (MCPWM_IRQn);
#else
  NVIC_EnableIRQ(ADC_IRQn);        /* enable the ADC0 interrupt */
#endif
  NVIC_EnableIRQ(MCPWM0_IRQn);
 // NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);

  NVIC_EnableIRQ(CMP_IRQn);         /* �򿪱Ƚ����ж� */

  __enable_irq();                   /* �������ж� */
}

/*******************************************************************************
 �������ƣ�    void Clock_Init(void)
 ����������    ʱ������
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void Clock_Init(void)
{

  SYS_WR_PROTECT = 0x7a83;   /* ���ϵͳ�Ĵ���д���� */

  SYS_AFE_REG0 |= BIT15;     /* BIT15:PLLPDN */
  SYS_CLK_CFG |= 0x000001ff; /* BIT8:0: CLK_HS,1:PLL  | BIT[7:0]CLK_DIV  | 1ff��Ӧ48Mʱ�� */

//    SYS_CLK_FEN = 0xfff;

}

/*******************************************************************************
 �������ƣ�    void SystemInit(void)
 ����������    Ӳ��ϵͳ��ʼ��������ʱ�ӳ�ʼ������
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2016/3/14      V1.0           Howlet Li          ����
 *******************************************************************************/
void SystemInit(void)
{
  Clock_Init();  /* ʱ�ӳ�ʼ�� */
}

/*******************************************************************************
 �������ƣ�    void TempSensor_Init(void)
 ����������    �¶ȴ�������ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void TempSensor_Init(void)
{

  SYS_WR_PROTECT = 0x7a83;   /* ���ϵͳ�Ĵ���д���� */

  SYS_AnalogModuleClockCmd(SYS_AnalogModule_TMP,ENABLE);    /* ���¶ȴ��������� */

//    m_TempertureCof.nCofA    = Read_Trim(0x00000398);
//    m_TempertureCof.nOffsetB = Read_Trim(0x0000039C);

}

/*******************************************************************************
 �������ƣ�    void PGA_Init(void)
 ����������    PGA��ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2016/3/15      V1.0           Howlet Li          ����
 *******************************************************************************/
void PGA_Init(void)
{

  SYS_AnalogModuleClockCmd(SYS_AnalogModule_OPA, ENABLE);

  SYS_WR_PROTECT = 0x7a83;

  SYS_AFE_REG0 &= ~3;
  SYS_AFE_REG0 |= OPA0_GIAN; /* OPA�������� */
}

/*******************************************************************************
 �������ƣ�    void CMP_Init(void)
 ����������    �Ƚ�����ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2016/3/15      V1.0           Howlet Li          ����
 *******************************************************************************/
void CMP_init(void)
{
  CMP_InitTypeDef CMP_InitStruct;
  CMP_StructInit(&CMP_InitStruct);
	
	SYS_AnalogModuleClockCmd( SYS_AnalogModule_CMP0,DISABLE); 	/* �Ƚ���0���� */
	SYS_AnalogModuleClockCmd( SYS_AnalogModule_CMP1,ENABLE);  /* �Ƚ���1���� */
	
  CMP_InitStruct.FIL_CLK10_DIV16 = 10;   // �Ƚ��� 1/0 �˲�
  CMP_InitStruct.FIL_CLK10_DIV2 = 2;     // �Ƚ��� 1/0 �˲�ʱ�ӷ�Ƶ
	
  // �Ƚ���1
  CMP_InitStruct.CMP1_IE = ENABLE;      // �Ƚ��� 1 �ж�ʹ��
  CMP_InitStruct.CMP1_IN_EN = ENABLE;   // �Ƚ��� 1 �ź�����ʹ��
  CMP_InitStruct.CMP1_POL = 0;           // �Ƚ��� 1 ����ѡ��0:�ߵ�ƽ��Ч��1:�͵�ƽ��Ч
    
  CMP_InitStruct.CMP1_SELN = CMP1_SELN_DAC;
  CMP_InitStruct.CMP1_SELP = CMP1_SELP_CMP1_IP1;
    
  // �Ƚ���0
  CMP_InitStruct.CMP0_IE = DISABLE;            // �Ƚ��� 0 �ж�ʹ��
  CMP_InitStruct.CMP0_IN_EN = DISABLE;         // �Ƚ��� 0 �ź�����ʹ��
  CMP_InitStruct.CMP0_POL = 0;                // �Ƚ��� 0 ����ѡ��0:�ߵ�ƽ��Ч��1:�͵�ƽ��Ч


  CMP_InitStruct.CMP0_SELN = CMP0_SELN_DAC;
  CMP_InitStruct.CMP0_SELP = CMP0_SELP_CMP0_IP3;
	
	CMP_InitStruct.CMP_HYS = CMP_HYS_0mV;

  CMP_Init(&CMP_InitStruct);
}

/*******************************************************************************
 �������ƣ�    void DAC_Init(void)
 ����������    DAC��ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2016/3/23      V1.0           Howlet Li          ����
 *******************************************************************************/
void DAC_Init(void)
{
  SYS_AnalogModuleClockCmd(SYS_AnalogModule_DAC, ENABLE);

  SYS_AFE_DAC = 50;                     /* 3*25/256/0.1 =3.0A, ����0.1Ϊĸ�߲�������*/
	
}

/*******************************************************************************
 �������ƣ�    void UART_init(void)
 ����������    UART0�Ĵ�������
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void UART_init(void)
{
  UART_InitTypeDef UART_InitStruct;

  UART_StructInit(&UART_InitStruct);
  UART_InitStruct.BaudRate = 38400;                 /* ���ò�����38400 */
  UART_InitStruct.WordLength = UART_WORDLENGTH_8b;  /* �������ݳ���8λ */
  UART_InitStruct.StopBits = UART_STOPBITS_1b;
  UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB;   /* �ȷ���LSB */
  UART_InitStruct.ParityMode = UART_Parity_NO;      /* ����żУ�� */
  UART_InitStruct.IRQEna = 0;
  UART_Init(UART0, &UART_InitStruct);
}

/*******************************************************************************
 �������ƣ�    void UART_SENDDATA(void)
 ����������    UART0���ͳ���
 ���������    n����Ҫ���͵�ֵ
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void UART_SENDDATA(UINT8 n)
{
  UART_BUFF = n;
}

///*******************************************************************************
// �������ƣ�    void UART_init(void)
// ����������    UART1�Ĵ�������
// ���������    ��
// ���������    ��
// �� �� ֵ��    ��
// ����˵����
// �޸�����      �汾��          �޸���            �޸�����
// -----------------------------------------------------------------------------
// 2015/11/5      V1.0           Howlet Li          ����
// *******************************************************************************/
//void UART1_init(void)
//{

//    UART_InitTypeDef UART_InitStruct;
//
//    UART_StructInit(&UART_InitStruct);
//    UART_InitStruct.BaudRate = 38400;                 /* ���ò�����38400 */
//    UART_InitStruct.WordLength = UART_WORDLENGTH_8b;  /* �������ݳ���8λ */
//    UART_InitStruct.StopBits = UART_STOPBITS_1b;
//    UART_InitStruct.FirstSend = UART_FIRSTSEND_LSB;   /* �ȷ���LSB */
//    UART_InitStruct.ParityMode = UART_Parity_NO;      /* ����żУ�� */
//    UART_InitStruct.IRQEna = 0;
//    UART_Init(UART1, &UART_InitStruct);

//}




/*******************************************************************************
 �������ƣ�    void ADC_init(void)
 ����������    ADC0Ӳ����ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void ADC_init(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;
  ADC_InitStructure.Align = ADC_LEFT_ALIGN;
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
  ADC_InitStructure.FirSeg_Ch = 2;                        /* ��һ�ι�����6��ͨ�� */
  ADC_InitStructure.SecSeg_Ch = 4;                        /* �ڶ��ι�����0��ͨ�� */
  ADC_InitStructure.MCPWM_Trigger_En = ADC_MCPWM_T0_TRG | ADC_MCPWM_T1_TRG;;  /* ��MCPWM_T0 MCPWM_T1Ӳ�������¼� */
  ADC_InitStructure.Trigger_Mode = ADC_2SEG_TRG;          /* ����ADCת��ģʽΪ˫��ʽ���� */
  ADC_InitStructure.IE = ADC_EOS1_IRQ_EN;                 /* ADC��һ�β��������ж�ʹ�� */
  ADC_InitStructure.ADC_CLK_SEL =  ADC_CLK_48M;
  ADC_InitStructure.ADC_SAMP_CLK = 12; /* 12��ʱ������ */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
  ADC_InitStructure.FirSeg_Ch = 6;                        /* ��һ�ι�����6��ͨ�� */
  ADC_InitStructure.SecSeg_Ch = 0;                        /* �ڶ��ι�����0��ͨ�� */
  ADC_InitStructure.Trigger_En = ENABLE ; /* ��MCPWM_T0 MCPWM_T1Ӳ�������¼� */
  ADC_InitStructure.SEL_En = 0 ; /* TADC������Դѡ��0:MCPWM��1:UTimer */
  ADC_InitStructure.Trigger_Mode = ADC_1SEG_TRG;          /* ����ADCת��ģʽΪ����ʽ���� */
  ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;                 /* ADC��һ�β��������ж�ʹ�� */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)
  ADC_InitStructure.FirSeg_Ch = 6;                        /* ��һ�ι�����6��ͨ�� */
  ADC_InitStructure.SecSeg_Ch = 0;                        /* �ڶ��ι�����0��ͨ�� */
  ADC_InitStructure.MCPWM_Trigger_En = ADC_MCPWM_T0_TRG ; /* ��MCPWM_T0 MCPWM_T1Ӳ�������¼� */
  ADC_InitStructure.Trigger_Mode = ADC_1SEG_TRG;          /* ����ADCת��ģʽΪ����ʽ���� */
  ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;                 /* ADC��һ�β��������ж�ʹ�� */
  ADC_InitStructure.ADC_CLK_SEL =  ADC_CLK_48M;
  ADC_InitStructure.ADC_SAMP_CLK = 20; /* 12��ʱ������ */
#endif
#endif
#endif

  ADC_Init(ADC, &ADC_InitStructure);

  ADC_IF = 0xff;
  ADC_STATE_RESET();
  ADC_NormalModeCFG();

  SYS_PROTECT = 0x7A83;    //���ϵͳ�Ĵ���д����
  SYS_AFE_REG2 = 0x00<< 8;
  SYS_WR_PROTECT = 0;    //ʹ��ϵͳ�Ĵ���д����

}

/*******************************************************************************
 �������ƣ�    void HALL_Init(void)
 ����������    GPIOӲ����ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2018/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void HALL_Perip_Init(void)
{
  HALL_InitTypeDef HALL_InitStruct;
  SYS_ModuleClockCmd(SYS_Module_HALL, ENABLE);


  HALL_StructInit(&HALL_InitStruct);
  HALL_InitStruct.FilterLen = 512;                /* Hall�ź������˲����� 512��ʱ������ */
  HALL_InitStruct.ClockDivision = HALL_CLK_DIV1;  /* ����Hallģ��ʱ�ӷ�Ƶϵ�� */
  HALL_InitStruct.Filter75_Ena = DISABLE;         /* Hall�ź��˲���ʽ��7��5ģʽ����ȫ1��Чģʽ */
  HALL_InitStruct.HALL_Ena = ENABLE;              /* ģ��ʹ�� */
  HALL_InitStruct.Capture_IRQ_Ena = ENABLE;       /* ��׽�ж�ʹ�� */
  HALL_InitStruct.OverFlow_IRQ_Ena = ENABLE;      /* ��ʱ�ж�ʹ�� */
  HALL_InitStruct.CountTH = 9600000;              /* Hallģ�����ģֵ����������ģֵ�������ʱ�ж� */

  HALL_Init(&HALL_InitStruct);
}
/*******************************************************************************
 �������ƣ�    void GPIO_init(void)
 ����������    GPIOӲ����ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/

void GPIO_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);

  /* MCPWM P0.9 P0.11~P0.15 */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIO0, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_9, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_11, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_12, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_13, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_14, AF3_MCPWM);
  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_15, AF3_MCPWM);

}

/*******************************************************************************
 �������ƣ�    void UTimer_init(void)
 ����������    UTimerӲ����ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void UTimer_init(void)
{
    TIM_TimerInitTypeDef TIM_InitStruct;

    TIM_TimerStrutInit(&TIM_InitStruct);
    TIM_InitStruct.EN = ENABLE;
    TIM_InitStruct.TH = 4800;
    TIM_InitStruct.IE = TIM_IRQ_IE_ZC;
    TIM_TimerInit(TIMER1, &TIM_InitStruct);

}

void MCPWM_init(void)
{
  MCPWM_InitTypeDef MCPWM_InitStructure;

  MCPWM_StructInit(&MCPWM_InitStructure);

  MCPWM_InitStructure.CLK_DIV = 0;                          /* MCPWMʱ�ӷ�Ƶ���� */
  MCPWM_InitStructure.MCLK_EN = ENABLE;                     /* ģ��ʱ�ӿ��� */
  MCPWM_InitStructure.MCPWM_Cnt0_EN = ENABLE;               /* ʱ��0����������ʼ����ʹ�ܿ��� */
  MCPWM_InitStructure.MCPWM_Cnt1_EN = ENABLE;               /* ʱ��1����������ʼ����ʹ�ܿ��� */
  MCPWM_InitStructure.MCPWM_WorkModeCH0 = CENTRAL_PWM_MODE;
  MCPWM_InitStructure.MCPWM_WorkModeCH1 = CENTRAL_PWM_MODE; /* ͨ������ģʽ���ã����Ķ������ض��� */
  MCPWM_InitStructure.MCPWM_WorkModeCH2 = CENTRAL_PWM_MODE;

  /* �Զ�����ʹ�ܼĴ��� MCPWM_TH00 �Զ�����ʹ�� MCPWM_TMR0 �Զ�����ʹ�� MCPWM_0TH �Զ�����ʹ�� MCPWM_0CNT �Զ�����ʹ��*/
  MCPWM_InitStructure.AUEN = TH00_AUEN | TH01_AUEN | TH10_AUEN | TH11_AUEN |
                             TH20_AUEN | TH21_AUEN | TMR0_AUEN | TMR1_AUEN |
                             TMR2_AUEN | TMR3_AUEN | TH0_AUEN | TH30_AUEN | TH31_AUEN ;

  MCPWM_InitStructure.GPIO_BKIN_Filter = 12;                /* ��ͣ�¼�(����IO���ź�)�����˲���ʱ������ */
  MCPWM_InitStructure.CMP_BKIN_Filter = 1;                 /* ��ͣ�¼�(���ԱȽ����ź�)�����˲���ʱ������ */

  MCPWM_InitStructure.TimeBase0_PERIOD = PWM_PERIOD;        /* ʱ��0�������� */
  MCPWM_InitStructure.TimeBase1_PERIOD = PWM_PERIOD;        /* ʱ��1�������� */
  MCPWM_InitStructure.TriggerPoint0 = (u16)(PWM_PERIOD - 10);//100 /* MCPWM_TMR0 ADC�����¼�T0 ���� */
  MCPWM_InitStructure.TriggerPoint1 = (u16)(800-PWM_PERIOD);/* MCPWM_TMR1 ADC�����¼�T1 ���� */
  MCPWM_InitStructure.DeadTimeCH0N = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH0P = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH1N = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH1P = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH2N = DEADTIME;
  MCPWM_InitStructure.DeadTimeCH2P = DEADTIME;              /* ����ʱ������ */

#if (PRE_DRIVER_POLARITY == P_HIGH__N_LOW)                    /* CHxP ����Ч�� CHxN�͵�ƽ��Ч */
  MCPWM_InitStructure.CH0N_Polarity_INV = ENABLE;           /* CH0Nͨ������������� | ���������ȡ�����*/
  MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;          /* CH0Pͨ������������� | ���������ȡ����� */
  MCPWM_InitStructure.CH1N_Polarity_INV = ENABLE;
  MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH2N_Polarity_INV = ENABLE;
  MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

  MCPWM_InitStructure.Switch_CH0N_CH0P =  DISABLE;           /* ͨ������ѡ������ | CH0P��CH0N�Ƿ�ѡ���źŽ��� */
  MCPWM_InitStructure.Switch_CH1N_CH1P =  DISABLE;           /* ͨ������ѡ������ */
  MCPWM_InitStructure.Switch_CH2N_CH2P =  DISABLE;           /* ͨ������ѡ������ */

  /* Ĭ�ϵ�ƽ���� Ĭ�ϵ�ƽ�������MCPWM_IO01��MCPWM_IO23�� BIT0��BIT1��BIT8��BIT9��BIT6��BIT14
                                                   ͨ�������ͼ��Կ��Ƶ�Ӱ�죬ֱ�ӿ���ͨ����� */
  MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH0N_default_output = HIGH_LEVEL;
  MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;      /* CH1P��Ӧ�����ڿ���״̬����͵�ƽ */
  MCPWM_InitStructure.CH1N_default_output = HIGH_LEVEL;     /* CH1N��Ӧ�����ڿ���״̬����ߵ�ƽ */
  MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH2N_default_output = HIGH_LEVEL;
#else
#if (PRE_DRIVER_POLARITY == P_HIGH__N_HIGH)                    /* CHxP ����Ч�� CHxN�ߵ�ƽ��Ч */
  MCPWM_InitStructure.CH0N_Polarity_INV = DISABLE;           /* CH0Nͨ������������� | ���������ȡ�����*/
  MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;          /* CH0Pͨ������������� | ���������ȡ����� */
  MCPWM_InitStructure.CH1N_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH2N_Polarity_INV = DISABLE;
  MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;

  MCPWM_InitStructure.Switch_CH0N_CH0P =  DISABLE;           /* ͨ������ѡ������ | CH0P��CH0N�Ƿ�ѡ���źŽ��� */
  MCPWM_InitStructure.Switch_CH1N_CH1P =  DISABLE;           /* ͨ������ѡ������ */
  MCPWM_InitStructure.Switch_CH2N_CH2P =  DISABLE;           /* ͨ������ѡ������ */

  /* Ĭ�ϵ�ƽ���� Ĭ�ϵ�ƽ�������MCPWM_IO01��MCPWM_IO23�� BIT0��BIT1��BIT8��BIT9��BIT6��BIT14
                                                   ͨ�������ͼ��Կ��Ƶ�Ӱ�죬ֱ�ӿ���ͨ����� */
  MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH0N_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;      /* CH1P��Ӧ�����ڿ���״̬����͵�ƽ */
  MCPWM_InitStructure.CH1N_default_output = LOW_LEVEL;     /* CH1N��Ӧ�����ڿ���״̬����ߵ�ƽ */
  MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
  MCPWM_InitStructure.CH2N_default_output = LOW_LEVEL;
#endif
#endif

  MCPWM_InitStructure.DebugMode_PWM_out = DISABLE;           /* �ڽ��Ϸ�����debug����ʱ����ͣMCU����ʱ��ѡ���PWMͨ��������������ź�
                                                                 �������Ĭ�ϵ�ƽ�������������� ENABLE:������� DISABLE:���Ĭ�ϵ�ƽ*/
  MCPWM_InitStructure.MCPWM_Base0T0_UpdateEN = ENABLE;      /* MCPWM ʱ��0 T0�¼�����ʹ�� */
  MCPWM_InitStructure.MCPWM_Base0T1_UpdateEN = DISABLE;     /* MCPWM ʱ��0 T1�¼����� ��ֹ*/

  MCPWM_InitStructure.MCPWM_Base1T0_UpdateEN = ENABLE;      /* MCPWM ʱ��1 T0�¼�����ʹ�� */
  MCPWM_InitStructure.MCPWM_Base1T1_UpdateEN = DISABLE;     /* MCPWM ʱ��1 T1�¼����� ��ֹ*/

#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
  MCPWM_InitStructure.CNT0_T1_Update_INT_EN = ENABLE;           /* T0�����¼� �ж�ʹ�ܻ�ر� */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
  MCPWM_InitStructure.CNT0_T0_Update_INT_EN = DISABLE;           /* T0�����¼� �ж�ʹ�ܻ�ر� */
#else
#if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)
  MCPWM_InitStructure.CNT0_T0_Update_INT_EN = DISABLE;           /* T0�����¼� �ж�ʹ�ܻ�ر� */
#endif
#endif
#endif

  MCPWM_InitStructure.FAIL0_INT_EN = DISABLE;               /* FAIL0�¼� �ж�ʹ�ܻ�ر� */
  MCPWM_InitStructure.FAIL1_INT_EN = ENABLE;               /* FAIL1�¼� �ж�ʹ�ܻ�ر� */
  MCPWM_InitStructure.FAIL1_INPUT_EN = ENABLE;             /* FAIL1ͨ����ͣ���ܴ򿪻�ر� */
  MCPWM_InitStructure.FAIL1_Signal_Sel = FAIL_SEL_CMP;      /* FAIL1�¼��ź�ѡ�񣬱Ƚ�����IO�� */
  MCPWM_InitStructure.FAIL1_Polarity = HIGH_LEVEL_VALID;           /* FAIL1�¼�����ѡ�񣬸���Ч�����Ч */

#ifdef MCPWM_SWAP_FUNCTION      /* ʹ��054D, 057D����Ԥ��оƬ��Ҫ��PWM�������� */
  MCPWM_PRT = 0x0000DEAD; /*enter password to unlock write protection */
  MCPWM_SWAP = 0x67;
#endif
  MCPWM_Init(MCPWM0, &MCPWM_InitStructure);                 /* MCPWM0 ģ���ʼ�� */

  mIPD_CtrProc.hDriverPolarity = MCPWM_IO01;                /* ������������ */
}




/*******************************************************************************
 �������ƣ�    void seekPosADCsetting(void)
 ����������    ����IPD����ǰ������ADC����ģʽ��Ϊ��ʼλ�ü����׼��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2017/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void seekPosADCsetting(void)
{

  ADC_STATE_RESET();         /*״̬����λ��idle״̬����ɺ��Զ���0 */
  MCPWM_TMR0 = PWM_PERIOD;   /*ADC����ʱ��0����Ϊ���Źض����ĵ� */
  MCPWM_TMR1 = (-600);  /*ADC����ʱ��1����ΪPWM���ĵ����ƫ600��Clock*/
}

/*******************************************************************************
 �������ƣ�    void DebugPWM_OutputFunction(void)
 ����������    PWM������ܵ���   ���25%ռ�ձ�
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2017/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void DebugPWM_OutputFunction(void)
{
  MCPWM_TH00 = (-(PWM_PERIOD >> 2));
  MCPWM_TH01 = (PWM_PERIOD >> 2);
  MCPWM_TH10 = (-(PWM_PERIOD >> 2));
  MCPWM_TH11 = (PWM_PERIOD >> 2);
  MCPWM_TH20 = (-(PWM_PERIOD >> 2));
  MCPWM_TH21 = (PWM_PERIOD >> 2);

  PWMOutputs(MCPWM0, ENABLE);
  while(1)
  {
  }
}



