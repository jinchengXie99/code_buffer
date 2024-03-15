/*******************************************************************************
 * 版权所有 (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * 文件名称： MC_hall.c
 * 文件标识：
 * 内容摘要： Hall信号处理
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Howlet Li
 * 完成日期： 2020年8月16日
 *
 * 修改记录1：
 * 修改日期：2020年8月16日
 * 版 本 号：V 1.0
 * 修 改 人：Howlet Li
 * 修改内容：创建
 *
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Global_Variable.h"
#include "MC_hall.h"
#include "hardware_config.h"
#include "mc_type.h"
#include "time_process.h"

Stru_HallParaConstDef struHallParamaConst =
{   /* Hall角速度计算，电角度拟合相关常量定义 */
    ROTOR_SPEED_FACTOR,
    (u32)(ROTOR_SPEED_FACTOR / 3),
    ROTOR_SPEED_FACTOR_SAT,
    ROTOR_SPEED_FACTOR_PER_UNIT,
    ROTOR_SPEED_FACTOR_PER_UNIT1,
    ROTOR_FACTOR_REDUCE_PER_UNIT,
    ROTOR_FACTOR_ECC_UNIT,

    LOW_CLOSE_SPEED_THD,
    LOW_CLOSE_SPEED_THD1,
    LOW_CLOSE_SPEED_THD2,

    PWM_PERIOD,

    PWM_TIME_500uS,
    PWM_TIME_1MS,
    PWM_TIME_2MS,
    PWM_TIME_1MS,
    PWM_TIME_20MS,

    HALL_LEARN_ADDR

};

void hall_comm_VariableInit(stru_HallProcessDef *this);

/*******************************************************************************
 函数名称：    void HALL_Init_Electrical_Angle(stru_HallProcessDef *this)
 功能描述：    Hall变化时，初始化当前电角度
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void HALL_Init_Electrical_Angle(stru_HallProcessDef *this)
{

    if(this->bHallRunFlg & HALL_COM_ERR)
    {   /* 解决堵转抖动180413 HL */
        this->bCloseLoopCnt = 0;
        this->bCloseLoopAngleFlg = 0;
        this->nHallChangeCnt = 0;

        this->nPhaseShift = this->nPhaseShiftOffset;
//      PID_Flux.hKi_Gain = PID_FLUX_KI_DEFAULT;
    }


    if((this->wMotorSpeedAvgCnt < this->wCloseLoopPeriodThd))
    {   /* 速度大于闭环门限，开通闭环控制 */
        if(this->bCloseLoopCnt < this->bOpenStartUpCntThd)
        {
            this->bCloseLoopCnt++;
        }
        else
        {
            if(this->bCloseLoopCnt < 20)
            {
                this->bCloseLoopCnt ++;
            }
            //开通闭环符号
            this->bCloseLoopAngleFlg = 1;

            if(this->bHallRunFlg & HALL_DIR_FLG)
            {
                this->nPhaseShift = this->nPhaseShiftOffset - S16_30_PHASE_SHIFT;

                if(this->bMotorDirtionCtrl)
                {
                    this->bCloseLoopCnt = 0;
                    this->bCloseLoopAngleFlg = 0;
                    this->bMotorDirtionCtrl = 0;
                }

            }
            else
            {   /* 电机反转命令 */
                this->nPhaseShift = this->nPhaseShiftOffset + S16_30_PHASE_SHIFT;
                if(this->bMotorDirtionCtrl == 0)
                {
                    this->bCloseLoopCnt = 0;
                    this->bCloseLoopAngleFlg = 0;
                    this->bMotorDirtionCtrl = 1;
                }


            }

        }
    }
    else
    {



        /* 速度小于开环门限，开通开环控制 */
        if(this->bCloseLoopAngleFlg)
        {
            this->nHallChangeCnt = 0;
        }
        this->bCloseLoopAngleFlg = 0;
        this->bCloseLoopCnt = 0;

        this->nPhaseShift = this->nPhaseShiftOffset;


    }

    /* 查表得到当前Hall位置下的角度值 */
    GetHall_edgeAngle(this);

    //hall状态发生改变
    if(this->bOldHallState != this->bHallState)
    {   /* 电机运行方向标志，0:正转 1:反转 */
        if(this->bHallRunFlg & HALL_DIR_FLG)
        {
            if(this->nHallQEP_Cnt < 15)
            {
                this->nHallQEP_Cnt ++;
            }
            //期望控制的电机运行方向
            if(this->bMotorDirtionCtrl == 0)
            {
                this->nHallChangeCnt ++;
            }
            else
            {
                this->nHallChangeCnt = 0;
            }
            //增加速度符号 速度的符号标志位置1;
            this-> bHallSpeedDirFlag = 1;
        }
        else
        {
            if(this->nHallQEP_Cnt > -15)
            {
                this->nHallQEP_Cnt --;
            }

            if(this->bMotorDirtionCtrl)
            {
                this->nHallChangeCnt ++;
            }
            else
            {
                this->nHallChangeCnt = 0;
            }
            //增加速度符号 速度的符号标志位置-1;
            this->bHallSpeedDirFlag = (-1);
        }
        this->bOldHallState = this->bHallState;


    }

    Angle_Init_Process(this);

    //低通滤波
    this->nRotorFreqDppFir = lowPass_filter(&this->stru_HallDppRC, this->nRotorFreqDpp);

    //转化成速度闭环反馈的物理量 原始根据实际速度标
    //hall反馈靠近 Sensor less反馈 0.194161*1.25051=0.2428012871536524    *符号位
    this->wSpeedfbkHall = App2CoreFreqTrans(User2AppFreqTrans(this->nRotorFreqDppFir * 0.2428)) * (this->bHallSpeedDirFlag);
    //    hall_change_flg = 1;
}




/*******************************************************************************
 函数名称：    void HALL_Init_Electrical_Angle(stru_HallProcessDef *this)
 功能描述：    Hall相关变量初始化
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void HALL_InitHallMeasure(stru_HallProcessDef *this)
{
    u8 t_i;

    this->nReduceToqueDpp = 0;

    this->bSpeedFIFO_Index = HALL_SPEED_FIFO_SIZE - 1;

    this->bHallRunFlg |= HALL_COM_ERR;

    for(t_i = 0; t_i < HALL_SPEED_FIFO_SIZE; t_i++)
    {
        this->wSensorPeriod[this->bSpeedFIFO_Index] = ROTOR_SPEED_FACTOR / 3;
    }

    this->wMotorSpeedAvgCnt = ROTOR_SPEED_FACTOR / 3;
    HALL_Init_Electrical_Angle(this);
    this->bHallRunFlg &= ~HALL_COM_ERR;

    //给定了hall的类型和角度
    this->nHallOffsetArry[0] = S16_60_PHASE_SHIFT;
    this->nHallOffsetArry[1] = S16_120_PHASE_SHIFT;
    this->nHallOffsetArry[2] = S16_180_PHASE_SHIFT;
    this->nHallOffsetArry[3] = S16_240_PHASE_SHIFT;
    this->nHallOffsetArry[4] = S16_300_PHASE_SHIFT;
    this->nHallOffsetArry[5] = 0;


    this->bHallCommTab[0] = 3; //--1(30°)
    this->bHallCommTab[1] = 1; //--5(90°)
    this->bHallCommTab[2] = 5; //--4(150°)
    this->bHallCommTab[3] = 4; //--6(210°)
    this->bHallCommTab[4] = 6; //--2(270°)
    this->bHallCommTab[5] = 2; //--3(330°)

    this->bHallCommTab[0] = 3; //--1(30°)
    this->bHallCommTab[1] = 2; //--5(90°)
    this->bHallCommTab[2] = 6; //--4(150°)
    this->bHallCommTab[3] = 4; //--6(210°)
    this->bHallCommTab[4] = 5; //--2(270°)
    this->bHallCommTab[5] = 1; //--3(330°)
    
    this->bHallCommTab[0] = 6; //--1(30°)
    this->bHallCommTab[1] = 2; //--5(90°)
    this->bHallCommTab[2] = 3; //--4(150°)
    this->bHallCommTab[3] = 1; //--6(210°)
    this->bHallCommTab[4] = 5; //--2(270°)
    this->bHallCommTab[5] = 4; //--3(330°)
    
    
    this->bHallCommTab[0] = 4; //--1(30°)
    this->bHallCommTab[1] = 6; //--5(90°)
    this->bHallCommTab[2] = 2; //--4(150°)
    this->bHallCommTab[3] = 3; //--6(210°)
    this->bHallCommTab[4] = 1; //--2(270°)
    this->bHallCommTab[5] = 5; //--3(330°)


    hall_comm_VariableInit(this);
    this->pStru_HallParama = &struHallParamaConst;      /* 结构体指针初始化 */
    this->bHallType = 120;                               /* Hall类型为120度Hall; 120 */
    this->nPhaseShiftOffset = S16_270_PHASE_SHIFT;      //S16_315_PHASE_SHIFT      /* 当前Hall角度偏移值设置 */
    this->bOpenStartUpCntThd = 6;                        /* 开环状态运行转闭环工作门限值 */
    this->wCloseLoopPeriodThd = HALL_PLL_OPEN2CLOSE_THD; /* HALL PLL 开环切闭环速度门限值 */
    this->wOpenLoopPeriodThd  = HALL_PLL_CLOSE2OPEN_THD; /* HALL PLL 闭环切开环速度门限值 */
    this->stru_HallDppRC.coef = FRAC16(0.2);             /* Hall角速度低通滤波FRAC16(0.2) */
    this->stru_HallDppRC.yk_1=0;                         /* Hall角速度低通滤波FRAC16(0.2) */
}



/*******************************************************************************
 函数名称：    void HALL_GetElectricalAngle(stru_HallProcessDef *this)
 功能描述：    Hall变化时，初始化当前电角度
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    返回当前转子电气角度位置
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
__inline s16 HALL_GetElectricalAngle(stru_HallProcessDef *this)
{
    return(this->nElectrical_Angle);
}


/*******************************************************************************
 函数名称：    void HALL_IRQProcess()
 功能描述：    Hall中断处理函数
 输入参数：    HALL_TypeDef *HALLx : Hall模块HALL0或HALL1
               stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void HALL_IRQProcess(HALL_TypeDef *HALLx, stru_HallProcessDef *this)
{
    unsigned char temp_state, t_FuncState;
    volatile u8 t_i, t_cnt;

    if(HALLx->INFO & BIT16)
    {   /* Hall 变化中断 */
        this->wHallCapCnt += HALLx->WIDTH;  /* 累计捕捉到的HALL周期值 */
        HALLx->INFO = BIT16;

        temp_state = ReadHallCapState(HALLx);

        t_FuncState = check_hall_state(temp_state, this);

        if(t_FuncState)
        {
            if (this->bSpeedFIFO_Index != HALL_SPEED_FIFO_SIZE - 1)
            {   /* FIFO 指针递增 */
                this->bSpeedFIFO_Index++;
            }
            else
            {
                this->bSpeedFIFO_Index = 0;
            }

            if(this->bFstHallSigFlg)
            {
                this->bFstHallSigFlg = 0;
            }
            else
            {
                u32 t_CapCnt;

                if(this->stru_HallRepare.bBrokenHallFlg)
                {   /* Hall正常的时候读寄存器，不正常用PWM计数 */
                    t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

                    if(t_CapCnt < MAX_SPEED_DPP)
                    {
                        t_CapCnt = MAX_SPEED_DPP;
                    }

                    this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;
                }
                else
                {


                    if(this->wHallCapCnt < MAX_SPEED_DPP)
                    {
                        this->wHallCapCnt = MAX_SPEED_DPP;
                    }

                    this->wSensorPeriod[this->bSpeedFIFO_Index] = this->wHallCapCnt;
                }
            }

            this->wHallCapCnt = 0;
            this->wHallPWMTimerCnt = 0;

            this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);

            if(this->bHallLearnFlg == 0)
            {   /* 学习状态不更新角度 */
                HALL_Init_Electrical_Angle(this);
            }
        }

    }

    if(HALLx->INFO & BIT17)
    {   /* Hall超时中断 */

        HALLx->INFO = BIT17;

        this->bHallRunFlg |= HALL_COM_ERR;
        this->bMotorDirtionCtrl = 0;

        this->bSpeedFIFO_Index = 0;

        for(t_i = 0; t_i < HALL_SPEED_FIFO_SIZE; t_i++)
        {   /* Hall超时，初始化FIFO值为最大周期值 */
            this->wSensorPeriod[this->bSpeedFIFO_Index] = ROTOR_SPEED_FACTOR / 3;
        }
        this->wMotorSpeedAvgCnt = ROTOR_SPEED_FACTOR / 3;

        temp_state = ReadHallState(HALLx);

        t_FuncState = check_hall_state(temp_state, this);

        if(this->bHallLearnFlg == 0)
        {   /* 学习状态不更新角度 */
            HALL_Init_Electrical_Angle(this);
        }
    }

}

/*******************************************************************************
 函数名称：    void GetAvrgHallPeriod()
 功能描述：    计算Hall平均周期变化值
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    Hall平均周期变化值，单位值为HALL模块主计数时钟
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
static __inline u32 GetAvrgHallPeriod(stru_HallProcessDef *this)
{
    u32 wAvrgBuffer;
    u8 wIndex;


    wAvrgBuffer = 0;

    if(this->bCloseLoopCnt >= 6)
    {

        for ( wIndex = 0; wIndex < HALL_SPEED_FIFO_SIZE; wIndex++ )
        {
            wAvrgBuffer += this->wSensorPeriod[wIndex];
        }

        wAvrgBuffer /= HALL_SPEED_FIFO_SIZE;  /* Average value */
    }
    else
    {
        wAvrgBuffer = (this->wOldSensorPeriod + this->wSensorPeriod[this->bSpeedFIFO_Index]) >> 1;
        if(wAvrgBuffer < ROTOR_SPEED_FACTOR / 2000)
        {
            wAvrgBuffer = ROTOR_SPEED_FACTOR / 3;
        }
    }
    this->wOldSensorPeriod = this->wSensorPeriod[this->bSpeedFIFO_Index];


    return (wAvrgBuffer);
}

/*******************************************************************************
 函数名称：    void Angle_Init_Process(stru_HallProcessDef *this)
 功能描述：    电角度初始化，Pll工作时，直接查表等到初始角度。Reduce toque模式下
               给定角度dpp值
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
static __inline void Angle_Init_Process(stru_HallProcessDef *this)
{
    if(this->bCloseLoopAngleFlg)
    {
        if(this->wMotorSpeedAvgCnt > this->wOpenLoopPeriodThd)
        {
            this->nEndElectricalAngle = (s16)(this->nTabElectrical_Angle - this->nElectrical_Angle);
            this->bFastAddElecAngleFlg = 1;
        }

        closeLoopAnglePLLInit(this->nTabElectrical_Angle, this);
        this->nMaxIncAngle = 0;
    }
    else
    {
        u16 t_RotorFreq_dpp;

        this->nRotorFreqDpp = ROTOR_SPEED_FACTOR / this->wMotorSpeedAvgCnt;

//此处限制了反方向的转速
//        if(this->nRotorFreqDpp > 100)
//        {
//            this->nRotorFreqDpp = 100;
//        }

        if(this->bHallRunFlg & HALL_COM_ERR)
        {   /* 换相错误，直接给出角度 */

            if(this->bHallRunFlg & HALL_DIR_FLG)
            {
                this->nEndElectricalAngle = (s16)(this->nTabElectrical_Angle - this->nElectrical_Angle);
                this->bFastAddElecAngleFlg = 1;
            }
            else
            {
                this->nElectrical_Angle = this->nTabElectrical_Angle;
            }

            this->bReduceToqueFlg = 0;



        }
        else
        {   /* 正常换相时，给出最终换相角 */
            calc_first_ElectAngle(this->nTabElectrical_Angle, this);
            
            t_RotorFreq_dpp = (u32)(ROTOR_FACTOR_REDUCE_PER_UNIT * this->nMaxReduceIncAngle)
                                      / this->wSensorPeriod[this->bSpeedFIFO_Index];

            if(t_RotorFreq_dpp < 10)
            {
                this->nReduceToqueDpp = 10;
            }

            else
            {
                this->nReduceToqueDpp = t_RotorFreq_dpp;
                if(this->nReduceToqueDpp > 200)
                {
                    this->nReduceToqueDpp = 200;
                }
            }
        }
    }
}

/*******************************************************************************
 函数名称：    void Verify_Hall_State(stru_HallProcessDef *this)
 功能描述：    验证当前Hall是否有效，当Hall中断检测到干扰值时，冗余检测出Hall值
               当损坏一个Hall的时候执行Hall损坏修复
 输入参数：    HALL_TypeDef *HALLx : Hall模块HALL0或HALL1
               stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：
 多任务访问:   不涉及局部静态变量，可重入函数
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void Verify_Hall_State(HALL_TypeDef *HALLx, stru_HallProcessDef *this)
{
    u8 temp_state1, temp_state;
    u8 t_cnt, t_i;
    u8 t_funcState;
    u32 t_CapCnt;
  

    //读取当前hall状态
    temp_state = ReadHallState(HALLx);

    if(this->bCloseLoopAngleFlg == 0)
    {   /* 如果当前Hall 锁相环工作模式 */
        if(this->eMotorState == RUN)
        {
            if(this->stru_HallRepare.nFocusCommCnt < PWM_TIME_20MS * 2)
            {
                this->stru_HallRepare.nFocusCommCnt ++;
            }

            if(this->stru_HallRepare.nFocusCommCnt == PWM_TIME_20MS)
            {   /* 20mS检测不到Hall变化，强制初始化角度信息 */
                this->stru_HallRepare.bHallState_org = temp_state;

                this->bHallState = this->bHallPossableTab[this->bHallState];

                if (this->bSpeedFIFO_Index < (HALL_SPEED_FIFO_SIZE - 1))
                {
                    this->bSpeedFIFO_Index++;
                }
                else
                {
                    this->bSpeedFIFO_Index = 0;
                }

                t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

                this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

                //
                this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);

                this->wHallPWMTimerCnt = 0;

                //霍尔角度初始化
                HALL_Init_Electrical_Angle(this);

            }
        }
    }

    if(this->stru_HallRepare.bNeed_DelayComm)
    {   /* Hall修复状态，如果需要延时换相 */
        if(this->wHallPWMTimerCnt > this->stru_HallRepare.bDelay_timeCnt)
        {   /* 延时到需要换相的时刻，初始化Hall值，并初始化角度信息　*/
            this->stru_HallRepare.bNeed_DelayComm = 0;

            this->bHallState = this->bHallPossableTab[this->bHallState];

            if (this->bSpeedFIFO_Index < (HALL_SPEED_FIFO_SIZE - 1))
            {
                this->bSpeedFIFO_Index++;
            }
            else
            {
                this->bSpeedFIFO_Index = 0;
            }

            t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

            this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

            this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);

            this->wHallPWMTimerCnt = 0;
            //初始化角度信息
            HALL_Init_Electrical_Angle(this);

        }
    }
    else
    {   //不需要延时换相
        t_cnt = 0;
        if(temp_state != this->stru_HallRepare.bHallState_org)
        {   /* 检测到Hall状态和当前状态不一致, 冗余检测Hall状态 */
            for (t_i = 0; t_i < 4; t_i++)
            {
                temp_state1 = ReadHallState(HALLx); /* 读霍尔状态 */

                if (temp_state == temp_state1)
                {
                    t_cnt++;
                }
            }

            if (t_cnt > 2)
            {
                //关闭hall中断
                NVIC_DisableIRQ (HALL_IRQn);

                t_funcState = check_hall_state(temp_state, this); /* 查询霍尔状态函数 */
                if (t_funcState)
                {
                    if (this->bSpeedFIFO_Index < (HALL_SPEED_FIFO_SIZE - 1))
                    {
                        this->bSpeedFIFO_Index++;
                    }
                    else
                    {
                        this->bSpeedFIFO_Index = 0;
                    }

                    t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

                    this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

                    this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);
                    this->wHallPWMTimerCnt = 0;
                    if(this->bHallLearnFlg == 0)
                    {
                    HALL_Init_Electrical_Angle(this);
                    }
                }
                //使能hall中断
                NVIC_EnableIRQ (HALL_IRQn);
            }
        }
    }

  
}

/*******************************************************************************
 函数名称：    hall_comm_VariableInit(void)
 功能描述：    HALL变量初始化, 更新Hall可能的换相表
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/15      V1.0           Howlet Li          创建
 *******************************************************************************/
void hall_comm_VariableInit(stru_HallProcessDef *this)
{
    u8 t_i, t_j;
    if (this->bHallType == 60)
    {
        for (t_i = 0; t_i < 8; t_i++)
        {
            this->bHallPossableTab[t_i] = 0;
            for (t_j = 0; t_j < 8; t_j++)
            {
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j != 5)
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[t_j + 1];
                    }
                    else
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[0];
                    }
                    break;
                }
            }

            for (t_j = 0; t_j < 8; t_j++)
            {
                this->bHallPossableTab2[t_i] = 0;
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j == 0)
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[5];
                    }
                    else
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[t_j - 1];
                    }
                    break;
                }
            }
        }
    }
    else
    {
        for (t_i = 1; t_i < 7; t_i++)
        {
            for (t_j = 0; t_j < 6; t_j++)
            {
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j != 5)
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[t_j + 1];
                    }
                    else
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[0];
                    }
                    break;
                }
            }

            for (t_j = 0; t_j < 6; t_j++)
            {
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j == 0)
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[5];
                    }
                    else
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[t_j - 1];
                    }
                    break;
                }
            }
        }
    }

    /* 找到Hall 1位置，判断Hall 相序的正反，为单Hall修复服务 171008 HL*/
    for (t_i = 0; t_i < 6; t_i++)
    {
        if(this->bHallCommTab[t_i] == 1)
        {
            break;
        }
    }

    if(t_i == 5)
    {
        if(this->bHallCommTab[0] == 3)
        {
            this->stru_HallRepare.bRepareDirect = 0;
        }
        else
        {
            this->stru_HallRepare.bRepareDirect = 1;
        }
    }
    else
    {
        if(this->bHallCommTab[t_i + 1] == 3)
        {
            this->stru_HallRepare.bRepareDirect = 0;
        }
        else
        {
            this->stru_HallRepare.bRepareDirect = 1;
        }
    }
}







/*******************************************************************************
 函数名称：    Hall_ElecAnglePWM_Process(void)
 功能描述：    Hall角度PWM中断周期处理
               HALL异常处理，异常发生时，快速累加电角度值
 输入参数：    stru_HallProcessDef *this : Hall处理结构体
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/15      V1.0           Howlet Li          创建
 *******************************************************************************/
void Hall_ElecAnglePWM_Process(stru_HallProcessDef *this)
{
    this->wHallPWMTimerCnt++;       /* Hall PWM时基计数 */
    if(this->bFastAddElecAngleFlg)
    {
        u16 t_addAngleStep;

        t_addAngleStep = HALL_FAST_ADD_DPP;

        if(this->stru_HallRepare.bBrokenHallFlg)
        {
            t_addAngleStep = HALL_FAST_ADD_DPP / 2;
        }

        if(this->nEndElectricalAngle > 0)
        {
            if(this->nEndElectricalAngle > t_addAngleStep)
            {
                this->nEndElectricalAngle -= t_addAngleStep;
                this->nElectrical_Angle += t_addAngleStep;
            }
            else
            {
                this->bFastAddElecAngleFlg = 0;
            }
        }
        else
        {
            if(this->nEndElectricalAngle < -t_addAngleStep)
            {
                this->nEndElectricalAngle += t_addAngleStep;
                this->nElectrical_Angle -= t_addAngleStep;
            }
            else
            {
                this->bFastAddElecAngleFlg = 0;
            }
        }
    }

    //hall 角度的两种处理
    if (this->bCloseLoopAngleFlg)
    {
        closeLoopAnglePLL(this);
    }
    else
    {
        reduceToqueAnglePll(this);
    }

}

/*******************************************************************************
 函数名称：    Hall_learn_Process(void)
 功能描述：    Hall角度自学习
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/15      V1.0           Howlet Li          创建
 *******************************************************************************/
void Hall_learn_Process(void)
{
    volatile u8 t_i;


    if (struHallProcess.bHallLearnFlg == 2)
    {
        struHallProcess.bHallLearnFlg = 1;
        struHallProcess.bHallLearnStateFlg = 0;
        struHallProcess.nLearnHallStep = 0;
        struHallProcess.nLearnHallTimeCnt = 0;
        struHallProcess.nElectrical_Angle = 0;
        struHallProcess.bHallCommTab[6] = 0;

        for(t_i = 0; t_i < 6; t_i++)
        {
            struHallProcess.bHallCommTab[t_i] = 0;
        }

        struFOC_CtrProc.bMC_RunFlg = 1;
        struFOC_CurrLoop.nQCur_Reference = App2CoreCurTrans(User2AppCurTrans(HALL_LEARN_CURRENT_SETTINT));;
        struFOC_CurrLoop.nDCur_Reference = 0;
    }

    if (struHallProcess.bHallLearnFlg == 1)
    {
        struFOC_CurrLoop.nQCur_Reference = App2CoreCurTrans(User2AppCurTrans(HALL_LEARN_CURRENT_SETTINT));; /* 学习电流， 值越大电流越大 */
        struFOC_CurrLoop.nDCur_Reference = 0;
        learn_hall_proc(&struHallProcess);
    }
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
