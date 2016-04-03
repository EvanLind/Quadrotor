//@filename： RCS_pwm.C
//@date:2012-08-18
//@author:黄东润
//@brief:pwm 方波输出控制
//计算公式
//  TimClk = (uint32_t ) (rcc_clocks.SYSCLK_Frequency / 2);TimClk 频率等于PCLK2的频率
//  PrescalerValue = (uuint32_t16_t) (TimClk / _CLKHZ ) - 1; 计数器频率由分频控制
//  ARR = (uint32_t)(_CLKHZ / _PWMHZ); ARR 控制输出频率
//  CCRValue = (uint32_t)(_percent * ARR / 100); CCR 控制占分比
//
#include "RCS_Pwm.h"




//@name: PWMInit,周期必须小于65535
//@brief: 启动时钟中断
//@param:TIM_TypeDef * _TIM ： 定时器号
//@param:uint8_t _ch  ：定时器 pwm输出 通道号
//@param:GPIO_TypeDef* _port ：pwmm输出管脚
//@param:uint32_t _pin : pwm输出针脚
//@param:uint32_t _CLKHZ : 定时器计数器频率，必须为能被计时器时钟频率（84MHZ）整除
//@param:uint32_t _PWMHZ : pwm输出频率！！！！！！！！！！！！！！对！！！！！！！！！！！
//@retval: 初始化成功与否
Boolean PWMInit(TIM_TypeDef *_TIM, uint8_t _ch,
                GPIO_TypeDef   *_port, uint32_t _pin,
                uint32_t _CLKHZ, uint32_t _PWMHZ)
{
    assert_param(IS_TIM_ALL_PERIPH(_TIM));
    assert_param(IS_GPIO_ALL_PERIPH(_port));
    assert_param(IS_GPIO_PIN(_pin));
    if (_TIM == TIM6 || _TIM == TIM7)
    {
        return FALSE;
    }
    RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);

    if (_TIM == TIM1 ||  _TIM == TIM8 || _TIM == TIM9 || _TIM == TIM10 || _TIM == TIM11 )
    {
        RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_TIM(_TIM), ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_TIM(_TIM), ENABLE);
    }
    GPIO_PinAFConfig(_port, GetRCS_GPIO_PinSource(_pin), GetRCS_GPIO_AF_TIM(_TIM));
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = _pin ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(_port, &GPIO_InitStructure);

    GPIO_PinAFConfig(_port, GetRCS_GPIO_PinSource(_pin) , GetRCS_GPIO_AF_TIM(_TIM));

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_ClocksTypeDef rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);

    uint32_t TimClk = (uint32_t ) (rcc_clocks.SYSCLK_Frequency / 2);
    if (_TIM == TIM1 ||  _TIM == TIM8 || _TIM == TIM9 || _TIM == TIM10 || _TIM == TIM11 )
    {
        TimClk = (uint32_t ) (rcc_clocks.SYSCLK_Frequency);
    }
    uint32_t PrescalerValue = (uint16_t) (TimClk / _CLKHZ) - 1;
    uint32_t ARR = (uint32_t)(_CLKHZ / _PWMHZ);
    assert( ARR <= 65536 );
    uint32_t CCRValue = (uint32_t)( 0.5 * ARR );

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ARR - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = CCRValue;
    if (_TIM == TIM1 || _TIM == TIM8)
    {
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    }
    //选择输出通道，不同通道对应着不同的管脚
    if (_ch == 1)
    {
        TIM_OC1Init(_TIM, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(_TIM, TIM_OCPreload_Enable);
    }
    else if (_ch == 2)
    {
        TIM_OC2Init(_TIM, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(_TIM, TIM_OCPreload_Enable);
    }
    else if (_ch == 3)
    {
        TIM_OC3Init(_TIM, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(_TIM, TIM_OCPreload_Enable);
    }
    else if (_ch == 4)
    {
        TIM_OC4Init(_TIM, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(_TIM, TIM_OCPreload_Enable);
    }

    //预装载使能
    TIM_ARRPreloadConfig(_TIM, ENABLE);
    /* TIMx 计数器使能 */
    TIM_Cmd(_TIM, ENABLE);
    if (_TIM == TIM1 ||  _TIM == TIM8)
    {
        TIM_CtrlPWMOutputs(_TIM, ENABLE);
    }

    return TRUE;
}


uint8_t Measure_ch;
double frequency, DutyCycle;


//@name: PWMGetIn_Period_Duty
//@brief: 测量输入pwm的频率和占空比
//@param:TIM_TypeDef * _TIM ： 定时器号
//@param:uint8_t _ch  ：定时器 pwm输入通道号
//@param:GPIO_TypeDef* _port ：pwmm输入管脚
//@param:uint32_t _pin : 输入
//@retval: 初始化成功与否
static void Measure_IRQHandler(void)
{
    double DutyCycle = 0, frequency = 0;
    uint32_t IC2Value = 0;
    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
    {
        IC2Value = TIM_GetCapture2(TIM3);              //读取IC2捕获寄存器的值，即为PWM周期的计数值
        if (IC2Value != 0)
        {
            DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value; //读取IC1捕获寄存器的值，并计算占空比
            frequency = 72000000 / IC2Value;      //计算PWM频率。
        }
        else
        {
            DutyCycle = 0;
            frequency = 0;
        }
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);       //清楚TIM的中断待处理位
}
Boolean PWMGetIn_Period_Duty(TIM_TypeDef *_TIM, uint8_t _ch,
                             GPIO_TypeDef   *_port, uint32_t _pin, uint8_t _pri)
{
    Measure_ch = _ch;
    assert_param(IS_TIM_ALL_PERIPH(_TIM));
    if (_TIM == TIM6 || _TIM == TIM7 || _TIM == TIM9 || _TIM == TIM10 || _TIM == TIM11 || _TIM == TIM12 || _TIM == TIM13 || _TIM == TIM14)
    {
        return FALSE;
    }
    RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);

    if (_TIM == TIM1 ||  _TIM == TIM8 || _TIM == TIM9 || _TIM == TIM10 || _TIM == TIM11 )
    {
        RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_TIM(_TIM), ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_TIM(_TIM), ENABLE);
    }
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = _pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(_port, &GPIO_InitStructure);

    NVIC_InitTypeDef nvic_initStruct;
    BSP_IntVectSet(GetBSP_INT_ID_TIM(_TIM), Measure_IRQHandler);
    nvic_initStruct.NVIC_IRQChannel = GetRCS_TIM_IRQn(_TIM);
    nvic_initStruct.NVIC_IRQChannelPreemptionPriority = (_pri >> 4 ) & 0x0f;
    nvic_initStruct.NVIC_IRQChannelSubPriority = _pri & 0x0f;
    nvic_initStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_initStruct);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = _ch;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;       //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF

    TIM_PWMIConfig(_TIM, &TIM_ICInitStructure);
    TIM_SelectInputTrigger(_TIM, TIM_TS_TI2FP2);                //选择IC2为始终触发源
    TIM_SelectSlaveMode(_TIM, TIM_SlaveMode_Reset);
    //TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
    TIM_SelectMasterSlaveMode(_TIM, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发
    TIM_Cmd(_TIM, ENABLE);
    TIM_ITConfig(_TIM, TIM_IT_CC2, ENABLE);     //打开中断
    return TRUE;
}



//@name: PWMInit,周期必须小于65535
//@brief: 启动时钟中断
//@param:TIM_TypeDef * _PwmTIM ： 定时器号
//@param:uint8_t _ch  ：定时器 pwm输出 通道号
//@param:GPIO_TypeDef* _port ：pwmm输出管脚
//@param:uint32_t _pin : pwm输出针脚
//@param:uint32_t _CLKHZ : 定时器计数器频率，必须为能被计时器时钟频率（84MHZ）整除
//@param:uint32_t _PWMHZ : pwm输出频率
//@retval: 初始化成功与否
static float PulsePara;//一个单位对应的脉冲数
static float PWMPara;
static TIM_TypeDef *PwmTIM;
static TIM_TypeDef *PwmTimingTIM;
static uint8_t PWMch;
static GPIO_TypeDef   *DirPort;
static uint32_t DirPin;

double TimingCnt = 0;//步进电机每次转的计数
static void StepMotorIsr()
{
    TIM_ClearITPendingBit(PwmTimingTIM, TIM_IT_Update);
    static double cnt = 0;
    if (++cnt >= TimingCnt)
    {
        cnt = 0;
        PWMOutput(PwmTIM, PWMch, 0);
        TIM_Cmd(PwmTimingTIM, DISABLE);
        TIM_ITConfig(PwmTimingTIM, TIM_IT_Update, DISABLE);
    }
}
void StepMotorInit(TIM_TypeDef *_PwmTIM, uint8_t _ch,
                   GPIO_TypeDef   *_pwmport, uint32_t _pwmpin,
                   GPIO_TypeDef   *_dirport, uint32_t _dirpin,
                   TIM_TypeDef *_TIM2Timing, uint8_t _priority, float _CirPulse, double _para)
{
    PwmTIM = _PwmTIM;
    PWMch = _ch; 
    DirPort = _dirport;
    DirPin = _dirpin;
    PwmTimingTIM = _TIM2Timing;
    InitTimerInt(_TIM2Timing, 100, 100000,
                 StepMotorIsr,  _priority);//一毫秒
    TIM_Cmd(_TIM2Timing, DISABLE);
    TIM_ITConfig(PwmTimingTIM, TIM_IT_Update, DISABLE);


    PWMInit(_PwmTIM, _ch, _pwmport, _pwmpin, 42000000, 10000);
    SetGpioOutput(_dirport, _dirpin);
    PWMOutput(_PwmTIM, _ch, 0);
    PulsePara = _CirPulse / _para;
}
//_distPara:输入参数，单位和 _para 一样
//_timInterv：单位 ms
#define STEP_DIR 1//1 or -1
void StepMotorOutPut(TIM_TypeDef *_PwmTIM, uint8_t _ch, double _distPara, double _timInterv)
{
    TimingCnt = _timInterv;
    if (_distPara * STEP_DIR > 0)
        GPIO_SetBits(DirPort, DirPin);
    else
        GPIO_ResetBits(DirPort, DirPin);
    TIM_Cmd(PwmTimingTIM, ENABLE);
    TIM_ITConfig(PwmTimingTIM, TIM_IT_Update, ENABLE);

    _distPara = fabs(_distPara);
    uint32_t PWMHZ = _distPara * PulsePara * 1000 / _timInterv;
    if (PWMHZ < 650)
    {
        PWMHZ = 650;
    }
    else if (PWMHZ >= 42000000)
    {
        PWMHZ = 42000000;
    }
    _PwmTIM->ARR = (42000000 / PWMHZ - 1);
    PWMOutput(_PwmTIM, _ch, 0.5);
}