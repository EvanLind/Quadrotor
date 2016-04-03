//@filename： RCS_pwm.C
//@date:2012-08-18
//@author:黄东润
//@brief:pwm 方波输出控制
//计算公式
//  TimClk = (uint32_t ) (rcc_clocks.SYSCLK_Frequency / 2);TimClk 频率等于PCLK2的频率
//  PrescalerValue = (uuint32_t16_t) (TimClk / _CLKHZ ) - 1; 计数器频率由分频控制
//  ARR = (uint32_t)(_CLKHZ / _PWMHZ); ARR 控制输出频率
//  CCRValue = (uint32_t)(_percent * ARR / 100); CCR 控制占分比

#ifndef __RCS_PWM__
#define __RCS_PWM__

#include "bsp.h"
#include "RCS_Types.h"

//@name: PWMInit
//@brief: 启动时钟中断
//@param:TIM_TypeDef * _TIM ： 定时器号
//@param:uint8_t _ch  ：定时器 pwm输出 通道号
//@param:GPIO_TypeDef* _port ：pwmm输出管脚
//@param:uint32_t _pin : pwm输出针脚
//@param:uint32_t _CLKHZ : 定时器计数器频率，必须为能被计时器时钟频率（84MHZ）整除
//@param:uint32_t _PWMHZ : pwm输出频率
//@retval: 初始化成功与否
Boolean PWMInit(TIM_TypeDef *_TIM, uint8_t _ch,
				GPIO_TypeDef   *_port, uint32_t _pin,
				uint32_t _CLKHZ, uint32_t _PWMHZ);

void StepMotorInit(TIM_TypeDef *_PwmTIM, uint8_t _ch,
                   GPIO_TypeDef   *_pwmport, uint32_t _pwmpin,
                   GPIO_TypeDef   *_dirport, uint32_t _dirpin,
                   TIM_TypeDef *_TIM2Timing, uint8_t _priority, float _CirPulse, double _para);
void StepMotorOutPut(TIM_TypeDef *_PwmTIM, uint8_t _ch, double _distPara, double _timInterv);
Boolean PWMGetIn_Period_Duty(TIM_TypeDef *_TIM, uint8_t _ch,
                             GPIO_TypeDef   *_port, uint32_t _pin, uint8_t _pri);

//@brief:获取定时器的周期
//@param:   TIM_TypeDef * _TIM 使用的定时器
inline uint32_t PWMGetPeriod(TIM_TypeDef *_TIM)
{
	uint32_t ARR = _TIM->ARR + 1;
	return ARR;
}


//@name: PWMOutput
//@brief: 在输出频率不变的情况下改变pwm占空比
//@param:TIM_TypeDef * _TIM 使用的定时器
//@param:uint8_t _ch，输出的通道
//@param:double _percent 输出占分比
inline void PWMOutput(TIM_TypeDef *_TIM, uint8_t _ch, double _percent)
{
	assert(_percent < 1 && _percent >= 0);
	uint32_t ARR = PWMGetPeriod(_TIM);
	uint32_t CCRValue = (uint32_t)(_percent * ARR);
	if (_ch == 1)
	{
		_TIM -> CCR1 = CCRValue;
	}
	else if (_ch == 2)
	{
		_TIM -> CCR2 = CCRValue;
	}
	else if (_ch == 3)
	{
		_TIM -> CCR3 = CCRValue;
	}
	else if (_ch == 4)
	{
		_TIM -> CCR4 = CCRValue;
	}
}

//@name: PWMGetPercent
//@brief:获取pwm的占空比
//@param:TIM_TypeDef * _TIM 使用的定时器
//@param:uint8_t _ch，输出的通道
//@retval: pwm占空比，范围0-1
inline double PWMGetPercent(TIM_TypeDef *_TIM, uint8_t _ch)
{
	uint32_t ARR = PWMGetPeriod(_TIM);
	if (_ch == 1)
	{
		return (double)(_TIM -> CCR1) / (double)ARR;
	}
	else if (_ch == 2)
	{
		return (double)(_TIM -> CCR2) / (double)ARR;
	}
	else if (_ch == 3)
	{
		return (double)(_TIM -> CCR3) / (double)ARR;
	}
	else if (_ch == 4)
	{
		return (double)(_TIM -> CCR4) / (double)ARR;
	}
	return -1;
}

#endif