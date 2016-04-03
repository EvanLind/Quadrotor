//文件名：  RCS_exti.h
//日期： 2012-08-17
//作者： 郑文楷&&魏闻
//文件说明：外部中断模块封装
//修改历史：
//2012-08-17    23:30   柯国霖   修改函数名 参数名 文件名
//2012-08-18    09:30   柯国霖   同步注释和代码，修改时间格式，建议统一为 yyyy-mm-dd
//2012-08-25	22:00	柯国霖	大量修改,改用函数封装
//2012-10-14    14:00   柯国霖   添加中断优先级的设定
//2012-12-08    17:00 	柯国霖   增加入口参数检查，规范注释
#ifndef _RCS_EXTI_H_
#define _RCS_EXTI_H_

#include "bsp.h"
#include "RCS_Types.h"

//@name:RCS_InitEXTI
//@brief:   初始化外部中断功能
//@param:GPIO_TypeDef *  _port: 端口
//@param:uint32_t _pin: 端口序号
//@param:EXTITrigger_TypeDef _trigger
//      @args:EXTI_Trigger_Rising
//      @args:EXTI_Trigger_Falling
//      @args:EXTI_Trigger_Rising_Falling
//@param:FNCT_VOID _isr: 中断服务程序的函数指针
//@param:uint8_t _priority ：前4位为抢占优先级，后4位为响应优先级
//@note: 清零中断函数 EXTI_ClearITPendingBit(GetRCS_EXTI_Line(_pin));
void RCS_InitEXTI(GPIO_TypeDef * _port, uint32_t _pin, EXTITrigger_TypeDef _trigger, FNCT_VOID _isr, uint8_t _priority);
void SetGpioOutput(GPIO_TypeDef *_port, uint32_t _pin);
void SetGpioInput(GPIO_TypeDef *_port, uint32_t _pin);

inline void SetGpioLow(GPIO_TypeDef *_port, uint32_t _pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	assert_param(IS_GPIO_PIN(_pin));
	_port->BSRRH = _pin;
}
inline void SetGpioHigh(GPIO_TypeDef *_port, uint32_t _pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	assert_param(IS_GPIO_PIN(_pin));
	_port->BSRRL = _pin;
}
inline void ToggleGpio(GPIO_TypeDef *_port, uint32_t _pin)
{
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	_port->ODR ^= _pin;
}

inline uint8_t ReadGpio(GPIO_TypeDef *_port, uint32_t _pin)
{
	return GPIO_ReadInputDataBit(_port, _pin);
}
#endif  //  _RCS_EXTI_H_