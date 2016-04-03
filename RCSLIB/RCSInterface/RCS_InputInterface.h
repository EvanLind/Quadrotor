#ifndef _RCS_INPUTINTERFACE_H_
#define _RCS_INPUTINTERFACE_H_

#include "RCS_Exti.h"

void InputInterface_Init();
void InputInterface_SelectGroup(uint8_t _group);
inline uint8_t InputInterface_ReadAll()
{
	uint8_t x = (uint8_t)(GPIOD->IDR & 0x00ff);
	return x;
}
//@brief:选择管脚号，范围1-8。
//@retval:返回管脚，1为有输入。0为没有。悬空状态不确定。
inline uint8_t InputInterface_ReadOne(uint8_t _num)
{
	uint8_t x = (uint8_t)(GPIOD->IDR & 0x00ff);
	x >>= (_num-1);
	x &= 0x01;
	return x;
}
#endif //_RCS_INPUTINTERFACE_H_