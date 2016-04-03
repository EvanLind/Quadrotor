#include "RCS_InputInterface.h"

//组号范围1-4.管脚号范围：1-8。


//@brief:初始化，默认选择第1组
void InputInterface_Init()
{
	SetGpioOutput(GPIOE, GPIO_Pin_13);
	SetGpioOutput(GPIOE, GPIO_Pin_11);
	SetGpioInput(GPIOD,
				 GPIO_Pin_0 |
				 GPIO_Pin_1 |
				 GPIO_Pin_2 |
				 GPIO_Pin_3 |
				 GPIO_Pin_4 |
				 GPIO_Pin_5 |
				 GPIO_Pin_6 |
				 GPIO_Pin_7);
		SetGpioLow(GPIOE, GPIO_Pin_13);
		SetGpioLow(GPIOE, GPIO_Pin_11);
}
//@brief:选择组号，范围1-4
void InputInterface_SelectGroup(uint8_t _group)
{
	assert(_group <= 4 && _group >= 1);
	switch (_group)
	{
	case 1:
		SetGpioLow(GPIOE, GPIO_Pin_13);
		SetGpioLow(GPIOE, GPIO_Pin_11);
		break;
	case 2:
		SetGpioHigh(GPIOE, GPIO_Pin_11);
		SetGpioLow(GPIOE, GPIO_Pin_13);
		break;
	case 3:
		SetGpioLow(GPIOE, GPIO_Pin_11);
		SetGpioHigh(GPIOE, GPIO_Pin_13);
		break;
	case 4:
		SetGpioHigh(GPIOE, GPIO_Pin_13);
		SetGpioHigh(GPIOE, GPIO_Pin_11);
		break;
	}
}
