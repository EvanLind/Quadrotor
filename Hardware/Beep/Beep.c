#include "rcs.h"
#include "Beep.h"


void InitBeep(GPIO_TypeDef *_port, uint32_t _pin)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = _pin ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(_port, &GPIO_InitStructure);
	GPIO_SetBits(_port, _pin);
}

void Beep(int count)
{
	for (int i = 0; i < count; ++i)
	{
		BUZZER_ON;
		OSTimeDly(100);
		BUZZER_OFF;
		OSTimeDly(100);
	}
}