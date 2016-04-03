/*
 *@author : kgolym
 *@date:2012-10-05
 *@brief: Read the count of the encoder
 */


#include "rcs.h"

#define ICx_FILTER      3
#define TIMx_PRE_EMPTION_PRIORITY 0
#define TIMx_SUB_PRIORITY 1


const uint16_t ENCODER_PPR = (uint16_t)(500 * 40) ;  // number of pulses per revolution
const uint16_t COUNTER_RESET = (uint16_t)(500 * 20);


const uint16_t STD_ENCODER_PPR = (uint16_t)(2000) ;//一圈的脉冲数

volatile int32_t encoder1OverFlow = -1;
volatile int32_t encoder2OverFlow = -1;
volatile int32_t encoder3OverFlow = -1;

static void InitEncoder(TIM_TypeDef *_TIM,
						GPIO_TypeDef   *_port,
						uint32_t _pin1, uint32_t _pin2, uint32_t _pin3,
						FNCT_VOID _isr, FNCT_VOID _confirm_isr);
static void Encoder1_ISR();
static void Encoder2_ISR();
static void Encoder3_ISR();
static void Encoder1_Confirm_ISR();
static void Encoder2_Confirm_ISR();
static void Encoder3_Confirm_ISR();
//@brief:初始化编码器。
void InitEncoders()
{
	InitEncoder(ENCODER1_TIMER, ENCODER1_PORT, ENCODER1_PIN1, ENCODER1_PIN2, ENCODER1_PIN3, Encoder1_ISR, Encoder1_Confirm_ISR);
	InitEncoder(ENCODER2_TIMER, ENCODER2_PORT, ENCODER2_PIN1, ENCODER2_PIN2, ENCODER2_PIN3, Encoder2_ISR, Encoder2_Confirm_ISR);	
	// InitEncoder(ENCODER3_TIMER, ENCODER3_PORT, ENCODER3_PIN1, ENCODER3_PIN2, ENCODER3_PIN3, Encoder3_ISR, Encoder3_Confirm_ISR);
}

static void InitEncoder(TIM_TypeDef *_TIM,
						GPIO_TypeDef   *_port,
						uint32_t _pin1, uint32_t _pin2, uint32_t _pin3,
						FNCT_VOID _isr, FNCT_VOID _confirm_isr)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);

	if (_TIM == TIM1 ||  _TIM == TIM8 || _TIM == TIM9 || _TIM == TIM10 || _TIM == TIM11 )
	{
		RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_TIM(_TIM), ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_TIM(_TIM), ENABLE);
	}

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

	GPIO_InitStructure.GPIO_Pin = _pin1 | _pin2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(_port, &GPIO_InitStructure);
	GPIO_PinAFConfig(_port, GetRCS_GPIO_PinSource(_pin1) , GetRCS_GPIO_AF_TIM(_TIM));
	GPIO_PinAFConfig(_port, GetRCS_GPIO_PinSource(_pin2) , GetRCS_GPIO_AF_TIM(_TIM));

	BSP_IntVectSet(GetBSP_INT_ID_TIM(_TIM), (_isr));
	NVIC_InitStructure.NVIC_IRQChannel = GetRCS_TIM_IRQn(_TIM);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_EMPTION_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(_TIM);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling
	TIM_TimeBaseStructure.TIM_Period = ENCODER_PPR - 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(_TIM, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(_TIM, TIM_EncoderMode_TI12,
							   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//配置码盘确认函数的中断
	//RCS_InitEXTI(_port, _pin3,EXTI_Trigger_Rising, _confirm_isr, 0x11);

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;//选择输入比较滤波器
	TIM_ICInit(_TIM, &TIM_ICInitStructure);

	// Clear all pending interrupts
	TIM_ClearFlag(_TIM, TIM_FLAG_Update);
	TIM_ITConfig(_TIM, TIM_IT_Update, ENABLE);
	//Reset counter
	_TIM->CNT = COUNTER_RESET - 1;

	TIM_Cmd(_TIM, ENABLE);
}
//@brief: 编码器1溢出处理
static void Encoder1_ISR()
{
	int16_t count = ENCODER1_TIMER->CNT;
	if (count <= 0 )
	{
		encoder1OverFlow++;
	}
	else if (count >= ENCODER_PPR - 2)
	{
		encoder1OverFlow--;
	}
	ENCODER1_TIMER->CNT = COUNTER_RESET - 1;
	TIM_ClearITPendingBit(ENCODER1_TIMER, TIM_IT_Update);
}
//@brief: 编码器2溢出处理
static void Encoder2_ISR()
{
	int16_t count = ENCODER2_TIMER->CNT;
	if (count <= 0 )
	{
		encoder2OverFlow++;
	}
	else if (count >= ENCODER_PPR - 2)
	{
		encoder2OverFlow--;
	}
	ENCODER2_TIMER->CNT = COUNTER_RESET - 1;
	TIM_ClearITPendingBit(ENCODER2_TIMER, TIM_IT_Update);
}
//@brief: 编码器3溢出处理
static void Encoder3_ISR()
{
	int16_t count = ENCODER3_TIMER->CNT;
	if (count <= 0 )
	{
		encoder3OverFlow++;
	}
	else if (count >= ENCODER_PPR - 2)
	{
		encoder3OverFlow--;
	}
	ENCODER3_TIMER->CNT = COUNTER_RESET - 1;
	TIM_ClearITPendingBit(ENCODER3_TIMER, TIM_IT_Update);
}
//@brief: 编码器1，z轴修正
static void Encoder1_Confirm_ISR()
{
	static uint16_t ENCODER1_init_PPR = 2012;//编码器校正标准值，第一次中断初始化编码器值

	int16_t count = ENCODER1_TIMER->CNT;

	if (ENCODER1_init_PPR > 2000)
	{
		ENCODER1_init_PPR = count;
		ENCODER1_init_PPR %= STD_ENCODER_PPR;
		return;
	}
	uint16_t remainder = (count - ENCODER1_init_PPR) % STD_ENCODER_PPR;
	uint16_t consult;
	if (count > ENCODER1_init_PPR)
	{
		if (remainder < STD_ENCODER_PPR / 2)
		{
			consult = (count - ENCODER1_init_PPR) / STD_ENCODER_PPR;
		}
		else
		{
			consult = (1 + (count - ENCODER1_init_PPR) / STD_ENCODER_PPR);
		}
		ENCODER1_TIMER->CNT = consult * STD_ENCODER_PPR + ENCODER1_init_PPR;
	}
	else if ( ENCODER1_init_PPR -  count > STD_ENCODER_PPR / 2)
	{
		ENCODER1_TIMER->CNT = ENCODER1_init_PPR + ENCODER_PPR - STD_ENCODER_PPR;
	}
	else
	{
		ENCODER1_TIMER->CNT = ENCODER1_init_PPR;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(ENCODER1_PIN3));
}
//@brief: 编码器2，z轴修正
static void Encoder2_Confirm_ISR()
{
	static uint16_t ENCODER2_init_PPR = 2012;
	int16_t count = ENCODER2_TIMER->CNT;

	if (ENCODER2_init_PPR > 2000)
	{
		ENCODER2_init_PPR = count;
		ENCODER2_init_PPR %= STD_ENCODER_PPR;
		return;
	}
	uint16_t remainder = (count - ENCODER2_init_PPR) % STD_ENCODER_PPR;
	uint16_t consult;

	if (count > ENCODER2_init_PPR)
	{
		if (remainder < STD_ENCODER_PPR / 2)
		{
			consult = (count - ENCODER2_init_PPR) / STD_ENCODER_PPR;
		}
		else
		{
			consult = (1 + (count - ENCODER2_init_PPR) / STD_ENCODER_PPR);
		}
		ENCODER2_TIMER->CNT = consult * STD_ENCODER_PPR + ENCODER2_init_PPR;
	}
	else if ( ENCODER2_init_PPR -  count > STD_ENCODER_PPR / 2)
	{
		ENCODER2_TIMER->CNT = ENCODER2_init_PPR + ENCODER_PPR - STD_ENCODER_PPR;
	}
	else
	{
		ENCODER2_TIMER->CNT = ENCODER2_init_PPR;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(ENCODER2_PIN3));
}

//@brief: 编码器3，z轴修正
static void Encoder3_Confirm_ISR()
{
	static uint16_t ENCODER3_init_PPR = 2012;
	int16_t count = ENCODER3_TIMER->CNT;

	if (ENCODER3_init_PPR > 2000)
	{
		ENCODER3_init_PPR = count;
		ENCODER3_init_PPR %= STD_ENCODER_PPR;
		return;
	}
	uint16_t remainder = (count - ENCODER3_init_PPR) % STD_ENCODER_PPR;
	uint16_t consult;

	if (count > ENCODER3_init_PPR)
	{
		if (remainder < STD_ENCODER_PPR / 2)
		{
			consult = (count - ENCODER3_init_PPR) / STD_ENCODER_PPR;
		}
		else
		{
			consult = (1 + (count - ENCODER3_init_PPR) / STD_ENCODER_PPR);
		}
		ENCODER3_TIMER->CNT = consult * STD_ENCODER_PPR + ENCODER3_init_PPR;
	}
	else if ( ENCODER3_init_PPR -  count > STD_ENCODER_PPR / 2)
	{
		ENCODER3_TIMER->CNT = ENCODER3_init_PPR + ENCODER_PPR - STD_ENCODER_PPR;
	}
	else
	{
		ENCODER3_TIMER->CNT = ENCODER3_init_PPR;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(ENCODER3_PIN3));
}