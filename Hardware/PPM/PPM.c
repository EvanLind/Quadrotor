#include "rcs.h"
#include "PPM.h"



TIM_ICInitTypeDef  TIM_ICInitStructure;
void InitPPM(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	 
	/* TIM4 chennel2 configuration: A.01 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15 , GPIO_AF_TIM4);
	// GPIO_PinRemapConfig(GPIO_FullRemap_TIM4, ENABLE);
	 
	/* Enable the TIM4 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	 
	 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	 
	TIM_TimeBaseStructure.TIM_Prescaler   = 4;
	TIM_TimeBaseStructure.TIM_Period      = 0xffff;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	 
	 
	 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);


	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	BSP_IntVectSet(GetBSP_INT_ID_TIM(TIM4), TIM4_IRQ);
	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM4, TIM_IT_CC1 |TIM_IT_CC2 |TIM_IT_CC3 |TIM_IT_CC4 , ENABLE);//TIM_IT_Update | 
	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);
}
//			飞机上下(左上下，上大下小)		飞机旋转(左左右，左小右大)		飞机左右飞(右左右，左小右大)	飞机前后飞(右上下，上小下大)			
// RecvCom  THR油门0:(18960, 31055)25416    RUD方向1:(19628, 31452)27296    AIL副翼2:(18813, 31179)25145	ELE升降3:(18813, 31452)25540
volatile int RecvCom[4], LastRecvCom[4] = {18960, 27296, 25145, 25540};
int32_t RecvCom_old[4]={0,0,0,0};
unsigned char Recv1_Flag,Recv2_Flag,Recv3_Flag,Recv0_Flag;
static char pulseState1 = 0,pulseState2 = 0,pulseState3 = 0,pulseState4 = 0;
int cal(uint32_t u1,uint32_t u2)
{
	return abs(u1 - u2);

	// int t = u1-u2;
	// if(t >= 65535)
	// 	return t - 65535;
	// else if(t <= -65535)
	// 	return t + 65535;
	// else
	// 	return t;

	// if(u1>u2)
	// 	return u1-u2;
	// else
	// 	return u1+0xffff-u2;
}

int time = 0;

void TIM4_IRQ(void)
{
	if (TIM4->SR & TIM_IT_CC1 )
	{
		TIM4 ->SR &= (~TIM_IT_CC1);//清除中断标志
		if (pulseState1 == 0) //变量pulseState1用于调整中断处理的任务：分为每通道第一次进入计数清零，第二次计入保存计数
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			RecvCom_old[0] = TIM_GetCapture1(TIM4);
			// TIM_SetCounter(TIM4,0);
		}
		else 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			RecvCom[0] = abs(TIM_GetCapture1(TIM4) - RecvCom_old[0]);
			if (RecvCom[0] >31500 || RecvCom[0] <18800 || abs(RecvCom[0] - LastRecvCom[0]) <= 10)
			{
				RecvCom[0] = LastRecvCom[0];
			}
			LastRecvCom[0] = RecvCom[0];
			Recv0_Flag=1;
		}
		pulseState1 = !pulseState1;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInit(TIM4, &TIM_ICInitStructure);
	}
	if (TIM4 ->SR & TIM_IT_CC2) 
	{
		TIM4 ->SR &= (~TIM_IT_CC2 );
		if (pulseState2 == 0) 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			RecvCom_old[1] = TIM_GetCapture2(TIM4);	 
		} 
		else 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			RecvCom[1] = abs(TIM_GetCapture2(TIM4) - RecvCom_old[1]);
			if (RecvCom[1] >31500 || RecvCom[1] <18800 || abs(RecvCom[1] - LastRecvCom[1]) <= 10)
			{
				RecvCom[1] = LastRecvCom[1];
			}
			LastRecvCom[1] = RecvCom[1];
			Recv1_Flag=1;
		}
		pulseState2 = !pulseState2;	 
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInit(TIM4, &TIM_ICInitStructure);
	 
	}
	 
	if (TIM4 ->SR & TIM_IT_CC3 ) 
	{
		TIM4 ->SR &= (~TIM_IT_CC3);
		if (pulseState3 == 0) 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			RecvCom_old[2] = TIM_GetCapture3(TIM4);		 
		} 
		else 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			RecvCom[2] = abs(TIM_GetCapture3(TIM4) - RecvCom_old[2]);
			if (RecvCom[2] >31500 || RecvCom[2] <18800 || abs(RecvCom[2] - LastRecvCom[2]) <= 10)
			{
				RecvCom[2] = LastRecvCom[2];
			}
			LastRecvCom[2] = RecvCom[2];
			Recv2_Flag=1;
		}
		pulseState3 = !pulseState3; 
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
		TIM_ICInit(TIM4, &TIM_ICInitStructure); 
	}
	 
	if (TIM4 ->SR & TIM_IT_CC4 ) 
	{
		TIM4 ->SR &= (~TIM_IT_CC4);
		if (pulseState4 == 0) 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			RecvCom_old[3] = TIM_GetCapture4(TIM4);		} 
		else 
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			RecvCom[3] = abs(TIM_GetCapture4(TIM4) - RecvCom_old[3]);
			if (RecvCom[3] >31500 || RecvCom[3] <18800 || abs(RecvCom[3] - LastRecvCom[3]) <= 10)
			{
				RecvCom[3] = LastRecvCom[3];
			}
			LastRecvCom[3] = RecvCom[3];
			Recv3_Flag=1;
		}
		pulseState4 = !pulseState4;		 
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
		TIM_ICInit(TIM4, &TIM_ICInitStructure);	 
	}

	time++;
	if (time == 200)
	{
		
	}
}