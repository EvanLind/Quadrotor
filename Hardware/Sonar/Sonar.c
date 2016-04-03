#include "bsp.h"
#include "rcs.h"
#include "Sonar.h"


#define SONAR_GPIO		GPIOE
#define SONAR_TRIG_PIN  GPIO_Pin_0
#define SONAR_ECHO_PIN  GPIO_Pin_1
#define SONAR_TIM		TIM5


volatile float sonarDis;

void InitSonar()
{
	RCS_InitEXTI(SONAR_GPIO, SONAR_ECHO_PIN, EXTI_Trigger_Rising_Falling, ECHO_EXTI_IRQHandler, 0x10);
	SetGpioOutput(SONAR_GPIO, SONAR_TRIG_PIN);
	InitTimerInt(SONAR_TIM, 10000, 1000000, NULL, 0x19);
}

void ECHO_EXTI_IRQHandler(void)
{
    if(GPIO_ReadInputDataBit(SONAR_GPIO, SONAR_ECHO_PIN) != 0)
    	SONAR_TIM->CNT=0;
    else 
    {
    	sonarDis = (float)TIM_GetCounter(SONAR_TIM) / 59.0;
    }
    EXTI_ClearITPendingBit(GetRCS_EXTI_Line(SONAR_ECHO_PIN));   
}

float GetSonarDis(void)
{   
    GPIO_SetBits(SONAR_GPIO, SONAR_TRIG_PIN);
	for (int i = 0; i < 300; ++i)
	{
		;
	}
    GPIO_ResetBits(SONAR_GPIO, SONAR_TRIG_PIN);
    // OSTimeDly(1);
 //    char res[50];
	// sprintf(res, "Dis = %f \n\0", sonarDis);
	// RCS_USART_Send_Str(DEBUGUSART, res);            
	// RCS_USART_Send_Char(DEBUGUSART, '\n');
    return sonarDis;
}



// #include "board.h"
// #include "mw.h"

// #ifdef SONAR

// /* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
//  * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
//  * echo froman object. The distance between the unit and the object is calculated
//  * by measuring the traveling time of sound and output it as the width of a TTL pulse.
//  *
//  * *** Warning: HC-SR04 operates at +5V ***
//  *
//  */

// static uint16_t trigger_pin;
// static uint16_t echo_pin;
// static uint32_t exti_line;
// static uint8_t exti_pin_source;
// static IRQn_Type exti_irqn;

// static uint32_t last_measurement;
// static volatile int16_t* distance_ptr;

// void ECHO_EXTI_IRQHandler(void)
// {
//     static uint32_t timing_start;
//     uint32_t timing_stop;
//     if(GPIO_ReadInputDataBit(GPIOB, echo_pin) != 0)
//         timing_start = micros();
//     else 
//     {
//         timing_stop = micros();
//         if(timing_stop > timing_start) 
//         {
//             // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
//             // The ping travels out and back, so to find the distance of the
//             // object we take half of the distance traveled.
//             //
//             // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
//             int32_t pulse_duration = timing_stop - timing_start;          
//             *distance_ptr = pulse_duration / 59 ;
//         }
//     }

//     EXTI_ClearITPendingBit(exti_line);   
// }

// void EXTI1_IRQHandler(void)
// {
//     ECHO_EXTI_IRQHandler();
// }

// void EXTI9_5_IRQHandler(void)
// {
//     ECHO_EXTI_IRQHandler();
// }

// void hcsr04_init(sonar_config_t config)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     EXTI_InitTypeDef EXTIInit;

//     //enable AFIO for EXTI support - already done is drv_system.c
//     //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph, ENABLE); 

//     switch(config)
//     {
//     case sonar_pwm56:
//         trigger_pin = GPIO_Pin_8;   // PWM5 (PB8) - 5v tolerant
//         echo_pin = GPIO_Pin_9;      // PWM6 (PB9) - 5v tolerant
//         exti_line = EXTI_Line9;
//         exti_pin_source = GPIO_PinSource9;
//         exti_irqn = EXTI9_5_IRQn;
//         break;
//     case sonar_rc78:    
//         trigger_pin = GPIO_Pin_0;   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
//         echo_pin = GPIO_Pin_1;      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
//         exti_line = EXTI_Line1;
//         exti_pin_source = GPIO_PinSource1;
//         exti_irqn = EXTI1_IRQn;
//         break;
//     }
	
//     // tp - trigger pin 
//     GPIO_InitStructure.GPIO_Pin = trigger_pin;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);

//     // ep - echo pin
//     GPIO_InitStructure.GPIO_Pin = echo_pin;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//     GPIO_Init(GPIOB, &GPIO_InitStructure);

//     // setup external interrupt on echo pin
//     GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, exti_pin_source);

//     EXTI_ClearITPendingBit(exti_line);
	   
//     EXTIInit.EXTI_Line = exti_line;
//     EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
//     EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//     EXTIInit.EXTI_LineCmd = ENABLE;    
//     EXTI_Init(&EXTIInit);    

//     NVIC_EnableIRQ(exti_irqn);

//     last_measurement = millis() - 60; // force 1st measurement in hcsr04_get_distance()
// }

// // distance calculation is done asynchronously, using interrupt
// void hcsr04_get_distance(volatile int16_t* distance)
// {   
//     uint32_t current_time = millis();

//     if( current_time < (last_measurement + 60) )
//     {
//         // the repeat interval of trig signal should be greater than 60ms
//         // to avoid interference between connective measurements.
//         return;
//     }
		
//     last_measurement = current_time;
//     distance_ptr = distance;

//     GPIO_SetBits(GPIOB, trigger_pin);
//     //  The width of trig signal must be greater than 10us
//     delayMicroseconds(11);
//     GPIO_ResetBits(GPIOB, trigger_pin);
// }

// #endif



	// SetGpioInput(GPIOE, GPIO_Pin_9);
	// GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	// SetGpioOutput(SONAR_GPIO, SONAR_TRIG_PIN);
	// //InitTimerInt(TIM3, 1, 1000000, NULL, 0x18);
	// InitTimerInt(TIM5, 10000, 1000000, NULL, 0x19);
	// while(1)
	// {
	// 	OSTimeDly(70);
	// 	GPIO_SetBits(GPIOE, GPIO_Pin_7);
	// 	//TIMDly1us(30);
	// 	for (int i = 0; i < 300; ++i)
	// 	{
	// 		;
	// 	}
	// 	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	// 	TIM5->CNT=0;
	// 	while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 0);
	// 	TIM_Cmd(TIM5, ENABLE);
	// 	while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9));
	// 	count = TIM_GetCounter(TIM5);
	// 	dis = TIM_GetCounter(TIM5) * 0.017;
	// 	TIM_Cmd(TIM5, DISABLE);
	// 	sprintf(strShow, "%f", dis);
	// 	oled_wr_6x8str(0, 0, strShow);
	// }
	// void TIMDly1us(uint16_t  delayTime)
	// {
	// 	uint16_t TIMCounter = delayTime*2;
	// 	TIM_Cmd(TIM3, ENABLE);
	// 	TIM_SetCounter(TIM3, TIMCounter);
	// 	while (TIM_GetCounter(TIM3) < TIMCounter)
	// 	{
	// 		;
	// 	}
	// 	TIM_Cmd(TIM3, DISABLE);
	// }