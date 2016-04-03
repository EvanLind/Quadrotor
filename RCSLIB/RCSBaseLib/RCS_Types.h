//文件名：  RCS_types.h
//日期： 2012-08-17
//作者： 柯国霖
//文件说明：一些类型的定义
//修改历史：
//2012-08-25	22:00	柯国霖	大量修改,改用函数封装
//
#ifndef _RCS_TYPES_H_
#define _RCS_TYPES_H_

#include "bsp.h"

typedef void (*FNCT_VOID)(void);        //无返回值空参数的函数指针类型

typedef void (*RCS_COMMAND_FUNT)(uint8_t *, uint8_t);        //无返回值空参数的函数指针类型


uint32_t GetRCS_RCC_APB2Periph_ADC(ADC_TypeDef* ADCx);
uint32_t GetRCS_RCC_AHB1Periph_GPIO(GPIO_TypeDef *_g);
uint16_t  GetRCS_GPIO_PinSource(uint32_t _p);
uint8_t GetRCS_EXTI_PortSourceGPIO(GPIO_TypeDef *_g);
uint8_t GetRCS_EXTI_PinSource(uint32_t _p);
uint32_t GetRCS_EXTI_Line(uint32_t _p);
uint16_t GetRCS_EXTI_IRQn(uint32_t _p);
CPU_DATA GetRCS_BSP_INT_ID_EXTI(uint32_t _p);


uint32_t  GetRCS_RCC_APB1Periph_USART(USART_TypeDef *_u);
uint32_t  GetRCS_RCC_APB2Periph_USART(USART_TypeDef *_u);
uint8_t  GetRCS_GPIO_AF(USART_TypeDef *_u);
CPU_DATA  GetBSP_INT_ID_USART(USART_TypeDef *_u);
uint8_t  GetRCS_USART_IRQn(USART_TypeDef *_u);
uint8_t  GetRCS_GPIO_CAN_AF(CAN_TypeDef *_u);
uint32_t  GetRCS_RCC_APB1Periph_CAN(CAN_TypeDef *_u);

uint32_t GetRCS_RCC_APB1Periph_TIM(TIM_TypeDef * _t);
uint32_t GetRCS_RCC_APB2Periph_TIM(TIM_TypeDef * _t);
CPU_DATA GetBSP_INT_ID_TIM(TIM_TypeDef * _t);
uint8_t GetRCS_TIM_IRQn(TIM_TypeDef * _t);
uint8_t GetRCS_GPIO_AF_TIM(TIM_TypeDef * _t);

uint8_t GetRCS_GPIO_AF_SPI(SPI_TypeDef * _s);
uint32_t GetRCS_RCC_APB1Periph_SPI(SPI_TypeDef * _s);
uint32_t GetRCS_RCC_APB2Periph_SPI(SPI_TypeDef * _s);
CPU_DATA GetBSP_INT_ID_SPI(SPI_TypeDef * _s);
uint8_t GetRCS_SPI_IRQn(SPI_TypeDef * _s);


uint32_t GetRCS_RCC_APB2Periph_I2C(I2C_TypeDef* _u);
CPU_DATA GetBSP_INT_ID_I2C_EV(I2C_TypeDef *_u);
CPU_DATA GetBSP_INT_ID_I2C_ER(I2C_TypeDef *_u);
uint8_t GetRCS_I2C_EV_IRQn(I2C_TypeDef *_u);
uint8_t GetRCS_I2C_ER_IRQn(I2C_TypeDef *_u);
uint8_t GetRCS_I2C_AF(I2C_TypeDef *_u);



#endif  //  _RCS_TYPES_H_
