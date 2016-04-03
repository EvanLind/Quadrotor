#ifndef _MOTOR_H_
#define _MOTOR_H_

#define MOTOR_RF_TIM         TIM8
#define MOTOR_RB_TIM         TIM8
#define MOTOR_LB_TIM         TIM8
#define MOTOR_LF_TIM         TIM8
#define MOTOR_RF_CH          1
#define MOTOR_RB_CH          2
#define MOTOR_LB_CH          3
#define MOTOR_LF_CH          4
#define MOTOR_RF_GPIO        GPIOC
#define MOTOR_RB_GPIO        GPIOC
#define MOTOR_LB_GPIO        GPIOC
#define MOTOR_LF_GPIO        GPIOC
#define MOTOR_RF_PIN         GPIO_Pin_6
#define MOTOR_RB_PIN         GPIO_Pin_7
#define MOTOR_LB_PIN         GPIO_Pin_8
#define MOTOR_LF_PIN         GPIO_Pin_9

void InitMotor();
void MotorCtrl();

#endif