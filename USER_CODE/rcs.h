
#ifndef _RCS_H_
#define _RCS_H_
#include "dmp_mpu6050.h"

typedef struct
{
	double pitch;
	double roll;
	double yaw;
} Attitude;

typedef struct
{
	double longitude;// longitude精度
	double latitude;// latitude纬度
	double altitude;// altitude
} Coordinate;

typedef struct
{
	double THRVal;
	double RUDVal;
	double AILVal;
	double ELEVal;
} JoystickVal;

typedef struct
{  
	double kp;
	double ki;              
	double kd;              
} PID;//Signal Level

typedef struct
{
	PID shellPitchPID;
	PID shellRollPID;
	PID shellYawPID;
	PID corePitchPID;
	PID coreRollPID;
	PID coreYawPID;
	Attitude expectAgl;
	Triaxial realAgl;
	Attitude attitudeOut;
} AttitudeCtrl;

typedef struct
{
	PID longitudePID;
	PID latitudePID;
	PID altitudePID;
	Coordinate realCoord;
	Coordinate expectCoord;
	Coordinate coordinateOut;
} CoordinateCtrl;

typedef struct
{
	AttitudeCtrl attitudeCtrl;
	CoordinateCtrl coordinateCtrl;
	JoystickVal joystickVal;
} QuadrotorCtrl;



#define DEG2RAD         1.74532925199433e-2f            //角度变换成弧度     
#define RAD2DEG         5.72957795130823e1f
#define FLOAT_ZERO      1e-10f
/******************************************************************Quadrotor*************************************************************************/
/*************************调试串口**************************/
#define DEBUGUSART    USART2
#define DEBUGGPIO     GPIOD
#define DEBUGTXD      GPIO_Pin_5
#define DEBUGRXD      GPIO_Pin_6
#define DEBUGBAUD     9600
// /******************************MPU6050IIC****************************************/
// #define MPU_USART           USART1
// #define MPU_GPIO            GPIOB
// #define MPU_GPIO_TXD        GPIO_Pin_6
// #define MPU_GPIO_RXD        GPIO_Pin_7
// #define MPU_BAUD            57600
/*************************蓝牙串口**************************/
#define BT_USART    USART2
#define BT_GPIO     GPIOA
#define BT_TXD      GPIO_Pin_2
#define BT_RXD      GPIO_Pin_3
#define BT_BAUD     115200

/**************************轮子电机*********************/
#define CTRLSPEED_TIMER_PPR       500
#define CTRLSPEED_TIMER_DIV       10000

#define MAX_PWM (0.99)
#define MAX_ANGLE_PWM (0.95)

#define SQRT3       1.73205080756888f    //sqrt(3);

#define PWM_TIM_CLK     2100000 //pwm对应时钟的频率
#define PWM_FREQ        450   //pwm的周期

#define WHEEL_A_TIM         TIM8
#define WHEEL_B_TIM         TIM8
#define WHEEL_C_TIM         TIM8
#define WHEEL_D_TIM         TIM8
#define WHEEL_A_CH          1
#define WHEEL_B_CH          2
#define WHEEL_C_CH          3
#define WHEEL_D_CH          4
#define WHEEL_A_GPIO        GPIOC
#define WHEEL_B_GPIO        GPIOC
#define WHEEL_C_GPIO        GPIOC
#define WHEEL_D_GPIO        GPIOC
#define WHEEL_A_PIN         GPIO_Pin_6
#define WHEEL_B_PIN         GPIO_Pin_7
#define WHEEL_C_PIN         GPIO_Pin_8
#define WHEEL_D_PIN         GPIO_Pin_9
#define WHEEL_A_DIR         1 //轮子旋转方向逆转，1代表正转 -1代表反转
#define WHEEL_B_DIR         -1
#define WHEEL_C_DIR         1
#define WHEEL_D_DIR         -1

#define CTRLSPEED_TIMER           TIM14  /*底盘加速度控制定时器*/
/*电机转速和轮子半径*/
#define MOTOR_SPEED_RPS (5.208)     // round per second == RPM/60/减速比
#define WHEEL_RADIUS    (75.0)     //the wheel radius, mm
#define WHEEL_DISTANCE  (230.94)     //轮子到中心的距离
#define MAX_SPEED       3000

// /******************************蜂鸣器****************************************/
#define BUZZER_GPIO     GPIOE
#define BUZZER_PIN      GPIO_Pin_6
#define BUZZER_ON       GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN)
#define BUZZER_OFF      GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN)


/***********************超声波********************************/
#define  UltrasonicGPIO  GPIOB
#define  UltrasonicPinTx GPIO_Pin_10
#define  UltrasonicPinRx GPIO_Pin_11 


/*************************手柄串口************************/
#define JOYSTICK_USART      USART2
#define JOYSTICK_GPIO       GPIOD
#define JOYSTICK_TXD        GPIO_Pin_5
#define JOYSTICK_RXD        GPIO_Pin_6
#define JOYSTICK_BAUD       57600//19200

/************************编码器***********************/
// 增量式1 B PA0_TIM2_CH1      A PA1_TIM2_CH2  
// 增量式2 B PA2_TIM9_CH1      A PA3_TIM9_CH2  
#define ENCODER1_TIMER  TIM2
#define ENCODER1_PORT   GPIOA
#define ENCODER1_PIN1   GPIO_Pin_0
#define ENCODER1_PIN2   GPIO_Pin_1
#define ENCODER1_PIN3   GPIO_Pin_14

#define ENCODER2_TIMER  TIM9
#define ENCODER2_PORT   GPIOA
#define ENCODER2_PIN1   GPIO_Pin_2
#define ENCODER2_PIN2   GPIO_Pin_3
#define ENCODER2_PIN3   GPIO_Pin_11

#define ENCODER3_TIMER  TIM3
#define ENCODER3_PORT   GPIOA
#define ENCODER3_PIN1   GPIO_Pin_6
#define ENCODER3_PIN2   GPIO_Pin_7
#define ENCODER3_PIN3   GPIO_Pin_11


#include "RCS_Types.h"
#include "RCS_Exti.h"
#include "RCS_TimerInterrupt.h"
#include "RCS_Usart.h"
#include "RCS_Pwm.h"
#include "RCS_BTDebuger.h"
#include "RCS_Para.h"
#include "RCS_Eeprom.h"
#include "RCS_JoyStick.h"
#include "RCS_Motion_Basemove.h"
#include "RCS_Motion_Encoder.h"
#include "RCS_CommonFunction.h"
#include "RCS_I2C.h"
#include "HMC5883.h"
#include "dmp_driver.h"
#include "dmp_mpu6050.h"
#include "IIC.h"
#include "MPUI2C.h"
#include "Motor.h"
#include "PPM.h"
#include "IMU.h"

extern int8_t IS_DEBUG;

static void RCS_ResetSystem(uint8_t *data, uint8_t n)
{
	NVIC_SystemReset();
}
static void My_Debug_If(uint8_t *data, uint8_t n)
{
	IS_DEBUG = 1;
	BT_Send_Line("IS_DEBUG=1");
}
inline void RCSBaseLib_Init()
{
	// FLASH_Unlock();
	// EE_Init();
	// FLASH_Lock();
	InitBTDebuger(BT_USART, BT_GPIO, BT_TXD, BT_RXD, BT_BAUD);
	BT_Send_Str(" The program run success!\n");
	SetRCSComVect(0x54, RCS_ResetSystem);
	SetRCSComVect(0x44, My_Debug_If);
	//RCS_USART_Config(MPU_USART, MPU_GPIO, MPU_GPIO_TXD, MPU_GPIO_RXD, MPU_Process_ISR, MPU_BAUD, 0x23);

	//RCS_Init_I2C(I2C2, GPIOB, GPIO_Pin_10, GPIO_Pin_11, (uint16_t)0x00);
	//InitJoyStick(JOYSTICK_USART, JOYSTICK_GPIO,
	//           JOYSTICK_TXD, JOYSTICK_RXD, JOYSTICK_BAUD, Joystick_Interrupt);
}
inline void Init_RCS_Motion()
{
	InitBaseMotionModule();
	//InitGPS();
	//Fuzzy_Init();
}
#endif  //  _RCS_H_
