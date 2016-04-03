#include "rcs.h"
#include "Motor.h"
#include <string.h>

// #define PIDMIX(X,Y,Z) rxCommand[THROTTLE] + axisPID[ROLL] * X + axisPID[PITCH] * Y + systemConfig.yawDirection * axisPID[YAW] * Z
//             motor[0] = PIDMIX(  1.0f, -0.666667f, 0.0f );  // Left  CW


#define THRLOW 0.51
#define THRHIGH 0.9
#define LIMIT(val)    {val<THRLOW?(val = 0.49):val; val>THRHIGH?(val = 0.9):val;}
#define ECINIT
// // X Mode X模式
// //     1   2         LF     RF     A
// //       X               X         |
// //     3   4         LB     RB     | 
// Motor1=MotorLimitValue(thr + ail - ele + rud);
// Motor2=MotorLimitValue(thr - ail - ele - rud);
// Motor3=MotorLimitValue(thr + ail + ele - rud);
// Motor4=MotorLimitValue(thr - ail + ele + rud);


extern volatile int RecvCom[4];
int InitCom[4];
extern QuadrotorCtrl Quadrotor;
extern Triaxial averageTria;


void InitMotor()
{
	PWMInit(MOTOR_RF_TIM, MOTOR_RF_CH, MOTOR_RF_GPIO, MOTOR_RF_PIN, PWM_TIM_CLK, PWM_FREQ);//右前方
	PWMInit(MOTOR_RB_TIM, MOTOR_RB_CH, MOTOR_RB_GPIO, MOTOR_RB_PIN, PWM_TIM_CLK, PWM_FREQ);//右后方
	PWMInit(MOTOR_LB_TIM, MOTOR_LB_CH, MOTOR_LB_GPIO, MOTOR_LB_PIN, PWM_TIM_CLK, PWM_FREQ);//左后方
	PWMInit(MOTOR_LF_TIM, MOTOR_LF_CH, MOTOR_LF_GPIO, MOTOR_LF_PIN, PWM_TIM_CLK, PWM_FREQ);//左前方
	PWMOutput(MOTOR_RF_TIM, MOTOR_RF_CH | MOTOR_RB_CH | MOTOR_LB_CH | MOTOR_LF_CH, 0.49);
	OSTimeDly(100);
	#ifdef DEBUG
		SetGpioInput(GPIOA, GPIO_Pin_0);
		#ifdef ECINIT
			OSTimeDly(30);
			PWMOutput(MOTOR_RF_TIM, MOTOR_RF_CH | MOTOR_RB_CH | MOTOR_LB_CH | MOTOR_LF_CH, 0.01);
			OSTimeDly(100);
			while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
			{
				OSTimeDly(5);
			} 
		#else
			PWMOutput(MOTOR_RF_TIM, MOTOR_RF_CH, 0.01);
			while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
			{
				OSTimeDly(5);
			}
		#endif
	#endif
	memcpy(InitCom, (void *)RecvCom, sizeof(RecvCom));
	InitCom[0] += 100;
	Quadrotor.attitudeCtrl.expectAgl.roll = averageTria.roll;
	Quadrotor.attitudeCtrl.expectAgl.pitch = averageTria.pitch;
	Quadrotor.attitudeCtrl.expectAgl.yaw = averageTria.yaw;
}

Attitude attitudeOut;
Coordinate coordinateOut;
//          飞机上下(左上下，上大下小)      飞机旋转(左左右，左小右大)      飞机左右飞(右左右，左小右大) 飞机前后飞(右上下，上小下大)         
// RecvCom  THR油门0:(18960, 31055)25416    RUD方向1:(20025, 31452)27296    AIL副翼2:(18813, 31179)25145  ELE升降3:(18813, 31452)25540
void MotorCtrl()
{
	Quadrotor.joystickVal.THRVal = 2.475e-5 * (RecvCom[0] - InitCom[0]) + 0.49;
	Quadrotor.joystickVal.RUDVal = 5e-6 * (RecvCom[1] - InitCom[1]);
	Quadrotor.joystickVal.AILVal = 4e-3 * (RecvCom[2] - InitCom[2]);
	Quadrotor.joystickVal.ELEVal = 4e-3 * (RecvCom[3] - InitCom[3]);

	Quadrotor.attitudeCtrl.expectAgl.roll = averageTria.roll + Quadrotor.joystickVal.AILVal;
	Quadrotor.attitudeCtrl.expectAgl.pitch = averageTria.pitch - Quadrotor.joystickVal.ELEVal;
	Quadrotor.attitudeCtrl.expectAgl.yaw += Quadrotor.joystickVal.RUDVal;

	attitudeOut = AttitudeControl(&(Quadrotor.attitudeCtrl));
	coordinateOut = AltitudeControl(&(Quadrotor.coordinateCtrl));

	/***********************************************改变目标角度*********************************************************/
	double THR = Quadrotor.joystickVal.THRVal * (2 - cos(Quadrotor.joystickVal.ELEVal * DEG2RAD) * cos(Quadrotor.joystickVal.AILVal * DEG2RAD));
	double RUD = attitudeOut.yaw;//+ 0.06 + attitudeOut.yaw;//Yaw(0~0.1)
	double AIL = attitudeOut.roll;//+ 0.0512 + attitudeOut.roll;//Roll(0~0.1)S
	double ELE = -attitudeOut.pitch;//+ 0.0532 + attitudeOut.pitch;//Pitch(0~0.1)

	/***********************************************PWM叠加模式**********************************************************/
	// double THR = 2.475e-5 * (RecvCom[0] - InitCom[0]) + 0.49;//基础值(0.5~0.8  60%)
	// double RUD = 4e-6 * (RecvCom[1] - InitCom[1]) + attitudeOut.yawOut;//+ 0.06 + attitudeOut.yawOut;//Yaw(0~0.1)
	// double AIL = 4e-6 * (RecvCom[2] - InitCom[2]) + attitudeOut.rollOut;//+ 0.0512 + attitudeOut.rollOut;//Roll(0~0.1)//
	// double ELE = 4e-6 * (RecvCom[3] - InitCom[3]) + attitudeOut.pitchOut;//+ 0.0532 + attitudeOut.pitchOut;//Pitch(0~0.1)

	/*************************************************速度融合***********************************************************/
	double MOTOR_RF_Pwm = THR - AIL + ELE + RUD;
	double MOTOR_RB_Pwm = THR - AIL - ELE - RUD;
	double MOTOR_LB_Pwm = THR + AIL - ELE + RUD;
	double MOTOR_LF_Pwm = THR + AIL + ELE - RUD;

	/*************************************************速度限幅***********************************************************/
	LIMIT(MOTOR_RF_Pwm);
	LIMIT(MOTOR_RB_Pwm);
	LIMIT(MOTOR_LB_Pwm);
	LIMIT(MOTOR_LF_Pwm);
	// char res[50];
	// // sprintf(res, "R0:%d R1:%d R2:%d R3:%d\n\0",RecvCom[0], RecvCom[1], RecvCom[2], RecvCom[3]);
	// // sprintf(res, "ctrlPitch:%.5f ctrlRoll:%.5f\n\0",cos(ctrlPitch * DEG2RAD), cos(ctrlRoll * DEG2RAD));
	// sprintf(res, "RF:%.5f RB:%.5f LB:%.5f LF:%.5f\n\0",MOTOR_RF_Pwm, MOTOR_RB_Pwm, MOTOR_LB_Pwm, MOTOR_LF_Pwm);
	// // sprintf(res, "yawP:%.5f rollP:%.5f pitchP:%.5f\n\0", attitudeOut.yawOut, attitudeOut.rollOut, attitudeOut.pitchOut);
	// RCS_USART_Send_Str(DEBUGUSART, res);
	// RCS_USART_Send_Char(DEBUGUSART, '\n');

	if (THR < 0.501)
	{
		MOTOR_RF_Pwm = 0.49;
		MOTOR_RB_Pwm = 0.49;
		MOTOR_LB_Pwm = 0.49;
		MOTOR_LF_Pwm = 0.49;
	}

	PWMOutput(MOTOR_RF_TIM, MOTOR_RF_CH, MOTOR_RF_Pwm);
	PWMOutput(MOTOR_RF_TIM, MOTOR_RB_CH, MOTOR_RB_Pwm);
	PWMOutput(MOTOR_RF_TIM, MOTOR_LB_CH, MOTOR_LB_Pwm);
	PWMOutput(MOTOR_RF_TIM, MOTOR_LF_CH, MOTOR_LF_Pwm);
}


