/*
 *@author：柯国霖
 *@brief:全场定位
 *@date:2012-10-10
 */

#include "rcs.h"
#include "math.h"
#include "RCS_CommonFunction.h"
/*
 *一般约定
 *安装方式如下：
 *                   G
 *         |  /      |
 *         |C/       |
 *         |/        |
 *         -----------
 *       #
 *     #   #
 *   #       #
 *  e2       e1
 *
 *
 */




#define GPS_TIMER_PPR       100
#define GPS_TIMER_DIV       10000

#define ROTATE_MESURE   1e-4

volatile double g_X;
volatile double g_Y;
volatile Boolean g_isRotate = FALSE;
volatile double g_linerSpeed;
volatile double g_NowlinerSpeedAngle = 0;
volatile double g_AnglurSpeed;

static volatile double Encoder1_RotateFact = 0;
static volatile double Encoder2_RotateFact = 0;
static volatile double Encoder3_RotateFact = 0;

static void GPS_ISR();
//@brief: 计算码盘的旋转消去参数
inline void CalculateRotateFact()
{

	Encoder1_RotateFact = Encoder1_RotateFact_g;
	Encoder2_RotateFact = Encoder2_RotateFact_g;
	Encoder3_RotateFact = Encoder3_RotateFact_g;
}

void InitGPS()
{
	CalculateRotateFact();
	OSTimeDly(500);
        InitEncoders();
	InitGyro(GPS_TIMER_PPR / (double)GPS_TIMER_DIV);
	g_X = START_X ;
	g_Y = START_Y ;
	InitTimerInt(GPS_TIMER, GPS_TIMER_PPR, GPS_TIMER_DIV, GPS_ISR, 0x10);
}
double PointDistanceSquare(int32_t _x,int32_t _y)
{
	double dx = g_X - _x;
	double dy =g_Y - _y;
	return dx * dx + dy * dy;
}
static void GPS_ISR()
{
	TIM_ClearITPendingBit(GPS_TIMER, TIM_IT_Update);
	double l1 = (double)GetEncoder1Count() * Encoder1_Fact * Encoder1_DIR;
	ClearEncoder1Count();
	double l2 = (double)GetEncoder2Count() * Encoder2_Fact * Encoder2_DIR;
	ClearEncoder2Count();
#ifndef TWO_ENCODERS
	double l3 = (double)GetEncoder3Count() * Encoder3_Fact * Encoder3_DIR;
	ClearEncoder3Count();
#endif
	static double lastV = 0;

#ifdef TWO_ENCODERS
	if ( ABS(l1) < ROTATE_MESURE && ABS(l2) < ROTATE_MESURE)
	{
		g_isRotate = FALSE;
	}
	else
	{
		g_isRotate = TRUE;
	}
#else
	if ( ABS(l1 - l3) < ROTATE_MESURE )
	{
		g_isRotate = FALSE;
	}
	else
	{
		g_isRotate = TRUE;
	}
#endif

	static double LastDeg = 0;
	double currentDeg = CalculateGyroDeg();
	double deltaDeg = currentDeg - LastDeg ;
	double degData = (LastDeg + currentDeg);
	if (deltaDeg > 180 )
	{
		deltaDeg -= 360;
		degData += 360;
	}
	else if (deltaDeg < -180)
	{
		deltaDeg += 360;
		degData += 360;
	}
	g_AnglurSpeed = deltaDeg * 200;
	l1 -= deltaDeg * Encoder1_RotateFact;
	l2 -= deltaDeg * Encoder2_RotateFact;
	float _ts = l1 * l1 + l2 * l2;
	arm_sqrt_f32(_ts,&_ts);
	g_linerSpeed =  _ts * 200;
	lastV = g_linerSpeed;
#ifndef TWO_ENCODERS
	l3 -= deltaDeg * Encoder3_RotateFact;

	l1 = (l1 + l3) / 2.0;
#endif
	//计算等效的平移方向
	//
	degData = degData *0.5f + ANGLE_C;
	degData *= DEG2RAD;
	LastDeg = currentDeg;

	float degCosData, degSinData;
	degCosData = arm_cos_f32(degData);
	degSinData = arm_sin_f32(degData);
	double deltaX = l2 * degCosData -  l1 * degSinData ;
	double deltaY = l1 * degCosData +  l2 * degSinData ;

	g_NowlinerSpeedAngle = atan2(deltaY , deltaX) * RAD2DEG;
	g_X += deltaX;
	g_Y += deltaY;

}