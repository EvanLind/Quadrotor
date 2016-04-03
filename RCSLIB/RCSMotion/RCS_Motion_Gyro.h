/*
 *@author：柯国霖
 *@brief:陀螺仪角度获取和计算
 *@date:2012-10-10
 */

#ifndef _RCS_GYRO_H_
#define _RCS_GYRO_H_
#include "bsp.h"
#include "rcs.h"

extern volatile double  g_deg;
extern volatile Boolean  g_canUsed;
inline double GetDeg()
{
	return g_deg;
}
inline Boolean GyroUsable()
{
	return g_canUsed;
}

// @brief : 初始化陀螺仪
// @param : _TimeDiv 多久调用一次中断
void InitGyro(double _TimeDiv);
//@brief:获取陀螺仪的累计角度
double CalculateGyroDeg(void);
#endif // _RCS_GYRO_H_