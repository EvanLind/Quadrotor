/*
 *@author：柯国霖
 *@brief:全场定位
 *@date:2012-10-10
 */

#ifndef _RCS_GPS_H_
#define _RCS_GPS_H_
#include "bsp.h"
extern volatile double g_X;
extern volatile double g_Y;

extern volatile double g_linerSpeed;
extern volatile double g_AnglurSpeed;
extern volatile double g_NowlinerSpeedAngle;
void InitGPS();
//@brief : 返回距离 距离(_x,_y) 的距离平方根
double PointDistanceSquare(int32_t _x, int32_t _y);
inline int32_t GetX()
{
    return (int32_t)g_X;
}
inline int32_t GetY()
{
    return (int32_t)g_Y;
}

inline double GetLinerSpeed()
{
    return g_linerSpeed;
}
inline double GetNowLinerSpeedAngle()
{
    return g_NowlinerSpeedAngle;
}
inline double GetAnglurSpeed()
{
    return g_AnglurSpeed;
}

extern volatile Boolean g_isRotate;
inline Boolean IsRotate()
{
    return g_isRotate;
}
#endif //  _RCS_GPS_H_