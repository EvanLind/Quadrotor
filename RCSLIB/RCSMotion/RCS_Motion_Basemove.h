/*
 *@author : kgolym
 *@date : 2012-10-01
 *@brief: The base motion for the robot
 */
#ifndef _RCS_BASEMOVE_H_
#define _RCS_BASEMOVE_H_
#include "bsp.h"
#include "RCS_Pwm.h"
/*
 *  @brief 初始化三角底盘模型
 */
void InitBaseMotionModule();
/*
 * @brief 瞬停，无加速度控制
 */
void BaseMotion_Stop();

/*
 * @brief 移动,使用时间来进行加速度控制。
 * @param _linerSpeedAngle:速度与正方向的夹角。逆时针为正
 * @param _LinerSpeed:速度的标量大小，单位，mm/s , 可输入0代表停止，不为负
 * @param _angleSpeed： 角速度，逆时针为正，单位度/s
 * @param _accTime:加速时间，单位秒
 */
void BaseMotion_Move(float _linerSpeedAngle, float _LinerSpeed,
					 float _angleSpeed,float _accTime);

#endif //_RCS_BASEMOVE_H_