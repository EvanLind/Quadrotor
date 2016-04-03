/*
 *@author：张翼
 *@brief:模糊控制
 *@date:2012-11-10
 */
#ifndef _RCS_FUZZYMOVE_H_
#define _RCS_FUZZYMOVE_H_
#include "bsp.h"

#define N_POINT 10
extern volatile Boolean g_isBusy;
extern volatile uint8_t g_CurrentPoint ;
extern volatile double g_speedRate;
//@brief:判断模糊是否在运行。（处于暂停态也会返回true）
inline Boolean FuzzyIsProcessing()
{
    return g_isBusy;
}
//@brief:判断模糊运行到的点。
inline uint8_t FuzzyCurrentPoint()
{
    return g_CurrentPoint;
}

//@brief:模糊初始化
void Fuzzy_Init();
void Fuzzy_Rotate(double _goalDeg, double _speed, double _error);
//@brief:设定模糊的目标参数，并启动模糊移动
//@param: _x[] 目标x
//@param: _y[] 目标y
//@param: _deg 目标姿态角
//@param: _maxLinerSpeed 最大线速度
//@param: _minStartSpeed 最小启动速度 建议小一点。
//@param: _startAcc 启动加速度
//@param: _stopAcc 停止加速度
//@param :_n 点的个数
void Fuzzy_InitPoints(int32_t _x[], int32_t _y[], double _deg,
                      double _maxLinerSpeed, double _minStartSpeed,
                      double _startAcc, double _stopAcc, uint8_t _n);

void Fuzzy_SetPointSpeed(uint8_t _n, double _speed, double _dist, double _startAcc, double _stopAcc);
void Fuzzy_SetDeg(uint8_t _n, double _deg);
void Fuzzy_Start();
void Hermite_Init(float rx1, float ry1, float rxN, float ryN,
                  float px[], float py[], float u, 
                  float ppx[][N_POINT], float ppy[][N_POINT]);
void Fuzzy_ReCalculate();
void Fuzzy_Pause();
void Fuzzy_PauseAndStop();
//@brief: 从暂停态恢复
void Fuzzy_Resume();
//@brief：停止模糊
void Fuzzy_Stop();
Boolean IsStop();
Boolean Is_My_Stop(uint32_t mystop_err);
uint8_t GetNowPoint();
#endif  // _RCS_FUZZYMOVE_H_

