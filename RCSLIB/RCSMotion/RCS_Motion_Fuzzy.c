/*
 *@author：张翼
 *@brief:模糊控制
 *@date:2012-11-10
 *@history:
 *2012-11-12    柯国霖   修改封装，去掉部分无用全局变量，规范函数名，
 *                      修正线速度方向计算的错误。增加stopzone和角速度倍率
 */

#include "rcs.h"
#include "math.h"
#include "RCS_CommonFunction.h"


#define FUZZY_POS_STOP  6       // r ，停止点位置误差
#define FUZZY_DEG_STOP  0.3      // deg ，停止点角度误差
#define FUZZY_STOP_COUNT 3
#define FUZZY_K_NEXT 1.57 //pi/2
#define FUZZY_POS_NEXT  50
#define FUZZY_DEG_NEXT  5      // deg ，中转点角度误差
#define FUZZY_NEXT_COUNT 1

#define MIN_ANGLE_SPEED 7    //最小旋转速度
#define MAX_ANGLE_SPEED 70
#define FUZZY_CTRL_TIME 10      //模糊控制的时间间隔
#define ANGLE_SPEED_RATE    1.1    //角速度放大倍率
#define MAX_POINT       50      // 最大点数
#define STOP_ZONE       30           //接近终点stopzone的范围内，以minStopSpeed匀速调整。

#define NO_ROTATE   -999

#define FIRST_ANGLE_RANGE 2
#define SECOND_ANGLE_RANGE 4
#define FIRST_ANGLE_SPEED 2
#define SECOND_ANGLE_SPEED 6
static const double minStopSpeed = 35;

#pragma data_alignment=8
static OS_STK FuzzyMove_task_stk[NORMAL_TASK_STK_SIZE];


volatile uint8_t g_CurrentPoint = 0; //当前在跑的目标点
volatile Boolean g_isBusy = TRUE;
volatile double tmpViewAngleV;
volatile double tmpViewSpeedAngle;
volatile double tmpViewSpeed;

static volatile int32_t MulGoalX[MAX_POINT];    // 多点目标X[]
static volatile int32_t MulGoalY[MAX_POINT];    // 多点目标Y[]
static volatile double MulGoalDeg[MAX_POINT];   // 多点目标deg[]
static volatile uint16_t MulGoalDegIndex[MAX_POINT];   // 多点目标deg[]
static volatile double MulRestDist[MAX_POINT];  // 多点目标剩余距离 restDist[]
static volatile double MulRunDist[MAX_POINT];   // 多点目标已跑距离 runDist[]
static volatile double MulGoalAccDist[MAX_POINT];   // 多点目标剩余距离 restDist[]
static volatile double MulGoalDecDist[MAX_POINT];   // 多点目标已跑距离 runDist[]
static volatile double MulGoalSpeed[MAX_POINT]; //多点模糊目标点速度
static volatile double MulGoalMaxSpeed[MAX_POINT];
static volatile double MulGoalMaxSpeedDist[MAX_POINT];
static volatile double MulGoalK[MAX_POINT];
static volatile double MulGoalSpeedDist[MAX_POINT];
static volatile double MulGoalStartAcc[MAX_POINT];
static volatile double MulGoalStopAcc[MAX_POINT];
static volatile double MulGoalResttime[MAX_POINT];
static volatile uint8_t CurrentPointN = 0; //点的个数

static volatile double goal_Deg;
static volatile double rotate_Speed;
static volatile double rotate_error;

static volatile Boolean g_setToPause = FALSE; // 暂停标致位
static volatile Boolean g_setToStop = FALSE;  //停止标志位

static volatile Boolean g_RotateTask = FALSE;
static OS_EVENT *fuzzySem;
static volatile double startAcc = 20; //启动加速度
static volatile double stopAcc = 10; //停止加速度
static volatile double minStartSpeed = 10; //最小启动速度
static volatile double maxLinerSpeed = 50; //最大线速度

static volatile int32_t currentX = 0;  //当前坐标的x
static volatile int32_t currentY = 0;  //当前坐标的y
static volatile double currentDeg = 0; //当前姿态角
static volatile double runDist = 0 ;   //已经走过的距离
static volatile double restDist = 0 ;  //剩余距离
static volatile double NextRestDist = 0;
static volatile double LastRunDist = 0;
static volatile double currentResttime = 0;

static volatile double accDist = 0; //理论的加速距离
static volatile double decDist = 0; //理论的减速距离
static volatile double maxSpeedDist = 0; //理论的最大速度距离



extern volatile int Change_flag;
//模糊控制任务
static void FuzzyMove_Task(void *p_arg);
//理论距离估算
static void DistEstimate(double _maxAccDist, double _maxDecDist, double _maxLinerSpeed);

uint8_t GetNowPoint()
{
    return g_CurrentPoint;
}
//@brief:模糊初始化
void Fuzzy_Init()
{
    rotate_Speed = 10;
    fuzzySem = OSSemCreate(0);
    OSTaskCreate(FuzzyMove_Task,
                 (void *)0,
                 &FuzzyMove_task_stk[NORMAL_TASK_STK_SIZE - 1],
                 MotionTask_PRIO);
}
void Fuzzy_Rotate(double _goalDeg, double _speed, double _error)
{
    goal_Deg = _goalDeg;
    rotate_Speed = _speed;
    rotate_error = _error;
    g_RotateTask = TRUE;
}

// rx1,  ry1,  rxN,  ryN,分别是首端和末端的切向量
// px[],py[]是跑点坐标，u是张力参数，N_POINT是坐标个数,在.h中定义
// ppx[][],ppy[][]是处理后的坐标,ppx[0][3]表示第一段中的第4个点

void Hermite_Init(float rx1, float ry1, float rxN, float ryN,
                  float px[], float py[], float u, 
                  float ppx[][N_POINT], float ppy[][N_POINT])
{
    float t, t2, t3, express, px0, py0, px1, py1,h0,h1,h2,h3;
    float rxf,ryf,rxl,ryl;
    int n, count;

    for (n = 1; n <= N_POINT - 3; n++)
    {
        px0 = px[n];
        py0 = py[n];
        px1 = px[n + 1];
        py1 = py[n + 1];
        rxf = (1 - u) * (px[n + 1] - px[n - 1]);
        ryf = (1 - u) * (py[n + 1] - py[n - 1]);
        rxl = (1 - u) * (px[n + 2] - px[n]);
        ryl = (1 - u) * (py[n + 2] - py[n]);

        for (t = 0, count = 0; count < N_POINT; t += 1.0f/N_POINT, count ++)
        {
            t2 = t * t;
            t3 = t2 * t;
            express = 3 * t2 - 2 * t3;
            h0 = 1 - express;
            h1 = express;
            h2 = t - 2 * t2 + t3;
            h3 = t3 - t2;
            if (n == 1)
            {
                ppx[n - 1][count] = px[0] * h0 + px[1] * h1 + rx1 * h2 + rxf * h3;
                ppy[n - 1][count] = py[0] * h0 + py[1] * h1 + ry1 * h2 + ryf * h3;

            }
            else if (n == N_POINT - 3)
            {
                ppx[n - 1][count] = px[N_POINT - 2] * h0 + px[N_POINT - 1] * h1 + rxl * h2 + rxN * h3;
                ppy[n - 1][count] = py[N_POINT - 2] * h0 + py[N_POINT - 1] * h1 + ryl * h2 + ryN * h3;
            }
            ppx[n - 1][count] = px0 * h0 + px1 * h1 + rxf * h2 + rxl * h3;
            ppy[n - 1][count] = py0 * h0 + py1 * h1 + ryf * h2 + ryl * h3;
        }
    }
}



//@brief:设定模糊的目标参数，并启动模糊移动
//@param: _x[] 目标x
//@param: _y[] 目标y
//@param: _deg[] 目标姿态角
//@param: _maxLinerSpeed 最大线速度
//@param: _minStartSpeed 最小启动速度
//@param: _startAcc 启动加速度
//@param: _stopAcc 停止加速度
//@param :_n 点的个数

void Fuzzy_InitPoints(int32_t _x[], int32_t _y[], double _deg,
                      double _maxLinerSpeed, double _minStartSpeed,
                      double _startAcc, double _stopAcc, uint8_t _n)
{
    g_isBusy = TRUE;
    g_RotateTask = FALSE;
    CurrentPointN = _n ;
    g_CurrentPoint = 1;

    for (int i = 1; i <= CurrentPointN ; i++)
    {
        MulGoalX[i] = _x[i - 1];
        MulGoalY[i] = _y[i - 1];
        MulGoalDeg[i] = NO_ROTATE;
    }
    MulGoalDeg[CurrentPointN] = _deg;
    currentX = GetX();
    currentY = GetY();
    currentDeg = GetDeg();
    MulGoalX[0] = currentX;
    MulGoalY[0] = currentY;
    MulGoalDeg[0] = currentDeg;
    double maxV = _maxLinerSpeed;
    minStartSpeed = _minStartSpeed;
    startAcc = _startAcc;
    stopAcc = _stopAcc;
    double maxAccDist = (maxV * maxV - minStartSpeed * minStartSpeed) / (2 * startAcc);
    double maxDecDist = (maxV * maxV) / (2 * stopAcc);
    DistEstimate(maxAccDist, maxDecDist, maxV);
}
void Fuzzy_SetPointSpeed(uint8_t _n, double _speed, double _dist, double _startAcc, double _stopAcc)
{
    if (_n == 0)  return;
    MulGoalSpeed[_n] = _speed;
    MulGoalSpeedDist[_n] = _dist;
    if (MulGoalSpeedDist[_n] > MulRunDist[_n + 1] - MulRunDist[_n])
    {
        MulGoalSpeedDist[_n] = MulRunDist[_n + 1] - MulRunDist[_n];
        MulGoalSpeed[_n + 1] = _speed;
    }
    MulGoalStartAcc[_n ] = _startAcc;
    MulGoalStopAcc[_n ] = _stopAcc;
    MulGoalSpeed[CurrentPointN] = minStopSpeed;
}
void Fuzzy_SetDeg(uint8_t _n, double _deg)
{
    if (_n == 0)  return;
    MulGoalDeg[_n] = _deg;
}
void Fuzzy_ReCalculate()
{
    if (!g_RotateTask)
    {
        uint8_t lastIndex = CurrentPointN;
        MulGoalDegIndex[CurrentPointN] = CurrentPointN;
        for (int i = CurrentPointN - 1; i >= 0 ; i--)
        {
            if (MulGoalDeg[i] != NO_ROTATE )
            {
                lastIndex = i;

            }
            MulGoalDegIndex[i] = lastIndex;
        }
        for (int i = 0; i < CurrentPointN ; ++i)
        {
            double tdist = MulRunDist[i + 1] - MulRunDist[i] - MulGoalSpeedDist[i];
            double taccdist = ABS(maxLinerSpeed * maxLinerSpeed - MulGoalSpeed[i] * MulGoalSpeed[i]) / (2 * MulGoalStartAcc[i]);
            double tdecdist = ABS(maxLinerSpeed * maxLinerSpeed - MulGoalSpeed[i + 1] * MulGoalSpeed[i + 1]) / (2 * MulGoalStopAcc[i]);
            if (tdist < (taccdist + tdecdist) )
            {
                float fs = (2 * MulGoalStartAcc[i] * MulGoalStopAcc[i] * tdist
                            + MulGoalStopAcc[i] * MulGoalSpeed[i] * MulGoalSpeed[i]
                            + MulGoalStartAcc[i] * MulGoalSpeed[i + 1] * MulGoalSpeed[i + 1]
                           )
                           / (MulGoalStartAcc[i] + MulGoalStopAcc[i]);
                MulGoalDecDist[i] = (fs - MulGoalSpeed[i + 1] * MulGoalSpeed[i + 1]) / (2 * MulGoalStopAcc[i]);
                if (MulGoalDecDist[i] < 0)
                {
                    MulGoalDecDist[i] = 0;
                }
                MulGoalAccDist[i] = tdist - MulGoalDecDist[i];
                if (MulGoalAccDist[i] < 0)
                {
                    MulGoalAccDist[i] = 0;
                }
                arm_sqrt_f32(fs, &fs);
                MulGoalMaxSpeed[i] = fs;
                MulGoalMaxSpeedDist[i] = 0;
            }
            else
            {
                MulGoalAccDist[i] = taccdist;
                MulGoalDecDist[i] = tdecdist;
                MulGoalMaxSpeed[i] = maxLinerSpeed;
                MulGoalMaxSpeedDist[i] = tdist - MulGoalAccDist[i] - MulGoalDecDist[i];
                if (MulGoalMaxSpeedDist[i] < 0)
                {
                    MulGoalMaxSpeedDist[i] = 0;
                }
            }
        }
        MulGoalResttime[CurrentPointN] = 0;
        for (int i = CurrentPointN - 1 ; i >= 0 ; i--)
        {
            double t =   ABS(MulGoalSpeedDist[i]) / MulGoalSpeed[i];
            t += ABS(MulGoalMaxSpeed[i] - MulGoalSpeed[i]) / MulGoalStartAcc[i];
            t += ABS(MulGoalMaxSpeed[i] - MulGoalSpeed[i + 1]) / MulGoalStopAcc[i];
            t += ABS(MulGoalMaxSpeedDist[i] ) / MulGoalMaxSpeed[i];
            MulGoalResttime[i] = t + MulGoalResttime[i + 1];
        }
    }
}
//@brief: 模糊开始
void Fuzzy_Start()
{
    if (!g_RotateTask)
    {
        uint8_t lastIndex = CurrentPointN;
        MulGoalDegIndex[CurrentPointN] = CurrentPointN;
        for (int i = CurrentPointN - 1; i >= 0 ; i--)
        {
            if (MulGoalDeg[i] != NO_ROTATE )
            {
                lastIndex = i;

            }
            MulGoalDegIndex[i] = lastIndex;
        }
        for (int i = 0; i < CurrentPointN ; ++i)
        {
            double tdist = MulRunDist[i + 1] - MulRunDist[i] - MulGoalSpeedDist[i];
            double taccdist = ABS(maxLinerSpeed * maxLinerSpeed - MulGoalSpeed[i] * MulGoalSpeed[i]) / (2 * MulGoalStartAcc[i]);
            double tdecdist = ABS(maxLinerSpeed * maxLinerSpeed - MulGoalSpeed[i + 1] * MulGoalSpeed[i + 1]) / (2 * MulGoalStopAcc[i]);
            if (tdist < (taccdist + tdecdist) )
            {
                float fs = (2 * MulGoalStartAcc[i] * MulGoalStopAcc[i] * tdist
                            + MulGoalStopAcc[i] * MulGoalSpeed[i] * MulGoalSpeed[i]
                            + MulGoalStartAcc[i] * MulGoalSpeed[i + 1] * MulGoalSpeed[i + 1]
                           )
                           / (MulGoalStartAcc[i] + MulGoalStopAcc[i]);
                MulGoalDecDist[i] = (fs - MulGoalSpeed[i + 1] * MulGoalSpeed[i + 1]) / (2 * MulGoalStopAcc[i]);
                if (MulGoalDecDist[i] < 0)
                {
                    MulGoalDecDist[i] = 0;
                }
                MulGoalAccDist[i] = tdist - MulGoalDecDist[i];
                if (MulGoalAccDist[i] < 0)
                {
                    MulGoalAccDist[i] = 0;
                }
                arm_sqrt_f32(fs, &fs);
                MulGoalMaxSpeed[i] = fs;
                MulGoalMaxSpeedDist[i] = 0;
            }
            else
            {
                MulGoalAccDist[i] = taccdist;
                MulGoalDecDist[i] = tdecdist;
                MulGoalMaxSpeed[i] = maxLinerSpeed;
                MulGoalMaxSpeedDist[i] = tdist - MulGoalAccDist[i] - MulGoalDecDist[i];
                if (MulGoalMaxSpeedDist[i] < 0)
                {
                    MulGoalMaxSpeedDist[i] = 0;
                }
            }
        }
        MulGoalResttime[CurrentPointN] = 0;
        for (int i = CurrentPointN - 1 ; i >= 0 ; i--)
        {
            double t =   ABS(MulGoalSpeedDist[i]) / MulGoalSpeed[i];
            t += ABS(MulGoalMaxSpeed[i] - MulGoalSpeed[i]) / MulGoalStartAcc[i];
            t += ABS(MulGoalMaxSpeed[i] - MulGoalSpeed[i + 1]) / MulGoalStopAcc[i];
            t += ABS(MulGoalMaxSpeedDist[i] ) / MulGoalMaxSpeed[i];
            MulGoalResttime[i] = t + MulGoalResttime[i + 1];
        }
    }
    OSSemPost(fuzzySem);
}
//@brief: 模糊暂停
void Fuzzy_Pause()
{
    g_setToPause = TRUE;
}
//@brief: 模糊暂停并停止移动
void Fuzzy_PauseAndStop()
{
    g_setToPause = TRUE;
    if (g_setToPause)
    {
        BaseMotion_Stop();
    }
}
//@brief: 从暂停态恢复
void Fuzzy_Resume()
{
    g_setToPause = FALSE;
}
//@brief：停止模糊
void Fuzzy_Stop()
{
    g_setToStop = TRUE;
    if (g_setToStop)
    {
        BaseMotion_Stop();
    }
    Change_flag = 0;
}



/*
    inline functions
*/
inline double GetDistance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    float fs = dx * dx + dy * dy ;
    arm_sqrt_f32(fs, &fs);
    return fs;
}
inline double GetCalLinerSpeed()
{
    if (restDist < STOP_ZONE)
    {
        currentResttime = NextRestDist / minStopSpeed;
        return minStopSpeed;
    }
    if (NextRestDist < MulGoalDecDist[g_CurrentPoint - 1])
    {

        float32_t fs = (float32_t)(2 * MulGoalStopAcc[g_CurrentPoint - 1] * NextRestDist ) + MulGoalSpeed[g_CurrentPoint] * MulGoalSpeed[g_CurrentPoint];
        arm_sqrt_f32(fs, &fs);
        currentResttime = ABS(fs - MulGoalSpeed[g_CurrentPoint]) / MulGoalStopAcc[g_CurrentPoint - 1];
        return fs;
    }
    else if (LastRunDist < MulGoalSpeedDist[g_CurrentPoint - 1])
    {
        currentResttime =   ABS(MulGoalSpeedDist[g_CurrentPoint - 1] - LastRunDist) / MulGoalSpeed[g_CurrentPoint - 1];
        currentResttime += ABS(MulGoalMaxSpeed[g_CurrentPoint - 1] - MulGoalSpeed[g_CurrentPoint - 1]) / MulGoalStartAcc[g_CurrentPoint - 1];
        currentResttime += ABS(MulGoalMaxSpeed[g_CurrentPoint - 1] - MulGoalSpeed[g_CurrentPoint]) / MulGoalStopAcc[g_CurrentPoint - 1];
        currentResttime += ABS(MulGoalMaxSpeedDist[g_CurrentPoint - 1] ) / MulGoalMaxSpeed[g_CurrentPoint - 1];
        return  MulGoalSpeed[g_CurrentPoint - 1];
    }
    else if (LastRunDist < MulGoalAccDist[g_CurrentPoint - 1])
    {

        float32_t fs = (float32_t)(2 * MulGoalStartAcc[g_CurrentPoint - 1] * (LastRunDist - MulGoalSpeedDist[g_CurrentPoint - 1]) ) + MulGoalSpeed[g_CurrentPoint - 1] * MulGoalSpeed[g_CurrentPoint - 1];
        arm_sqrt_f32(fs, &fs);
        currentResttime = ABS(MulGoalMaxSpeed[g_CurrentPoint - 1] - fs) / MulGoalStartAcc[g_CurrentPoint - 1];
        currentResttime += ABS(MulGoalMaxSpeed[g_CurrentPoint - 1] - MulGoalSpeed[g_CurrentPoint]) / MulGoalStopAcc[g_CurrentPoint - 1];
        currentResttime += ABS(MulGoalMaxSpeedDist[g_CurrentPoint - 1] ) / MulGoalMaxSpeed[g_CurrentPoint - 1];

        return fs;
    }
    else
    {
        currentResttime = ABS(MulGoalMaxSpeed[g_CurrentPoint - 1] - MulGoalSpeed[g_CurrentPoint]) / MulGoalStopAcc[g_CurrentPoint - 1];
        currentResttime += ABS(MulGoalMaxSpeedDist[g_CurrentPoint - 1]  + MulGoalSpeedDist[g_CurrentPoint - 1] - LastRunDist ) / MulGoalMaxSpeed[g_CurrentPoint - 1];

        return MulGoalMaxSpeed[g_CurrentPoint - 1];
    }
}

// 获取角速度
inline double GetAngleSpeed(double _deltaDeg, double _linerSpeed)
{
    //if(runDist < accDist / 2) return 0;
    if (_deltaDeg > 180 )
    {
        _deltaDeg -= 360;
    }
    else if (_deltaDeg < -180)
    {
        _deltaDeg += 360;
    }

    double _restTime = ABS(MulGoalResttime[g_CurrentPoint] - MulGoalResttime[MulGoalDegIndex[g_CurrentPoint]] + currentResttime);
    if ( ABS(_deltaDeg) < FUZZY_DEG_STOP)   // 角度够了
    {
        return 0 ;
    }
    else if (ABS(_deltaDeg) > FUZZY_DEG_STOP && restDist < STOP_ZONE)
    {
        return MIN_ANGLE_SPEED * _deltaDeg / ABS(_deltaDeg);    // 用最小速度旋转
    }
    else    // 正常情况
    {
        double angleV = ANGLE_SPEED_RATE * _deltaDeg / _restTime;
        // if (GetLinerSpeed() < maxLinerSpeed)
        // {
        //     angleV *=  (maxLinerSpeed / (GetLinerSpeed() + _linerSpeed) * 2);
        // }
        if ( ABS(MAX_ANGLE_SPEED) < ABS(angleV))
        {
            if (angleV > 0)
            {
                angleV =  MAX_ANGLE_SPEED ;
            }
            else
            {
                angleV =  -MAX_ANGLE_SPEED ;
            }
        }

        return angleV;
    }
}
inline double GetLinerSpeedAngle(int32_t _goalX, int32_t _goalY)
{
    double dx = _goalX - currentX;
    double dy = _goalY - currentY;
    double tk = atan2(dy, dx);
    if ( ABS(tk - MulGoalK[g_CurrentPoint]) > FUZZY_K_NEXT)
    {
        // tk = MulGoalK[g_CurrentPoint];
    }
    return tk * RAD2DEG - 90 - currentDeg;
}
//@brief:更新当前坐标信息
inline void UpdateData()
{
    currentX = GetX();
    currentY = GetY();
    currentDeg = GetDeg();

    runDist = MulRunDist[g_CurrentPoint - 1] + GetDistance(currentX, currentY, MulGoalX[g_CurrentPoint - 1], MulGoalY[g_CurrentPoint - 1]);
    LastRunDist = runDist - MulRunDist[g_CurrentPoint - 1];
    NextRestDist = GetDistance(currentX, currentY, MulGoalX[g_CurrentPoint], MulGoalY[g_CurrentPoint]);
    restDist = NextRestDist + MulRestDist[g_CurrentPoint];
    maxSpeedDist = runDist + restDist - decDist - accDist;


}

//@brief : 模糊停止的条件
Boolean IsStop()
{
    static uint8_t count = 0;
    double tempDeg = ABS(MulGoalDeg[g_CurrentPoint] - currentDeg);
    if ((restDist < (uint32_t)FUZZY_POS_STOP) && (tempDeg < FUZZY_DEG_STOP))
    {
        count++;
    }
    else
    {
        count = 0;
    }
    if ( count > FUZZY_STOP_COUNT)
    {
        return TRUE;
    }
    return FALSE;
}

Boolean Is_My_Stop(uint32_t mystop_err)
{
    static uint8_t count = 0;
    if ((restDist < (uint32_t)mystop_err))
    {
        count++;
    }
    else
    {
        count = 0;
    }
    if ( count > FUZZY_STOP_COUNT)
    {
        return TRUE;
    }
    return FALSE;
}

//@brief : 跳到下一个点的条件
inline Boolean IsNext()
{
    double tmpk2 = atan2(currentY - MulGoalY[g_CurrentPoint] , currentX - MulGoalX[g_CurrentPoint] );
    if ( (ABS(tmpk2 - MulGoalK[g_CurrentPoint]) < FUZZY_K_NEXT || NextRestDist <  FUZZY_POS_NEXT) )
    {
        return TRUE;
    }
    return FALSE;
}

inline void MoveHelper()
{
    double linerV = 0.0;
    double angleV = 0 ;
    double linerAngle = 0;
    //double deltaDeg = 0;
    //double restTime = 0;
    linerV = GetCalLinerSpeed();
    angleV = GetAngleSpeed(MulGoalDeg[MulGoalDegIndex[g_CurrentPoint]] - currentDeg, linerV);
    linerAngle = GetLinerSpeedAngle(MulGoalX[g_CurrentPoint], MulGoalY[g_CurrentPoint]);
    tmpViewSpeedAngle = linerAngle;
    tmpViewSpeed = linerV;
    tmpViewAngleV = angleV;
    BaseMotion_Move((float)linerAngle, (float)linerV, (float)angleV, 0);
}
inline Boolean RotateHelper()
{
    currentDeg = GetDeg();
    double deltaDeg = goal_Deg - currentDeg;
    if (deltaDeg > 180 )
    {
        deltaDeg -= 360;
    }
    else if (deltaDeg < -180)
    {
        deltaDeg += 360;
    }
    double angleV = 0;
    if (ABS(deltaDeg) < rotate_error )
    {
        angleV = 0;
        BaseMotion_Stop();
        return TRUE;
    }
    if ( ABS(deltaDeg) < FUZZY_DEG_STOP)   // 角度够了
    {
        angleV = 0;
        BaseMotion_Stop();
        return TRUE;
    }
    else if ( ABS(deltaDeg) < FIRST_ANGLE_RANGE)
    {
        if (deltaDeg > 0 )
        {
            angleV = FIRST_ANGLE_SPEED;
        }
        else
        {
            angleV = -FIRST_ANGLE_SPEED;
        }
    }
    else if ( ABS(deltaDeg) < SECOND_ANGLE_RANGE)
    {
        if (deltaDeg > 0 )
        {
            angleV = SECOND_ANGLE_SPEED;
        }
        else
        {
            angleV = -SECOND_ANGLE_SPEED;
        }
    }
    else
    {
        if (deltaDeg > 0 )
        {
            angleV = rotate_Speed;
        }
        else
        {
            angleV = -rotate_Speed;
        }
    }

    BaseMotion_Move(0, 0, angleV, 0);
    return FALSE;
}
/*
    end of inline  functions
*/

static void FuzzyMove_Task(void *p_arg)
{
    while (TRUE)
    {
        OSSemPend(fuzzySem, 0, (void *)0);
        g_isBusy = TRUE;
        g_setToPause = FALSE;
        g_setToStop = FALSE;
        // 遍历完成多点路径
        if (g_RotateTask)
        {
            while (1)
            {
                if (g_setToPause)
                {
                    OSTimeDly(FUZZY_CTRL_TIME);
                    continue;
                }
                if (g_setToStop)
                {
                    BaseMotion_Stop();
                    break;
                }
                if (RotateHelper())
                {
                    break;
                }
                OSTimeDly(FUZZY_CTRL_TIME);
            }
        }
        else
        {
            for (g_CurrentPoint = 1; g_CurrentPoint <= CurrentPointN ; ++g_CurrentPoint)
            {
                while (TRUE)
                {
                    int32_t time = OSTimeGet();
                    // 是否暂停
                    if (g_setToPause)
                    {
                        OSTimeDly(FUZZY_CTRL_TIME);
                        continue;
                    }
                    UpdateData();
                    // (到达最后一个点 && 到达停止条件) || 人工设置停止
                    if (g_CurrentPoint == CurrentPointN && IsStop() || g_setToStop)
                    {
                        BaseMotion_Stop();
                        break;
                    }
                    // (还未到达最后一个点 && 正在继续) || 人工设置停止
                    else if (g_CurrentPoint < CurrentPointN  && IsNext()  || g_setToStop )
                    {
                        break;
                    }
                    else
                    {
                        MoveHelper();
                    }
                    time = OSTimeGet() - time;
                    if (time < FUZZY_CTRL_TIME)
                    {
                        OSTimeDly(FUZZY_CTRL_TIME - time);
                    }
                }
            }
        }



        g_isBusy = FALSE;
    }
}


//@brief : 估计加速距离和减速距离和真正的最大速度
static void DistEstimate(double _maxAccDist, double _maxDecDist, double _maxLinerSpeed)
{

    MulRunDist[0] = 0;
    for (int i = 1; i <= CurrentPointN; ++i)
    {
        MulRunDist[i] = MulRunDist[i - 1] +
                        GetDistance(MulGoalX[i - 1], MulGoalY[i - 1],
                                    MulGoalX[i], MulGoalY[i]);
    }
    MulRestDist[CurrentPointN] = 0;
    for (int i = CurrentPointN - 1 ; i >= 0; i-- )
    {
        MulRestDist[i] =  MulRestDist[i + 1] +
                          GetDistance(MulGoalX[i + 1], MulGoalY[i + 1],
                                      MulGoalX[i], MulGoalY[i]);
    }
    restDist = MulRestDist[0];
    runDist = 0;


    if (restDist > (_maxAccDist + _maxDecDist))
    {
        accDist = _maxAccDist;
        decDist = _maxDecDist;
        maxLinerSpeed = _maxLinerSpeed;
        maxSpeedDist = restDist - accDist - decDist;
    }
    else
    {
        /*
         *  v^2 -v0^2 = 2*a1*s1     v^2 = 2*a2*s2
         */
        float fs = (2 * startAcc * stopAcc * restDist
                    + stopAcc * minStartSpeed * minStartSpeed
                    + startAcc * minStopSpeed * minStopSpeed
                   )
                   / (startAcc + stopAcc);
        decDist = (fs - minStopSpeed * minStopSpeed) / (2 * stopAcc);
        accDist = restDist - decDist;
        arm_sqrt_f32(fs, &fs);
        maxLinerSpeed = fs;
        maxSpeedDist = 0;
    }


    for (int i = 0; i <= CurrentPointN; ++i)
    {
        MulGoalSpeedDist[i] = 0;
        MulGoalStartAcc[i] = startAcc;
        MulGoalStopAcc[i] = stopAcc;
    }

    for (int i = 0; i <= CurrentPointN; ++i)
    {
        MulGoalSpeed[i] = maxLinerSpeed;
    }

    MulGoalSpeed[0] = minStartSpeed;
    for (int i = 1; i <= CurrentPointN; ++i)
    {
        if (MulRunDist[i] < accDist)
        {
            float32_t fs = (float32_t)(2 * startAcc * MulRunDist[i]  + MulGoalSpeed[0] * MulGoalSpeed[0]);
            arm_sqrt_f32(fs, &fs);
            MulGoalSpeed[i] = fs;
        }
        else
        {
            break;
        }
    }
    MulGoalSpeed[CurrentPointN] = minStopSpeed;

    for (int i = CurrentPointN - 1; i >= 0 ; i--)
    {
        if (MulRestDist[i] < decDist)
        {
            float32_t fs = (float32_t)(2 * stopAcc * MulRestDist[i] + MulGoalSpeed[CurrentPointN] * MulGoalSpeed[CurrentPointN]);
            arm_sqrt_f32(fs, &fs);
            MulGoalSpeed[i] = fs;
        }
        else
        {
            break;
        }
    }

    for (int i = 1; i <= CurrentPointN  ; i++)
    {
        MulGoalK[i] =  atan2(MulGoalY[i] - MulGoalY[i - 1], MulGoalX[i] - MulGoalX[i - 1]);

    }
}

