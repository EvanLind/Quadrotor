/*
 *@author : kgolym
 *@date : 2012-10-01
 *@brief: The base motion for the robot
 */

#include "rcs.h"
#include "math.h"
#include "RCS_CommonFunction.h"
#include "stm32f4xx_conf.h"


/*
 *  一般约定
 *  三个轮子如下布局（俯视图）

 * 即
 *      A
 *      .
 *      .
 *      .
 *     .  .
 *    .    .
 *   .      .
 *  B        C
 * 四轮底盘如下定义：
 * A -------- D
 * -          -
 * -          -
 * -          -
 * -          -
 * B-----------C
 */




const float MAX_MOTER_SPEED = MOTOR_SPEED_RPS * WHEEL_RADIUS * 2 * PI; //单个轮子的最大速度

/*
 *全局变量，加速度控制的加速度和目标速度
 */
//由于计数器的精度误差，直接获取的pwm占空比可能会一直不变。所以本地保存一个pwm
static volatile double wheelAPWMCurrent = 0.5;
static volatile double wheelBPWMCurrent = 0.5;
static volatile double wheelCPWMCurrent = 0.5;

//The goal of the pwm
static volatile double wheelAPWMGoal = 0.5;
static volatile double wheelBPWMGoal = 0.5;
static volatile double wheelCPWMGoal = 0.5;

//The acc of the pwm
static volatile double wheelAPWMAcc = 0;
static volatile double wheelBPWMAcc = 0;
static volatile double wheelCPWMAcc = 0;

#ifdef FOUR_WHEEL
static volatile double wheelDPWMCurrent = 0.5;
static volatile double wheelDPWMGoal = 0.5;
static volatile double wheelDPWMAcc = 0;
#endif
//加速度控制服务函数
static void CtrlSpeedISR();
static void BTMotion(uint8_t *data, uint8_t n);

double tmp123 = 0;
#ifdef FOUR_WHEEL
inline double GetMax(double _a, double _b, double _c , double _d)
{
    double max = ABS(_a) > ABS(_b) ? _a : _b;
    max = ABS(max) > ABS(_c) ? max : _c;
    max = ABS(max) > ABS(_d) ? max : _d;
    return max;
}
#else
inline double GetMax(double _a, double _b, double _c )
{
    double max = ABS(_a) > ABS(_b) ? _a : _b;
    max = ABS(max) > ABS(_c) ? max : _c;
    return max;
}
#endif
/*
 *  @brief 初始化三角底盘模型
 */
void InitBaseMotionModule()
{
    PWMInit(WHEEL_A_TIM, WHEEL_A_CH, WHEEL_A_GPIO, WHEEL_A_PIN, PWM_TIM_CLK, PWM_FREQ);
    PWMInit(WHEEL_B_TIM, WHEEL_B_CH, WHEEL_B_GPIO, WHEEL_B_PIN, PWM_TIM_CLK, PWM_FREQ);
    PWMInit(WHEEL_C_TIM, WHEEL_C_CH, WHEEL_C_GPIO, WHEEL_C_PIN, PWM_TIM_CLK, PWM_FREQ);
#ifdef FOUR_WHEEL
    PWMInit(WHEEL_D_TIM, WHEEL_D_CH, WHEEL_D_GPIO, WHEEL_D_PIN, PWM_TIM_CLK, PWM_FREQ);
#endif
    InitTimerInt(CTRLSPEED_TIMER, CTRLSPEED_TIMER_PPR, CTRLSPEED_TIMER_DIV, CtrlSpeedISR, 0x11);
    wheelAPWMCurrent = 0.5;
    wheelBPWMCurrent = 0.5;
    wheelCPWMCurrent = 0.5;
#ifdef FOUR_WHEEL
    wheelDPWMCurrent = 0.5;
#endif
    SetRCSComVect(0x20, BTMotion);
}

/*
 * @brief 瞬停，无加速度控制
 */
void BaseMotion_Stop()
{
    PWMOutput(WHEEL_A_TIM, WHEEL_A_CH, 0.5);
    PWMOutput(WHEEL_B_TIM, WHEEL_B_CH, 0.5);
    PWMOutput(WHEEL_C_TIM, WHEEL_C_CH, 0.5);
    wheelAPWMCurrent = 0.5;
    wheelBPWMCurrent = 0.5;
    wheelCPWMCurrent = 0.5;
    wheelAPWMGoal = 0.5;
    wheelBPWMGoal = 0.5;
    wheelCPWMGoal = 0.5;

#ifdef FOUR_WHEEL
    PWMOutput(WHEEL_D_TIM, WHEEL_D_CH, 0.5);
    wheelDPWMCurrent = 0.5;
    wheelDPWMGoal = 0.5;
#endif
}
/*
 * @brief 移动,使用时间来进行加速度控制。
 * @param _linerSpeedAngle:速度与正方向的夹角。逆时针为正
 * @param _LinerSpeed:速度的标量大小，单位，mm/s , 可输入0代表停止，不为负
 * @param _angleSpeed： 角速度，逆时针为正，单位度/s
 * @param _accTime:加速时间，单位秒
 */
void BaseMotion_Move(float _linerSpeedAngle, float _LinerSpeed,
                     float _angleSpeed, float _accTime)
{
    //保证参数的正确性
    assert_param(_LinerSpeed >= 0);
    assert_param(_accTime >= 0);
    double va, vb, vc;
#ifdef FOUR_WHEEL
    double vd;
#endif
    //停止未完成的加速度控制
    wheelAPWMAcc =  0;
    wheelBPWMAcc =  0;
    wheelCPWMAcc =  0;
#ifdef FOUR_WHEEL
    wheelDPWMAcc = 0;
#endif
    double linerSpeedAngle = _linerSpeedAngle * DEG2RAD;
    //计算轮子的分速度
#ifdef FOUR_WHEEL
    float tmpCos1 = arm_cos_f32( 0.75 * PI - linerSpeedAngle);
    float tmpCos2 = -arm_cos_f32( 0.25 * PI - linerSpeedAngle);
    va = tmpCos1;
    vb = tmpCos2;
    vc = - tmpCos1;
    vd = - tmpCos2;
#else
    va = -arm_cos_f32(linerSpeedAngle + PI * 0.5f);
    vb = -arm_cos_f32(linerSpeedAngle - PI / 6.0f);
    vc = -arm_cos_f32(linerSpeedAngle - 5.0f * PI / 6.0f );
#endif
    double speedK = _LinerSpeed / ( MAX_MOTER_SPEED );
    double w = _angleSpeed * DEG2RAD;
    double v = w * WHEEL_DISTANCE / MAX_MOTER_SPEED;
    if (ABS(v) >= MAX_ANGLE_PWM)
    {
        if(v > 0)
        {
            v = MAX_ANGLE_PWM;
        }
        else
        {
            v = - MAX_ANGLE_PWM;
        }
    }
    double w_pwm = WHEEL_A_DIR * v * 0.5;

    wheelAPWMGoal = 0.5 + w_pwm;
    wheelBPWMGoal = 0.5 + w_pwm;
    wheelCPWMGoal = 0.5 + w_pwm;

#ifdef FOUR_WHEEL
    wheelDPWMGoal = 0.5 + w_pwm;
    w_pwm *= 2;
    double lva = va * speedK * WHEEL_A_DIR ;
    double lvb = vb * speedK * WHEEL_B_DIR ;
    double lvc = vc * speedK * WHEEL_C_DIR ;
    double lvd = vd * speedK * WHEEL_D_DIR ;
    double MaxVPwm = GetMax(lva + w_pwm, lvb + w_pwm,
                            lvc + w_pwm, lvd + w_pwm);
#else
    w_pwm *= 2;
    double lva = va * speedK * WHEEL_A_DIR ;
    double lvb = vb * speedK * WHEEL_B_DIR ;
    double lvc = vc * speedK * WHEEL_C_DIR ;
    double MaxVPwm = GetMax(lva + w_pwm, lvb + w_pwm,
                            lvc + w_pwm);
#endif

    if (ABS(MaxVPwm) >= MAX_PWM) //速度溢出处理
    {
        double rate = (MAX_PWM - ABS(w_pwm)) / (ABS(MaxVPwm - w_pwm));
        lva *= rate;
        lvb *= rate;
        lvc *= rate;
#ifdef FOUR_WHEEL
        lvd *= rate;
#endif
    }
    wheelAPWMGoal += lva * 0.5;
    wheelBPWMGoal += lvb * 0.5;
    wheelCPWMGoal += lvc * 0.5;
#ifdef FOUR_WHEEL
    wheelDPWMGoal += lvd * 0.5;
#endif

    //确保速度不会溢出
    assert(wheelAPWMGoal < 1 && wheelAPWMGoal > 0);
    assert(wheelBPWMGoal < 1 && wheelBPWMGoal > 0);
    assert(wheelCPWMGoal < 1 && wheelCPWMGoal > 0);
#ifdef FOUR_WHEEL
    assert(wheelDPWMGoal < 1 && wheelDPWMGoal > 0);
#endif
    wheelAPWMCurrent = PWMGetPercent(WHEEL_A_TIM, WHEEL_A_CH);
    wheelBPWMCurrent = PWMGetPercent(WHEEL_B_TIM, WHEEL_B_CH);
    wheelCPWMCurrent = PWMGetPercent(WHEEL_C_TIM, WHEEL_C_CH);
#ifdef FOUR_WHEEL
    wheelDPWMCurrent = PWMGetPercent(WHEEL_D_TIM, WHEEL_D_CH);
#endif
    if (_accTime < FLOAT_ZERO )
    {
        PWMOutput(WHEEL_A_TIM, WHEEL_A_CH, wheelAPWMGoal);
        PWMOutput(WHEEL_B_TIM, WHEEL_B_CH, wheelBPWMGoal);
        PWMOutput(WHEEL_C_TIM, WHEEL_C_CH, wheelCPWMGoal);
#ifdef FOUR_WHEEL
        PWMOutput(WHEEL_D_TIM, WHEEL_D_CH, wheelDPWMGoal);
#endif
        wheelAPWMCurrent = PWMGetPercent(WHEEL_A_TIM, WHEEL_A_CH);
        wheelBPWMCurrent = PWMGetPercent(WHEEL_B_TIM, WHEEL_B_CH);
        wheelCPWMCurrent = PWMGetPercent(WHEEL_C_TIM, WHEEL_C_CH);
#ifdef FOUR_WHEEL
        wheelDPWMCurrent = PWMGetPercent(WHEEL_D_TIM, WHEEL_D_CH);
#endif
    }
    else
    {
        // double wheeltemp=GetMax(wheelAPWMGoal - wheelAPWMCurrent,wheelBPWMGoal - wheelBPWMCurrent,wheelCPWMGoal - wheelCPWMCurrent);
        // double blocktime=wheeltemp/0.03;//每秒改变0.03的输出
        // double tmpDiv = ((double)CTRLSPEED_TIMER_PPR / CTRLSPEED_TIMER_DIV) / blocktime;
        double tmpDiv = ((double)CTRLSPEED_TIMER_PPR / CTRLSPEED_TIMER_DIV) / _accTime;
        wheelAPWMAcc = (wheelAPWMGoal - wheelAPWMCurrent) * tmpDiv;
        wheelBPWMAcc = (wheelBPWMGoal - wheelBPWMCurrent) * tmpDiv;
        wheelCPWMAcc = (wheelCPWMGoal - wheelCPWMCurrent) * tmpDiv;

    }

}
//加速度控制服务函数


static void CtrlSpeedISR()
{
    TIM_ClearITPendingBit(CTRLSPEED_TIMER, TIM_IT_Update);
    double deltaA, deltaB, deltaC;
    double accAAbs, accBAbs, accCAbs;
#ifdef FOUR_WHEEL
    double deltaD, accDAbs;
#endif
    static uint16_t count = 0;
    //防止被其他东西干扰，隔断时间更新当前的pwm值
    if (count == 100)
    {
        wheelAPWMCurrent = PWMGetPercent(WHEEL_A_TIM, WHEEL_A_CH);
        wheelBPWMCurrent = PWMGetPercent(WHEEL_B_TIM, WHEEL_B_CH);
        wheelCPWMCurrent = PWMGetPercent(WHEEL_C_TIM, WHEEL_C_CH);
#ifdef FOUR_WHEEL
        wheelDPWMCurrent = PWMGetPercent(WHEEL_D_TIM, WHEEL_D_CH);
#endif
        count = 0;
    }
    deltaA = ABS(wheelAPWMCurrent - wheelAPWMGoal);
    deltaB = ABS(wheelBPWMCurrent - wheelBPWMGoal);
    deltaC = ABS(wheelCPWMCurrent - wheelCPWMGoal);
    accAAbs = ABS(wheelAPWMAcc);
    accBAbs = ABS(wheelBPWMAcc);
    accCAbs = ABS(wheelCPWMAcc);
#ifdef FOUR_WHEEL
    deltaD = ABS(wheelDPWMCurrent - wheelDPWMGoal);
    accDAbs = ABS(wheelDPWMAcc);
#endif

    if ( deltaA > accAAbs )
    {
        if (accAAbs > FLOAT_ZERO)
        {

            if ( wheelAPWMAcc > 0 && wheelAPWMCurrent < wheelAPWMGoal || wheelAPWMAcc < 0 && wheelAPWMCurrent > wheelAPWMGoal)
            {
                wheelAPWMCurrent += wheelAPWMAcc;
                PWMOutput(WHEEL_A_TIM, WHEEL_A_CH, wheelAPWMCurrent);
            }
            else
            {
                wheelAPWMCurrent = wheelAPWMGoal;
                PWMOutput(WHEEL_A_TIM, WHEEL_A_CH, wheelAPWMGoal);
            }
        }
    }
    else if ( deltaA > FLOAT_ZERO )
    {

        wheelAPWMCurrent = wheelAPWMGoal;
        PWMOutput(WHEEL_A_TIM, WHEEL_A_CH, wheelAPWMGoal);

    }

    if ( deltaB > accBAbs )
    {
        if (accBAbs > FLOAT_ZERO)
        {
            if ( wheelBPWMAcc > 0 && wheelBPWMCurrent < wheelBPWMGoal || wheelBPWMAcc < 0 && wheelBPWMCurrent > wheelBPWMGoal)
            {
                wheelBPWMCurrent += wheelBPWMAcc;
                PWMOutput(WHEEL_B_TIM, WHEEL_B_CH, wheelBPWMCurrent);
            }
            else
            {
                wheelBPWMCurrent = wheelBPWMGoal;
                PWMOutput(WHEEL_B_TIM, WHEEL_B_CH, wheelBPWMGoal);
            }
        }
    }
    else if ( deltaB > FLOAT_ZERO )
    {
        wheelBPWMCurrent = wheelBPWMGoal;
        PWMOutput(WHEEL_B_TIM, WHEEL_B_CH, wheelBPWMGoal);
    }

    if ( deltaC > accCAbs)
    {
        if (accCAbs > FLOAT_ZERO)
        {
            if ( wheelCPWMAcc > 0 && wheelCPWMCurrent < wheelCPWMGoal || wheelCPWMAcc < 0 && wheelCPWMCurrent > wheelCPWMGoal)
            {
                wheelCPWMCurrent += wheelCPWMAcc;
                PWMOutput(WHEEL_C_TIM, WHEEL_C_CH, wheelCPWMCurrent);
            }
            else
            {
                wheelCPWMCurrent = wheelCPWMGoal;
                PWMOutput(WHEEL_C_TIM, WHEEL_C_CH, wheelCPWMGoal);
            }
        }
    }
    else if ( deltaC > FLOAT_ZERO )
    {
        wheelCPWMCurrent = wheelCPWMGoal;
        PWMOutput(WHEEL_C_TIM, WHEEL_C_CH, wheelCPWMGoal);
    }
#ifdef FOUR_WHEEL
    if ( deltaD > accDAbs)
    {
        if (accDAbs > FLOAT_ZERO)
        {
            wheelDPWMCurrent += wheelDPWMAcc;
            PWMOutput(WHEEL_D_TIM, WHEEL_D_CH, wheelDPWMCurrent);
        }
    }
    else if ( deltaD > FLOAT_ZERO )
    {
        wheelDPWMCurrent = wheelDPWMGoal;
        PWMOutput(WHEEL_D_TIM, WHEEL_D_CH, wheelDPWMGoal);
    }
#endif
    count++;
}

static void BTMotion(uint8_t *data, uint8_t n)
{
    uint8_t linerangle = data[0];
    uint8_t linerspeed = data[1] * 10;
    int8_t anglespeed = (data[2] - 0x50) * 2;
    BaseMotion_Move(linerangle, linerspeed, anglespeed, 0);
}