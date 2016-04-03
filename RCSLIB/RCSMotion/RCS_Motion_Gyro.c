/*
 *@author：柯国霖
 *@brief:陀螺仪角度获取和计算
 *@date:2012-10-10
 */
#include "rcs.h"
#include "RCS_CommonFunction.h"
#include "rcs_spi.h"
#include "MPU6050.h"
#define FOR_TIMEDELAY       for(uint8_t ii = 0;ii<10;ii++)

#define SPI_CS_LOW()       GPIO_ResetBits(GYRO_SPI_CS_GPIO, GYRO_SPI_CS_PIN)
#define SPI_CS_HIGH()      GPIO_SetBits(GYRO_SPI_CS_GPIO, GYRO_SPI_CS_PIN)

#define GYRO_TIMER          TIM9
#define GYRO_TIMER_PPR       5
#define GYRO_TIMER_DIV       10000

volatile double  g_deg = 0.0;//陀螺仪的当前角度
volatile Boolean  g_canUsed = FALSE;
//extern volatile int mpu_temp;
static volatile double measureCovariance = 400;
static volatile double forecastCovariance = 10;
static volatile double zeroValue = 400;

volatile int32_t tmpView1 = 0;//调试用的参数
volatile double tmp2 = 0;



static void BTSendGyroData(uint8_t *data, uint8_t n);
#ifdef _ADIS_16135_
static uint16_t getDataCmd = 0x0600;
#elif defined _ADIS_16265_
static uint16_t getDataCmd = 0x0400;
#else

#endif

#if defined _ADIS_16265_ || defined _ADIS_16135_

static volatile int32_t degDataSum = 0;
static volatile uint32_t degDataCount = 0;

inline uint16_t SpiSendData(uint16_t _data)
{
    SPI_CS_LOW();
    FOR_TIMEDELAY;
    while (SPI_I2S_GetFlagStatus(GYRO_SPI, SPI_I2S_FLAG_TXE) == RESET);
    GYRO_SPI->DR = _data;
    while (SPI_I2S_GetFlagStatus(GYRO_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    uint16_t data = (GYRO_SPI->DR);
    FOR_TIMEDELAY;
    SPI_CS_HIGH();
    return data;
}
// @brief: Adis16265的初始化过程
inline void InitAdis16265(void)
{
    SpiSendData(0xb601);//256采样频率
    SpiSendData(0xb902);//160度
    SpiSendData(0xb804);//4taps
    SpiSendData(0xbe08);//保存
    OSTimeDly(100);

}

inline void InitAdis16135(void)
{
    SpiSendData(0x9e7f);//采样频率256
    SpiSendData(0xa004);//16taps滤波
    SpiSendData(0xa808);//保存
    OSTimeDly(100);

}
inline void cailAdis16135()
{
    SpiSendData(0xA801);
    OSTimeDly(100000);
}
static void GYRO_TIMER_ISR()
{
    TIM_ClearITPendingBit(GYRO_TIMER, TIM_IT_Update);
    int16_t data = SpiSendData(getDataCmd);
    // tmpView1 = data;
    degDataSum += data;
    degDataCount ++;
}
#else

static volatile int32_t degDataSum = 0;
static volatile uint32_t degDataCount = 0;


static void Gyro_Spi_ISR(void)
{
    EXTI_ClearITPendingBit(GetRCS_EXTI_Line(GYRO_SPI_CS_PIN));
    while (SPI_I2S_GetFlagStatus(GYRO_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    degDataSum += (int16_t)(GYRO_SPI -> DR);
    tmpView1 = (int16_t)(GYRO_SPI -> DR);
    degDataCount ++;
}
#endif
// @brief : 初始化陀螺仪
// @param : _TimeDiv 多久调用一次中断
void InitGyro(double _TimeDiv)
{

#if defined _ADIS_16265_ || defined _ADIS_16135_
    InitSpi(GYRO_SPI, SPI_Mode_Master,
            GYRO_SPI_CLK_GPIO, GYRO_SPI_CLK_PIN,
            GYRO_SPI_MOSI_GPIO, GYRO_SPI_MOSI_PIN,
            GYRO_SPI_MISO_GPIO, GYRO_SPI_MISO_PIN,
            SPI_CPOL_High, SPI_CPHA_2Edge);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GYRO_SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GYRO_SPI_CS_GPIO, &GPIO_InitStructure);
    SPI_CalculateCRC(GYRO_SPI, DISABLE);
    SPI_CS_HIGH();
#else
    GPIO_PinAFConfig(GYRO_SPI_CS_GPIO, GetRCS_GPIO_PinSource(GYRO_SPI_CS_PIN), GetRCS_GPIO_AF_SPI(GYRO_SPI));

    GPIO_InitTypeDef GPIO_Struct;

    GPIO_StructInit(&GPIO_Struct);
    GPIO_Struct.GPIO_Pin = GYRO_SPI_CS_PIN;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef  OD_OUTPUT
    GPIO_Struct.GPIO_OType = GPIO_OType_OD;
    GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
    GPIO_Struct.GPIO_OType = GPIO_OType_PP;
    GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
#endif
    GPIO_Init(GYRO_SPI_CS_GPIO, &GPIO_Struct);

    InitSpi(GYRO_SPI, SPI_Mode_Slave,
            GYRO_SPI_CLK_GPIO, GYRO_SPI_CLK_PIN,
            GYRO_SPI_MOSI_GPIO, GYRO_SPI_MOSI_PIN,
            GYRO_SPI_MISO_GPIO, GYRO_SPI_MISO_PIN
            , SPI_CPOL_Low, SPI_CPHA_1Edge);
    SPI_CalculateCRC(GYRO_SPI, DISABLE);

#endif


#if defined _ADIS_16265_ || defined _ADIS_16135_
    //InitAdis16265();
    OSTimeDly(50);
    SpiSendData(getDataCmd);
    InitTimerInt(GYRO_TIMER, GYRO_TIMER_PPR, GYRO_TIMER_DIV, GYRO_TIMER_ISR, 0x00);
#else
    // BSP_IntVectSet (GetBSP_INT_ID_SPI(GYRO_SPI), Gyro_Spi_ISR);
    // NVIC_InitTypeDef   NVIC_InitStructure;
    // NVIC_InitStructure.NVIC_IRQChannel = GetRCS_SPI_IRQn(GYRO_SPI);
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
    // SPI_I2S_ITConfig(GYRO_SPI, SPI_I2S_IT_RXNE, ENABLE);
    RCS_InitEXTI (GYRO_SPI_CS_GPIO, GYRO_SPI_CS_PIN, EXTI_Trigger_Falling, Gyro_Spi_ISR, 0x00);
#endif
    g_deg = 0;
    SetRCSComVect(0x10, BTSendGyroData);
}

//@brief:获取陀螺仪的返回值
inline double GetGyroData()
{
    static int times = 0;
    char aa[20] = "the gyro is error";
    char Str0[10] = "0000";
#ifdef _ADIS_16135_
    // int16_t data = SpiSendData(getDataCmd);
    // tmp2 = data;
    // return (double)data;

    double gyroData = 0;
    if (degDataCount != 0 )
    {
        gyroData = degDataSum / (double)degDataCount;
        tmp2 = gyroData;
    }
    degDataSum = 0;
    degDataCount = 0;
    return gyroData;

#elif defined _ADIS_16265_
    uint16_t data = SpiSendData(getDataCmd);
    int16_t data2;
    if (!(data & 0x2000))
    {
        data2 = (int16_t)(data & 0x3fff);
    }
    else
    {
        data2 = (int16_t)(data | 0xc000) ;
    }
    tmpView1 = data2;
    return (double)data2;

#else
    double gyroData = 0;
    if (degDataCount != 0 )
    {
        gyroData = degDataSum / (double)degDataCount;
        tmp2 = gyroData;
        /******************************************/
        //if (times == 0 && fabs(gyroData) > 17000)
        //{

            //BT_Send_Line(aa);
            //sprintf(aa, "X:%d , Y: %d , DEG:%7.2f, POINT:%d", GetX(), GetY(), GetDeg(), FuzzyCurrentPoint());
            //BT_Send_Line(aa);
            //BUZZER_ON;
            //times = 1;
            // while (1)
            // {
            //     Fuzzy_Pause();
            //     BaseMotion_Move(0, 0, 0, 0);
            //     RCS_USART_Send_Str(MPU_USART, Str0);
            //     PWMOutput(Motor_PWM_TIM2, Motor_PWM_CH2, 0);
            // }
        //}
        /******************************************/
    }

    degDataSum = 0;
    degDataCount = 0;
    return gyroData;
#endif

}
//@brief : 卡尔曼滤波。
//@param ：测量值。
inline double kalmanFlitering(double measureValue)
{
    double optimalValue = 0;             //计算最优值
    static volatile double preOptimalValue = 0;              //上一次计算最优值
    float kalmanGain = 0.5;             //卡尔曼增益
    static volatile double optimalCovariance = 5;            //最优值方差
    // optimalCovariance最优值方差： forecastCovariance：预报协方差 measureCovariance：测量协方差
    //arm_sqrt_f32(( optimalCovariance + forecastCovariance) / ( optimalCovariance + measureCovariance + forecastCovariance) , &kalmanGain);
    kalmanGain = ( optimalCovariance + forecastCovariance) / ( optimalCovariance + measureCovariance + forecastCovariance);
    //根据卡尔曼增益计算最优值
    optimalValue = preOptimalValue + kalmanGain * (measureValue - preOptimalValue);
    //计算最优值方差
    optimalCovariance = (1 - kalmanGain) * (optimalCovariance + forecastCovariance);
    //更新上一次的计算值为当前值
    preOptimalValue = optimalValue;

    return optimalValue;
}
//@brief: 将陀螺仪的值滤波并转换为真实角速度。
inline double GyroData2Deg(double _data)
{
    return kalmanFlitering(_data) * ToDegFactor;
}
//@brief : 获取陀螺仪的零点。
static void getZeroValue(double data, uint16_t maxCount)
{
    static double sum = 0;
    static uint32_t count = 0;
    if (count < maxCount)
    {
        sum += data;
        count++;
    }
    else
    {
        zeroValue = sum / (double)count;
        sum = 0;
        count = 0;
        if (zeroValue > 20.0 || zeroValue < -20.0)
            BUZZER_ON;
    }
}

//brief : 获取陀螺仪的测量值方差。
static void getMeasureCovariance(double data, uint16_t maxCount)
{
    static double sum = 0;
    static uint32_t count = 0;
    if (count < maxCount)
    {
        sum += data * data;
        count++;
    }
    else
    {
        measureCovariance = sum / (double)count;
        g_canUsed  = TRUE;
        sum = 0;
        count = 0;
    }
}

//@brief:获取陀螺仪的累计角度
double CalculateGyroDeg(void)
{
    static  uint32_t count = 0;         //count为采集数据的次数
    volatile double  deltaDeg = 0;       //角度变化增量
    double gyroData = ANGLE_DIR * GetGyroData();
    if (count > COLLECT_TIME * 2 + 1)
    {
        gyroData -= zeroValue;
        if (!IsRotate())
        {
            gyroData = 0;
        }
        g_deg  += GyroData2Deg(gyroData);
        if (g_deg >= 180 ) g_deg -= 360;
        else if (g_deg < -180 ) g_deg += 360;
    }
    else if (count <= COLLECT_TIME)
    {
        if (IsRotate())
        {
            return 0.0;
        }
        getZeroValue(gyroData, COLLECT_TIME);
        count++;
    }
    else
    {
        if (IsRotate())
        {
            return 0.0;
        }
        getMeasureCovariance(gyroData - zeroValue, COLLECT_TIME);
        count++;
    }
    return g_deg;
}

extern uint8_t temppoint_1;
extern uint8_t temppoint_2;

extern volatile double tmpViewAngleV;
extern volatile double tmpViewSpeedAngle;
extern volatile double tmpViewSpeed;

static void BTSendGyroData(uint8_t *data, uint8_t n)
{
    static int count = 0;
    char bb[80];
    char aa[80];
    int32_t l1 = (int32_t)GetEncoder1Count();
    int32_t l2 = (int32_t)GetEncoder2Count();
    int32_t l3 = (int32_t)GetEncoder3Count();
    //sprintf(bb, "X: %d, Y: %d, deg: %.2lf, V1: %d, V2: %d, tmpView1:%d",
     //       GetX(), GetY(), GetDeg(), l1, l2, tmpView1);

    //sprintf(bb, "%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%d,%d,%d,%f,%d,%d", g_deg, , g_Y, tmpViewSpeedAngle, tmpViewSpeed, tmpViewAngleV,
    //  l1, l2 , l3 , tmpView1,CCD_GetWidth(),CCD_GetShift());
    //  sprintf(bb, "%d,%.4f,%.4lf", tmp2,tmpView1, g_deg);
    BT_Send_Line(bb);
    //sprintf(aa, "mpu:%d , INF_Pos: %d,zero:%7.2f,ads:%7.2f", HANDINF_Pos_Cnt, UPINF_Pos_Cnt, zeroValue, tmp2);
    // OSTimeDly(50);
    BT_Send_Line(aa);
}