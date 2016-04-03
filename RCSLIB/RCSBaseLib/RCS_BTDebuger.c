//@filename：  RCS_BTDebuger.c
//@date： 2012-08-29
//@author: 柯国霖
//@brief：蓝牙串口调试模块

#include "rcs.h"


#define BT_NAME     "BT"

#define BUF_MAX_SIZE    20
#define MSG_MAX_SIZE    30
#define START_BYTES_COUNT   5
#define IS_START_BYTE(x) (x == RCS_SB_B || x == RCS_SB_L)

#define CURRENT_BUF dataBuf[pIn]
#define PUTOUT_BUF dataBuf[pOut]
#define APPLICATION_ADDRESS   (uint32_t)0x08000000

const uint8_t RCS_SB_B = 0x5A;
const uint8_t RCS_SB_L = 0xA5;
const uint8_t RCS_PARA_MODIFY = 0x01;
const uint8_t RCS_PARA_REC = 0x02;
const uint8_t RCS_PARA_ACK = 0x0F;
const uint8_t RCS_PARA_ERR = 0x0E;
const uint8_t RCS_PARA_SAVE2FLASH = 0x03;
const uint8_t RCS_BURN = 0xA0;



extern void SaveAllChange2Flash(); //define in RCS_para.c
extern uint8_t GetParaTypeLen(uint8_t _type);//define in RCS_para.c

static void ProcessRecDate_task(void *p_arg);
static Boolean IsBig_Endian();
static void NullFunt(uint8_t *_data, uint8_t n);
static void InitRCSComVect();
static void BTRecISR();

static USART_TypeDef *BT_Usart = USART6;
static GPIO_TypeDef *BT_Gpio = GPIOC;
static uint16_t BT_txd = GPIO_Pin_6;
static uint16_t BT_rxd = GPIO_Pin_7;
static uint32_t BT_baud = 230400;

#pragma data_alignment=8
static OS_STK ProcessRecDate_task_stk[NORMAL_TASK_STK_SIZE];

static Boolean isBigEndian = FALSE;
static uint8_t dataBuf[BUF_MAX_SIZE][MSG_MAX_SIZE];
static uint8_t pIn = 0;
static uint8_t pOut = 0;
static OS_EVENT *semRecData;
static RCS_COMMAND_FUNT  RCS_COMVectTbl[0xff];


//@brief:初始化蓝牙
void InitBTDebuger(USART_TypeDef *_btUsart, GPIO_TypeDef *_group, uint16_t _tx, uint16_t _rx, uint32_t _baud)
{
    assert_param(IS_USART_ALL_PERIPH(_btUsart));
    assert_param(IS_USART_BAUDRATE(_baud));
    assert_param(IS_GPIO_ALL_PERIPH(_group));
    assert_param(IS_GPIO_PIN(_tx));
    assert_param(IS_GPIO_PIN(_rx));

    BT_Usart = _btUsart;
    BT_Gpio = _group;
    BT_txd = _tx;
    BT_rxd = _rx;
    BT_baud = _baud;

    RCS_USART_Config(BT_Usart, BT_Gpio, BT_txd, BT_rxd, BTRecISR, BT_baud, 0x22);

    // RCS_USART_Send_Str(BT_Usart, "AT+BAUD9");
    RCS_USART_Send_Str(BT_Usart, BT_NAME );
    isBigEndian = IsBig_Endian();
    semRecData = OSSemCreate(0);
    InitRCSComVect();
    OSTaskCreate(ProcessRecDate_task,
                 (void *)0,
                 &ProcessRecDate_task_stk[NORMAL_TASK_STK_SIZE - 1],
                 ProcessRecDate_PRIO);
}


//@name SetRCSComVect
//@brief: 将指令编号和函数绑定
//@param:uint8_t id 绑定的指令号
//@param:RCS_COMMAND_FUNT 绑定的服务函数
void SetRCSComVect(uint8_t _id, RCS_COMMAND_FUNT _isr)
{
    OS_SR_ALLOC();
    OS_ENTER_CRITICAL();
    RCS_COMVectTbl[_id] = _isr;
    OS_EXIT_CRITICAL();

}
//@name:BT_Send_Char
//@brief:发送蓝牙串口
void BT_Send_Char(char _c)
{

    RCS_USART_Send_Char(BT_Usart, (uint16_t)_c);

}
//@name:BT_Send_Str
//@brief:发送字符串
void BT_Send_Str(char *_str)
{

    RCS_USART_Send_Str(BT_Usart, (uint8_t *)_str);

}
//@name: BT_Send_Line
//@brief: 发送一行数据
void BT_Send_Line(char *_str)
{
    char temp[100];
    sprintf(temp, "%s\r\n", _str);
    RCS_USART_Send_Str(BT_Usart, (uint8_t *)temp);
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    //char temp[70];
    //BaseMotion_Stop();
    //sprintf(temp, "assert failed on file %s line %d \r\n", file, line);
    //RCS_USART_Send_Str(BT_Usart, (uint8_t *)temp);
    /* Infinite loop */
    while (1)
    {
    }
}
#endif //USE_FULL_ASSERT
//@name:    IsBig_Endian
//@brief:   判断内存是否大端
//@retval:   true 为大端 false为小端
static Boolean IsBig_Endian()
{
    uint16_t test = 0x1122;
    if (*( (uint8_t *) &test ) == 0x11)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

//@name:    ProcessRecDate_task
//@brief:   对串口发过来的数据进行处理
static void ProcessRecDate_task(void *p_arg)
{
    uint8_t err;
    while (1)
    {
        //等待数据发送过来
        OSSemPend(semRecData, 0, &err);


        if (PUTOUT_BUF[2] == RCS_PARA_MODIFY)
        {
            Boolean recIsBigEndian = FALSE;
            char tmpName[20];
            uint8_t nameLen = PUTOUT_BUF[3];
            uint8_t dataStart = nameLen + 5;
            if ( PUTOUT_BUF[0] == RCS_SB_B )
            {
                recIsBigEndian = TRUE;
            }
            for (uint8_t i = 0; i < nameLen; ++i)
            {
                tmpName[i] = PUTOUT_BUF[i + 4];
            }
            tmpName[nameLen] = '\0';
            uint8_t typeLen = GetParaTypeLen(PUTOUT_BUF[dataStart - 1] );
            if (isBigEndian != recIsBigEndian)
            {
                for (int i = 0; i < typeLen / 2; ++i)
                {
                    uint8_t tmp = PUTOUT_BUF[dataStart + i];
                    PUTOUT_BUF[dataStart + i] = PUTOUT_BUF[dataStart + typeLen - 1 - i];
                    PUTOUT_BUF[dataStart + typeLen - 1 - i] = tmp;
                }

            }
            SetParaDataByName(tmpName, (void *) & (PUTOUT_BUF[dataStart]));
        }
        else if (PUTOUT_BUF[2] == RCS_PARA_SAVE2FLASH)
        {
            SaveAllChange2Flash();
        }
        else if (PUTOUT_BUF[2] == RCS_BURN)
        {
            BSP_IntDisAll();
            uint32_t JumpAddress = *(__IO uint32_t *) (APPLICATION_ADDRESS + 4);
            FNCT_VOID Jump_To_Application = (FNCT_VOID) JumpAddress;
            /* Initialize user application's Stack Pointer */
            __set_MSP(*(__IO uint32_t *) APPLICATION_ADDRESS);
            Jump_To_Application();
        }
        else
        {
            uint8_t id = PUTOUT_BUF[2];
            uint8_t n = PUTOUT_BUF[3];
            uint8_t data[10];
            for (uint8_t i = 0; i < n ; i++)
            {
                data[i] = PUTOUT_BUF[4 + i];
            }
            RCS_COMVectTbl[id]( data, n );
        }

        if (pIn == pOut)
            continue;
        pOut++;
        if (pOut == 20)
            pOut = 0;

        OSTimeDly(50);
    }
}

//@brief: 指令默认空函数
static void NullFunt(uint8_t *_data, uint8_t n)
{

}

//@brief: 将指令向量表初始化
static void InitRCSComVect()
{
    for (int i = 0x10; i < 0xff; ++i)
    {
        RCS_COMVectTbl[i] = NullFunt;
    }
}



//@brief:蓝牙串口接收中断服务函数
static void BTRecISR()
{
    static uint16_t count = 0;
    static uint16_t startCount = 0;
    static uint8_t dataCount = 0;
    static uint8_t lastData = 0;
    static uint8_t dataLen = 0;
    static uint8_t sum = 0;
    static Boolean isStart = FALSE;
    static uint8_t errcount = 0;
    if (USART_GetITStatus(BT_Usart, USART_IT_RXNE) != RESET)
    {
        uint8_t recData = (uint8_t)USART_ReceiveData(BT_Usart);
        if (isStart)
        {
            if (count == START_BYTES_COUNT)
            {
                dataLen = recData;
                CURRENT_BUF[1] = recData;
                dataCount = 0;
            }
            else
            {
                if (dataCount < dataLen)
                {
                    CURRENT_BUF[2 + dataCount] = recData;
                    sum +=  recData;
                }
                else if (dataCount == dataLen)
                {
                    if (sum == recData)
                    {
                        pIn++;
                        if (pIn == 20)
                            pIn = 0;
                        OSSemPost(semRecData);
                    }
                    isStart = FALSE;
                    startCount = 0;
                    count = 0;
                    dataCount = 0;
                    dataLen = 0;
                    sum = 0;
                    return ;
                }
                dataCount++;
            }
        }

        if (( count == 0  && IS_START_BYTE(recData) ||
                IS_START_BYTE(recData) && IS_START_BYTE(lastData) && recData == lastData)
           )
        {
            startCount++;
            if ( startCount >= START_BYTES_COUNT )
            {
                isStart = TRUE;
                CURRENT_BUF[0] = recData;
            }
        }
        else
        {
            startCount = 0;
            lastData = recData;
            count = 0;
            return;
        }

        count++;
        lastData = recData;
    }
    else
    {
        errcount++;
        if (errcount > 20)
        {
            errcount = 0;
            USART_DeInit(BT_Usart);
            InitBTDebuger(BT_USART, BT_GPIO, BT_TXD, BT_RXD, BT_BAUD);
        }
    }
    USART_ClearITPendingBit(BT_Usart, USART_IT_RXNE);
}