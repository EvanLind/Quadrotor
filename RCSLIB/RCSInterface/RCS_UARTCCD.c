#include "RCS_UARTCCD.h"
#include "rcs.h"
#define STARTBYTE 0xAA

#define STARTCMD 0xAA
#define STOPCMD 0x55
volatile uint8_t g_width = 0;
volatile uint8_t g_shift = 0;
volatile Boolean g_isComplete = FALSE;
static void CCDRecISR()
{
    static uint8_t count = 0;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t recData = (uint8_t)USART_ReceiveData(USART3);
        if (recData == STARTBYTE)
        {
            count = 0;
        }
        else if (count == 0)
        {
            g_width = recData;
            count = 1;
        }
        else if (count == 1)
        {
            g_shift = recData;
            g_isComplete = TRUE;
            count = 2;
        }
    }
    else
    {

        USART_DeInit(USART3);
        RCS_USART_Config(USART3, GPIOD, GPIO_Pin_8, GPIO_Pin_9, CCDRecISR, 230400, 0x32);
    }
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}

void CCD_Init()
{
    RCS_USART_Config(USART3, GPIOD, GPIO_Pin_8, GPIO_Pin_9, CCDRecISR, 230400, 0x32);
}
void CCD_Deinit()
{
    USART_DeInit(USART3);
}
void CCD_Start()
{
    RCS_USART_Send_Char(USART3, STARTCMD);
    g_isComplete = FALSE;
}
void CCD_Stop()
{
    RCS_USART_Send_Char(USART3, STOPCMD);
}