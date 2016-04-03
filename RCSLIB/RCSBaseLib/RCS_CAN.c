//文件名：  RCS_CAN.c
//日期： 2014-11-9

#include "rcs.h"
#include "RCS_CAN.h"
#define USE_INTERRUPT


extern volatile int Platform_Dir, Table_Agl, Racket_Str;
// void wushua_test()
// {
//     static int i = 0;
//     int16_t encoder_203;
//     int16_t current_203;
//     int16_t send_current_203[1000];
//     int16_t* save=send_current_203;
//     if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
//     {
//         CanRxMsg RxMessage;
//         CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
//         encoder_203 = (RxMessage.Data[0]<<8)|RxMessage.Data[1];
//             if((encoder_203 & 0x1000) == 0x1000)
//             {
//                 encoder_203 = encoder_203 - 8191;   //将201的值变为连续的正负数
//             }
//         current_203 = (RxMessage.Data[4]<<8)|RxMessage.Data[5];
//         if(i%10==0)
//         {
//           *(save+i/10)=current_203;
//         }        
//         i++;
//         if(i>9990)
//         {
//             int a=1;
//         }
        //RCS_USART_Send_Str(USART1, send_current_203);
        //if(abs(current_203) > 13000)
        //{
          //Cmd_ESC(0,0,0);
          //recieveflag++;
        //}
        // CAN_FIFORelease(CAN1,CAN_FIFO0);
//     }
// }
// Platform_Dir, Table_Agl, Racket_Str;
volatile int Tar_x = 0, Tar_z = 0, ServeFlag = 0;
void CAN1_RX0_Interrupt(void) 
{
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
    {        
        CanRxMsg RxMessage;
        RCS_CAN1_Recieve(&RxMessage);

        if (RxMessage.Data[0] == '!')
        {
            ServeFlag = 1;
        }
        else
        {
            if(RxMessage.Data[5] == '$')
            {
                Tar_x = RxMessage.Data[0] << 8| RxMessage.Data[1];
                Tar_z = RxMessage.Data[2] << 8| RxMessage.Data[3];
                if (RxMessage.Data[4] == 0)
                {
                    Tar_x = -Tar_x;
                }
            }            
        }

        

        // if (RxMessage.Data[5] == '$')
        // {
        //     for (int i = 0; i < 3; ++i)
        //     {
        //         Agl_Mes[i] = RxMessage.Data[i];
        //     }
        //     sscanf(Agl_Mes, "%d", &Table_Agl);
        //     if (Table_Agl > 180)
        //     {
        //         Table_Agl = Table_Agl - 360;
        //     }
            
        //     Platform_Mes[0] = RxMessage.Data[3];
        //     sscanf(Platform_Mes, "%d", &Platform_Dir);

        //     Racket_Mes[0] = RxMessage.Data[4];
        //     sscanf(Racket_Mes, "%d", &Racket_Str);
        // }        

        CAN_FIFORelease(CAN1,CAN_FIFO0);
    }   
}


//@name: CAN1_Config
//@brief: 配置CAN1的通信功能
//@param: _baudRate  设置波特率（波特率需能整除3000000）

void RCS_CAN1_Config(uint32_t _baudRate)
{
    NVIC_InitTypeDef nvic_initStruct;
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    RCC_PCLK1Config(RCC_HCLK_Div8);
    RCC_ClocksTypeDef rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 3000000 / _baudRate;     //波特率计算公式：CAN时钟频率/（(SJW + BS1 + BS2) * Prescaler)
    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber=0; //选择过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32位过滤器 
    CAN_FilterInitStructure.CAN_FilterIdHigh =0x0400;//接收板的CAN地址   
    CAN_FilterInitStructure.CAN_FilterIdLow =0x0000;   
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //0x0000;多机通讯ffff
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //选择FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//使能过滤器
    CAN_FilterInit(&CAN_FilterInitStructure); //进入初始化函数
    #ifdef USE_INTERRUPT

    BSP_IntVectSet(BSP_INT_ID_CAN1_RX0, CAN1_RX0_Interrupt);

    nvic_initStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic_initStruct.NVIC_IRQChannelPreemptionPriority = 0x0000;
    nvic_initStruct.NVIC_IRQChannelSubPriority = 0x0004;
    nvic_initStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_initStruct); 
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    #endif   
}

void RCS_CAN1_Send(uint32_t Id, uint8_t Length, uint8_t* sendData)
{
	CanTxMsg TxMessage;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.StdId = Id;
    TxMessage.ExtId = Id;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = Length;
    for(int i = 0;i < Length; i++)
    {
        TxMessage.Data[i] = sendData[i];
    }
    CAN_Transmit(CAN1, &TxMessage);
    //flag = CAN_TransmitStatus(CAN1, transmail);
}

void RCS_CAN1_Recieve(CanRxMsg* RxMessage)
{
	CAN_Receive(CAN1, CAN_FIFO0, RxMessage);
	CAN_FIFORelease(CAN1,CAN_FIFO0);
}