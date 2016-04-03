#include "bsp.h"
#include "rcs.h"
#include "RCS_Usart.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "oled.h"

// #define AUXILIARY_IIC



#pragma data_alignment=8
static OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];


static void startup_task(void *p_arg);
void InitQuadrotor(void);
void DebugISR(void);
QuadrotorCtrl Quadrotor;
Triaxial averageTria;

int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0xC000);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	BSP_IntDisAll();                        //关闭所有中断v
	OSInit();
	OSTaskCreate(startup_task,
	  (void *)0,
	  &startup_task_stk[STARTUP_TASK_STK_SIZE - 1],
	  STARTUP_TASK_PRIO);
	OSStart();
	return 0;
}

static void startup_task(void *p_arg)
{
	BSP_Init(); //包含了中断向量的初始化
	CPU_Init(); //
	
	//根据官方代码，时钟要在这里初始化
	RCC_ClocksTypeDef rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);
	SysTick_Config(rcc_clocks.HCLK_Frequency / OS_TICKS_PER_SEC);
	Mem_Init();
	#if(OS_TASK_STAT_EN > 0)
		OSStatInit(); // Determine CPU capacity.
	#endif

	InitQuadrotor();
	while(1)
	{
		OSTimeDly(1);
		MotorCtrl();
		// GetSonarDis();

		// char res[50];
		// extern volatile int RecvCom[4];
		// sprintf(res, "Rec0 = %d  Rec1 = %d  Rec2 = %d  Rec3 = %d \n\0", RecvCom[0], RecvCom[1], RecvCom[2], RecvCom[3]);
		// RCS_USART_Send_Str(DEBUGUSART, (uint8_t *)res);            
		// RCS_USART_Send_Char(DEBUGUSART, '\n');

		// Triaxial tria;
		// float Magnetic_ang=0;
		// tria = DMP_MPU6050_GetData();
		// Magnetic_ang = Read_HMC5883L();
		// char res[50];
		// if( tria.err_flag == 0 )
		// {
		// 	sprintf(res, "Roll = %.3f   Pitch = %.3f    Yaw = %.3f   A = %.3f\n\0",tria.roll, tria.pitch, tria.yaw, Magnetic_ang);
		// 	RCS_USART_Send_Str(DEBUGUSART, res);
		// 	// sprintf(res, "Magnetic_ang = %.3f\n\0",Magnetic_ang);
		// 	RCS_USART_Send_Char(DEBUGUSART, '\n');
		// }
	}
}

void InitQuadrotor(void)
{
	/****************************************************通信串口初始化****************************************************/
	RCS_USART_Config(DEBUGUSART, DEBUGGPIO, DEBUGTXD, DEBUGRXD, DebugISR, DEBUGBAUD, 0x05);
	OSTimeDly(100);
	/****************************************************IIC初始化*********************************************************/
	I2CInit(MPUI2C_PORT, MPUI2C_SCL_Pin, MPUI2C_SDA_Pin);
	#ifndef AUXILIARY_IIC
		I2CInit(HCMI2C_PORT, HCMI2C_SCL_Pin, HCMI2C_SDA_Pin);
	#endif
	OSTimeDly(100);
	/****************************************************蜂鸣器始化********************************************************/
	InitBeep(BUZZER_GPIO, BUZZER_PIN);
	OSTimeDly(100);
	/****************************************************MPU6050初始化*****************************************************/
	DMP_MPU6050_Init();
	OSTimeDly(100);
	/***************************************************HMC5883L初始化*****************************************************/
	HMC5883L_Init();
	OSTimeDly(100);
	/******************************************************PPM初始化*******************************************************/
	InitPPM();
	OSTimeDly(100);
	/****************************************************等待DMP稳定*******************************************************/
	Triaxial tria[200];
	for (int i = 0; i < 200; ++i)
	{
		tria[i] = DMP_MPU6050_GetData();
		if (tria[i].err_flag != 0)
		{
			i--;
		}
	}
	for (int i = 0; i < 200; ++i)
	{
		averageTria.pitch += tria[i].pitch;
		averageTria.roll += tria[i].roll;
		averageTria.yaw += tria[i].yaw;
	}
	averageTria.pitch /= 200; 
	averageTria.roll /= 200; 
	averageTria.yaw /= 200;
	Quadrotor.attitudeCtrl.expectAgl.pitch = averageTria.pitch;
	Quadrotor.attitudeCtrl.expectAgl.roll = averageTria.roll;
	Quadrotor.attitudeCtrl.expectAgl.yaw = averageTria.yaw;
	AttitudeInit(&Quadrotor.attitudeCtrl);
	/****************************************************电机初始化********************************************************/
	InitMotor();
	OSTimeDly(100);
	/****************************************************声纳初始化********************************************************/
	InitSonar();

	Beep(3);
}

void DebugISR(void)
{
	// static char recvBuff[20], ChaBuff[20];
	// static int index = 0;
	// if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
	// {        
	//  USART_ReceiveData(USART1);
	// }
	// if (RESET != USART_GetITStatus(USART1, USART_IT_RXNE))
	// {     
	//  recvBuff[index] = (char)USART_ReceiveData(USART1);
	//  if(recvBuff[index] == 0x0D)
	//  {           
	//      int i = 0,j = 0,error = 0;
	//      while(1)
	//      {
	//          if (recvBuff[i] == 'E')
	//          {
	//              error = 1;
	//              break;
	//          }
	//          if (recvBuff[i] == '-' || (recvBuff[i] >= '0' && recvBuff[i] <= '9'))
	//          {       
	//              error = 0;                                 
	//              break;
	//          }
	//          i++;
	//      }                
	//      while(error != 1 && recvBuff[i] != '\r')
	//      {
	//          ChaBuff[j] = recvBuff[i];
	//          i++;
	//          j++;
	//      }         
	//      if (error != 1)
	//      {
	//          ChaBuff[j] = '\0';
	//          sscanf(ChaBuff, "%d", &SlideMotEncoData);                        
	//      }
	//      index = 0;
	//      i = 0;
	//      error = 0;
	//      memset(recvBuff, 0,  sizeof(recvBuff));                          
	//  }    
	//  index++;   
	// }
	// USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
}