/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2007; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                      APPLICATION CONFIGURATION
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                     Micrium uC-Eval-STM32F107
*                                        Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : EHS
*********************************************************************************************************
*/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__


/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            BSP CONFIGURATION
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                              TASKS NAMES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  OS_TASK_TMR_PRIO                       (OS_LOWEST_PRIO - 2)

/*taskpriority*/
#define	STARTUP_TASK_PRIO 		4
#define ProcessRecDate_PRIO		5
#define STRIKE_TASK_PRIO		6
#define GETPOS_TASK_PRIO		7
// #define LimitCtrlTask_PRIO		7
// #define AutoTask_PRIO			8
// #define SubTask_PRIO			9
// #define TASK4           		10
// #define TASK5           		11
// #define TASK6           		12
#define UPMotorTask_PRIO		13

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/

#define	STARTUP_TASK_STK_SIZE 	256
#define NORMAL_TASK_STK_SIZE	512


/*
*********************************************************************************************************
*                                        uC/LIB CONFIGURATION
*********************************************************************************************************
*/

#define  LIB_MEM_CFG_OPTIMIZE_ASM_EN      DEF_ENABLED
#define  LIB_MEM_CFG_ARG_CHK_EXT_EN       DEF_ENABLED
#define  LIB_MEM_CFG_ALLOC_EN             DEF_DISABLED
#define  LIB_MEM_CFG_POOL_NBR                  10
#define  LIB_MEM_CFG_HEAP_SIZE              35000L




#endif
