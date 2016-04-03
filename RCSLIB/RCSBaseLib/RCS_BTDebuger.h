//文件名：  RCS_BTDebuger.h
//日期： 2012-08-29
//作者： 柯国霖
//文件说明：蓝牙串口调试模块
//修改历史

#ifndef _RCS_BTDEBUGER_H_
#define _RCS_BTDEBUGER_H_

#include "RCS_Usart.h"

//@brief:初始化蓝牙
void InitBTDebuger(USART_TypeDef *_btUsart, GPIO_TypeDef *_group, uint16_t _tx, uint16_t _rx, uint32_t _baud);


//@name SetRCSComVect
//@brief: 将指令编号和函数绑定
//@param:uint8_t id 绑定的指令号
//@param:RCS_COMMAND_FUNT 绑定的服务函数
void SetRCSComVect(uint8_t _id, RCS_COMMAND_FUNT _isr);
void BT_Send_Char(char _c);
void BT_Send_Str(char *_str);
void BT_Send_Line(char *_str);

#endif  //  _RCS_BTDEBUGER_H_