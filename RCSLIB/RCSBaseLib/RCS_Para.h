//@filename:RCS_para.c
//@date:2012-09-02
//@author:柯国霖
//@brief：参数结构体的定义和相关操作


#ifndef _RCS_PARA_H_
#define _RCS_PARA_H_
#include "bsp.h"
#include "RCS_CommonFunction.h"

#define RCS_UI8     0x01
#define RCS_I8      0x02
#define RCS_UI16    0x03
#define RCS_I16     0x04
#define RCS_UI32    0x05
#define RCS_I32     0x06
#define RCS_F32     0x07



//@name:  AddPara
//@brief:   添加一个参数结构体
//@param:char *_name 参数名称
//@param:uint8_t _type 参数类型
//@param:void *_data   参数数据
//retval:参数id
uint8_t AddPara(char *_name, uint8_t _type, void *_data);
//@name:  GetParaDataByName
//@brief:   根据参数名返回数据
//@param:char *_name 参数名称
//retval:void * 请用类型转换将数据转成相应的类型
//          NULL为失败
void *GetParaDataByName(char *_name);
//@name:  GetParaDataById
//@brief:   根据参数id返回数据
//@param:uint8_t _id 参数id
//retval:void * 请用类型转换将数据转成相应的类型
//          NULL为失败
void *GetParaDataById(uint8_t _id);
//@name:  SetParaDataByName
//@brief:   根据参数名设置参数数据
//@param:char *_name 参数名称
//@param:void *_data   参数数据
//返回值:   TRUE 成功 FALSE 失败
Boolean SetParaDataByName(char *_name, void *_data);




#endif //_RCS_PARA_H_