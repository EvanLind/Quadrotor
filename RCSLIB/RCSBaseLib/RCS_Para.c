//@filename:RCS_para.c
//@date:2012-09-02
//@author:柯国霖
//@brief：参数结构体的定义和相关操作

#include "RCS_Para.h"
#include "RCS_Eeprom.h"

#define FLASH_DATA_BEGIN_ADDR   0x0001
#define FLASH_DATA_END_ADDR     EEPROM_MAX_SIZE
#define PARA_SIZE               (4)
#define IS_IN_FLASH             0x5A
#define MAX_PARA_NUMBER         50

union paraData
{
	uint8_t data[4];
	uint8_t u8Data;
	int8_t i8Data;
	uint16_t u16Data;
	int16_t i16Data;
	uint32_t u32Data;
	int32_t i32Data;
	float f32Data;
};

typedef struct RCSPara
{
	uint8_t type;
	uint8_t typeLen;
	uint8_t id;
	char name[20];
	union paraData data;
	uint8_t isChanged;

} RCSParaClass;



/*
data in flash
|| IS_IN_FLASH || nameCheckSum || nameXor || dataChecksum || data(1-4) ||
*/

static RCSParaClass RCS_PARAS[MAX_PARA_NUMBER]; //用于保存参数的数组
static uint16_t currentParaNumber = 0; //当前参数的个数

static void SetParaData(RCSParaClass *_para, void *_data);
static void InitPara(RCSParaClass *_para, char *_name, uint8_t _id, uint8_t _type, void *_data);
static void SavePara2Flash(RCSParaClass *_para);
static uint16_t GetParaIdByName(char *_name);

uint8_t GetParaTypeLen(uint8_t _type);


//@name:  SetParaData
//@brief:   设置参数的数据
//@param:RCSParaClass *_para: 参数结构体
//@param:void *_data 数据
//@note: 这个函数由系统维护，请不要调用
static void SetParaData(RCSParaClass *_para, void *_data)
{
	switch (_para->type)
	{
	case RCS_UI8  : _para -> data.u8Data = *(uint8_t *)_data ; break;
	case RCS_I8   : _para -> data.i8Data = *(int8_t *)_data ; break;
	case RCS_UI16 : _para -> data.u16Data = *(uint16_t *)_data ; break;
	case RCS_I16  : _para -> data.i16Data = *(int16_t *)_data ; break;
	case RCS_UI32 : _para -> data.u32Data = *(uint32_t *)_data ; break;
	case RCS_I32  : _para -> data.i32Data = *(int32_t *)_data ; break;
	case RCS_F32  : _para -> data.f32Data = *(float *)_data ; break;
	}
}

//@name:  InitPara
//@brief:   初始化参数结构体
//@param:RCSParaClass *_para: 参数结构体
//@param:char *_name 参数名称
//@param:uint8_t _id  参数编号
//@param:uint8_t _type 参数类型
//@param:void *_data   参数数据
//@note:此函数由系统自动调用，用户请调用addpara
static void InitPara(RCSParaClass *_para, char *_name, uint8_t _id, uint8_t _type, void *_data)
{
	strcpy( _para->name, _name);
	_para -> id = _id;
	_para -> type = _type;
	_para -> isChanged = 0;
	uint32_t addr = FLASH_DATA_BEGIN_ADDR + PARA_SIZE * _id;
	//从flash读取数据
	FLASH_Unlock();
	uint16_t tmp = 0;
	uint8_t tmpData[8];
	EE_ReadVariable(addr, &tmp);
	TakepartTwoBytes(tmp, &tmpData[0], &tmpData[1]);
	EE_ReadVariable(addr + 1, &tmp);
	TakepartTwoBytes(tmp, &tmpData[2], &tmpData[3]);
	EE_ReadVariable(addr + 2, &tmp);
	TakepartTwoBytes(tmp, &tmpData[4], &tmpData[5]);
	EE_ReadVariable(addr + 3, &tmp);
	TakepartTwoBytes(tmp, &tmpData[6], &tmpData[7]);
	FLASH_Lock();

	uint8_t nameSum = _para -> name[0];
	uint8_t nameXor = _para -> name[0];
	uint8_t dataSum = 0;
	for (int i = 1; _para -> name[i] != '\0' ; ++i)
	{
		nameSum += _para -> name[i];
		nameXor ^= _para -> name[i];
	}
	_para -> typeLen = GetParaTypeLen(_para->type);
	for (int i = 0; i < _para -> typeLen; i++)
	{
		dataSum += tmpData[4 + i];
	}
	//检验数据是否被改变
	if ( tmpData[0] == IS_IN_FLASH &&
			tmpData[1] == nameSum &&
			tmpData[2] == nameXor &&
			tmpData[3] == dataSum
	   )
	{
		SetParaData(_para, (void *)(&tmpData[4]));
	}
	else
	{
		SetParaData(_para, _data);
	}
}

//@name:  GetParaIdByName
//@briefS:   根据名字获取参数结构体的ID
//@param:char *_name 参数名称
//@retval:   id。如为MAX_PARA_NUM失败
static uint16_t GetParaIdByName(char *_name)
{
	for (int i = 0 ; i < currentParaNumber ; i++)
	{
		if (strcmp(RCS_PARAS[i].name, _name) == 0)
		{
			return i;
		}

	}
	return MAX_PARA_NUMBER;
}

//@name:  SavePara2Flash
//@brief: 将参数保存到flash
//@param:RCSParaClass *_para 参数结构体
static void SavePara2Flash(RCSParaClass *_para)
{
	FLASH_Unlock();
	uint32_t addr = FLASH_DATA_BEGIN_ADDR + PARA_SIZE * _para -> id;
	uint8_t nameSum = _para -> name[0];
	uint8_t nameXor = _para -> name[0];
	uint8_t dataSum = 0;
	for (int i = 1; _para -> name[i] != '\0' ; ++i)
	{
		nameSum += _para -> name[i];
		nameXor ^= _para -> name[i];
	}
	for (int i = 0; i < _para -> typeLen; ++i)
	{
		dataSum += (uint8_t)( _para -> data.data[i]);
	}

	uint16_t tmp = 0;
	tmp = CombineTwoBytes(IS_IN_FLASH, nameSum);
	EE_WriteVariable(addr, tmp);

	tmp = CombineTwoBytes(nameXor, dataSum);
	EE_WriteVariable(addr + 1, tmp);

	switch (_para -> typeLen)
	{
	case 1 :
		tmp = CombineTwoBytes(_para -> data.data[0], 0);
		EE_WriteVariable(addr + 2, tmp);
		break;
	case 2 :
		tmp = CombineTwoBytes(_para -> data.data[0], _para -> data.data[1]);
		EE_WriteVariable(addr + 2, tmp);
		break;
	case 4:
		tmp = CombineTwoBytes(_para -> data.data[0], _para -> data.data[1]);
		EE_WriteVariable(addr + 2, tmp);

		tmp = CombineTwoBytes(_para -> data.data[2], _para -> data.data[3]);
		EE_WriteVariable(addr + 3, tmp);
		break;
	}
	FLASH_Lock();
}

//@name:  AddPara
//@brief:   添加一个参数结构体
//@param:char *_name 参数名称
//@param:uint8_t _type 参数类型
//@param:void *_data   参数数据
//retval:参数id
uint8_t AddPara(char *_name, uint8_t _type, void *_data)
{
	if (currentParaNumber >= MAX_PARA_NUMBER)
	{
		return -1;
	}
	OS_SR_ALLOC();
	OS_ENTER_CRITICAL();
	InitPara(&RCS_PARAS[currentParaNumber], _name, currentParaNumber, _type, _data);
	currentParaNumber++;
	OS_EXIT_CRITICAL();
	return currentParaNumber - 1;

}

//@name:  GetParaDataByName
//@brief:   根据参数名返回数据
//@param:char *_name 参数名称
//retval:void * 请用类型转换将数据转成相应的类型
//          NULL为失败
void *GetParaDataByName(char *_name)
{
	uint16_t pid  = GetParaIdByName(_name);
	return GetParaDataById(pid);
}

//@name:  GetParaDataById
//@brief:   根据参数id返回数据
//@param:uint8_t _id 参数id
//retval:void * 请用类型转换将数据转成相应的类型
//          NULL为失败
void *GetParaDataById(uint8_t _id)
{
	if (_id < MAX_PARA_NUMBER)
	{
		return (void *)(RCS_PARAS[_id].data.data);
	}
	else
	{
		return NULL;
	}
}

//@name:  SetParaDataByName
//@brief:   根据参数名设置参数数据
//@param:char *_name 参数名称
//@param:void *_data   参数数据
//返回值:   TRUE 成功 FALSE 失败
Boolean SetParaDataByName(char *_name, void *_data)
{
	uint16_t pid  = GetParaIdByName(_name);
	OS_SR_ALLOC();
	if (pid < MAX_PARA_NUMBER)
	{
		OS_ENTER_CRITICAL();
		SetParaData(&RCS_PARAS[pid], _data);
		RCS_PARAS[pid].isChanged = 1;
		OS_EXIT_CRITICAL();
		return TRUE;
	}
	return FALSE;
}

//@name:  SaveAllChange2Flash
//@brief:   将所有未保存的参数保存到flash
//@retval:   TRUE 成功 FALSE 失败
void SaveAllChange2Flash()
{
	for (int i = 0; i < currentParaNumber; ++i)
	{
		if ( RCS_PARAS[i].isChanged == 1 )
		{
			SavePara2Flash(&RCS_PARAS[i]);
			RCS_PARAS[i].isChanged = 0;
		}
	}
}
//@name:  getParaTypeLen
//@brief:   获取类型对应的长度
//@retval:   类型长度
uint8_t GetParaTypeLen(uint8_t _type)
{
	switch (_type)
	{
	case RCS_UI8  : return 1;  break;
	case RCS_I8   : return 1;  break;
	case RCS_UI16 : return 2;  break;
	case RCS_I16  : return 2;  break;
	case RCS_UI32 : return 4;  break;
	case RCS_I32  : return 4;  break;
	case RCS_F32  : return 4;  break;
	}
	return 0;
}
