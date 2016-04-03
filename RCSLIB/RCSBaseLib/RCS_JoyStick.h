/************************************************************************************************
  * @file    RCS_joyStick.h
  * @author  黄东润
  * @version V1.0.0
  * @date    2012年8月21日
  * @brief   利用USART串口通信，实现游戏手柄控制
  * @history
 */
#ifndef __JOYSTICK__
#define __JOYSTICK__

#include "rcs.h"
#include "bsp.h"

#define USB_KEY_ONE                 0x0001
#define USB_KEY_TWO                 0x0002
#define USB_KEY_THREE               0x0004
#define USB_KEY_FOUR                0x0008
#define USB_KEY_FIVE                0x0010
#define USB_KEY_SIX                 0x0020
#define USB_KEY_SEVEN               0x0040
#define USB_KEY_EIGHT               0x1000
#define USB_KEY_NINE                0x0100
#define USB_KEY_TEN                 0x0200
#define USB_ROLE_LEFT_PRESS         0x0400
#define USB_ROLE_RIGHT_PRESS        0x0800
#define DEFAULT_VALUE_START         0xc3
#define DEFAULT_VALUE_END           0xc0
#define DEFAULT_VALUE_RANGE     0x00
#define DEFAULT_VALUE_KEY             0x0000
#define DEFAULT_VALUE_ANGLE         0x7ffe

#define AUTO_MODE 0x00
#define NORMAL_MODE 0xff

typedef struct RealData
{
	uint16_t key;
	int16_t role_left_angle;
	int16_t role_right_angle;
} CtrlData;

typedef void (*JOYSTICK_FUNT)(CtrlData ctrlData);

//函数名: InitJoyStick
//功能: 配置USART的串口通信功能
//返回值: 无
void InitJoyStick(USART_TypeDef *_usart, GPIO_TypeDef *_group, uint16_t _tx, uint16_t _rx, uint32_t _baud, JOYSTICK_FUNT _joystickCtrl);


#endif
