/************************************************************************************************
  * @file    RCS_joyStick.h
  * @author  黄东润
  * @version V1.0.0
  * @date    2012年8月21日
  * @brief   利用USART串口通信，实现游戏手柄控制
  * @history
**************************************************************************************************/

#include "rcs.h"

static USART_TypeDef *JoyStick_usart = USART3;
static GPIO_TypeDef *JoyStick_gpio = GPIOC;
static uint16_t JoyStick_txd = GPIO_Pin_10;
static uint16_t JoyStick_rxd = GPIO_Pin_11;
static uint32_t JoyStick_baud = 19200;


static volatile CtrlData ctrlData ;//按键返回参数
static JOYSTICK_FUNT joystickCtrl ;
static void JoyStick_usart_ISR(void);

//@name: InitJoyStick
//@biref: 配置USART的串口通信功能
void InitJoyStick(USART_TypeDef *_usart, GPIO_TypeDef *_group,
                  uint16_t _tx, uint16_t _rx, uint32_t _baud, JOYSTICK_FUNT _joystickCtrl)
{
    assert_param(IS_USART_ALL_PERIPH(_usart));
    assert_param(IS_USART_BAUDRATE(_baud));
    assert_param(IS_GPIO_ALL_PERIPH(_group));
    assert_param(IS_GPIO_PIN(_tx));
    assert_param(IS_GPIO_PIN(_rx));
    JoyStick_usart = _usart;
    JoyStick_gpio = _group;
    JoyStick_txd = _tx;
    JoyStick_rxd = _rx;
    JoyStick_baud = _baud;
    RCS_USART_Config(JoyStick_usart, JoyStick_gpio, JoyStick_txd,
                     JoyStick_rxd, JoyStick_usart_ISR, JoyStick_baud, 0x21);
    joystickCtrl = _joystickCtrl;
}

//@brief:手柄按键串口中断
static void JoyStick_usart_ISR(void)
{
    static uint32_t count = 0;
    uint8_t temp;
    uint16_t tempInt;
    static uint8_t sum = 0;
    static uint8_t rSum = 100;
    static uint8_t errcount = 0;
    if (USART_GetITStatus(JoyStick_usart, USART_IT_RXNE) != RESET)
    {
        temp = (uint8_t)USART_ReceiveData(JoyStick_usart);
        tempInt = temp;
        tempInt &= 0x00ff;
        if (temp == DEFAULT_VALUE_START)
        {
            count = 0;
            sum = 0;
        }
        else if ((temp == DEFAULT_VALUE_END))
        {
            sum &= 0x7f;
            if (rSum == sum && count == 7)
            {

                ctrlData.role_left_angle *= 3;
                ctrlData.role_right_angle *= 3;
                if (ctrlData.role_left_angle != 0x7FFE)
                {
                    ctrlData.role_left_angle += 270;
                    if (ctrlData.role_left_angle >= 360)
                    {
                        ctrlData.role_left_angle -= 360;
                    }
                }
                if (ctrlData.role_right_angle != 0x7FFE)
                {

                    ctrlData.role_right_angle += 270;
                    if (ctrlData.role_right_angle >= 360)
                    {
                        ctrlData.role_right_angle -= 360;
                    }
                }
                joystickCtrl(ctrlData);
            }
            count = 0;
            sum = 0;
            rSum = 100;
        }
        else
        {
            switch (count)
            {
                case 0: ctrlData.key = tempInt << 8; sum = temp; break;
                case 1: ctrlData.key |= tempInt; sum += temp; break;
                case 2: ctrlData.role_left_angle = tempInt << 8; sum += temp; break;
                case 3: ctrlData.role_left_angle |= tempInt; sum += temp; break;
                case 4: ctrlData.role_right_angle = tempInt << 8; sum += temp; break;
                case 5: ctrlData.role_right_angle |= tempInt; sum += temp; break;
                case 6: rSum = temp; break;
            }
            count++;
        }
    }
    else
    {
        errcount++;
        if (errcount > 100)
        {
            USART_DeInit(JOYSTICK_USART);
            InitJoyStick(JOYSTICK_USART, JOYSTICK_GPIO,
                         JOYSTICK_TXD, JOYSTICK_RXD, JOYSTICK_BAUD, joystickCtrl);
            errcount = 0;
        }
    }
    USART_ClearITPendingBit(JoyStick_usart, USART_IT_RXNE);
}