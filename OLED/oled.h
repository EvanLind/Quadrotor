#ifndef _OLED_H_
#define _OLED_H_

//****特别注意：向（x，y）写入数据时，光标会自动一向下一位，即y++
//而到达行末是，光标不会自动移向下一行，即不会x++
//|并起 &取该位 >>
//文档在E：硬件文档/
#include "bsp.h"
#include "oled.h"
#include "rcs.h"
#include "RCS_CAN.h"

/*----桶形失真参数-----*/
#define k1 	0.87//放大缩小
#define k2x 0.0001	//桶形失真列参数0.0001
#define k2y 0.0001 //桶形失真行参数
#define Middle_zero_y 	(31.5)//31.50//不能乱修改
#define Middle_zero_x 	(39.5)//39.5
#define Middle_real_y 	(32)//32//移图像中点位置
#define Middle_real_x 	(40)//40
/*----最小二乘参数-----*/
#define range 			2
#define collect_numer  	50		//当采集直线点大于这个值
#define math_numer  	50		//最小二值处理
/*---------------------*/
// #define OLED_RCC_AHB1Periph RCC_AHB1Periph_GPIOB
// #define OLED_DC_PIN         GPIO_Pin_15
// #define OLED_RST_PIN        GPIO_Pin_14
// #define OLED_SDA_PIN        GPIO_Pin_13
// #define OLED_SCL_PIN        GPIO_Pin_12
// #define MAX_COL 128
// #define MAX_ROW 64

// #define OLED_DC_L                GPIO_ResetBits(GPIOB,OLED_DC_PIN)
// #define OLED_DC_H                GPIO_SetBits(GPIOB,OLED_DC_PIN)
// #define OLED_RST_L               GPIO_ResetBits(GPIOB,OLED_RST_PIN)
// #define OLED_RST_H               GPIO_SetBits(GPIOB,OLED_RST_PIN)
// #define OLED_SDA_L               GPIO_ResetBits(GPIOB,OLED_SDA_PIN)
// #define OLED_SDA_H               GPIO_SetBits(GPIOB,OLED_SDA_PIN)
// #define OLED_SCL_L               GPIO_ResetBits(GPIOB,OLED_SCL_PIN)
// #define OLED_SCL_H               GPIO_SetBits(GPIOB,OLED_SCL_PIN)

//SCL:B1  SDA:E8   RST:E10	DC:E12
#define OLED_DC_PIN         GPIO_Pin_12
#define OLED_RST_PIN        GPIO_Pin_10
#define OLED_SDA_PIN        GPIO_Pin_8
#define OLED_SCL_PIN        GPIO_Pin_1
#define MAX_COL 128
#define MAX_ROW 64

#define OLED_DC_L                GPIO_ResetBits(GPIOE,OLED_DC_PIN)
#define OLED_DC_H                GPIO_SetBits(GPIOE,OLED_DC_PIN)
#define OLED_RST_L               GPIO_ResetBits(GPIOE,OLED_RST_PIN)
#define OLED_RST_H               GPIO_SetBits(GPIOE,OLED_RST_PIN)
#define OLED_SDA_L               GPIO_ResetBits(GPIOE,OLED_SDA_PIN)
#define OLED_SDA_H               GPIO_SetBits(GPIOE,OLED_SDA_PIN)
#define OLED_SCL_L               GPIO_ResetBits(GPIOB,OLED_SCL_PIN)
#define OLED_SCL_H               GPIO_SetBits(GPIOB,OLED_SCL_PIN)

#define _DELAY asm volatile ("nop");\
    asm volatile ("nop");\

void oled_io_init(void);
void oled_config(void);
void oled_wr_6x8str(uint8_t x, uint8_t y, char *str);
void oled_wr_8x16str(uint8_t x, uint8_t y, uint8_t *str);
void oled_wr_dat(uint8_t data);
void oled_wr_cmd(uint8_t cmd);
void oled_set_pos(uint8_t x, uint8_t y);
void oled_clear(uint8_t ch);
void oled_left_sroll(void);
void oled_right_sroll(void);
void oled_up_down_sroll(void);
void oled_PutPixel(uint8_t x,uint8_t y);
void oled_convert_num_to_str(int num, uint8_t *str);
void oled_Display_float_to_char(uint8_t x, uint8_t y, float goal_value);
void oled_Display_int_to_char(uint8_t x, uint8_t y, int goal_value);
//void oled_switch(state_enum state);
void oled_wr_bit(uint8_t data);
// void Dis_Camera_Picture(unsigned char img[], unsigned char position);
// void Dis_Play_Data8x40(unsigned char arr[], unsigned position);
// /*---------carema handle-----------*/
// void Dis_uint8_t_Camera(u8 img[][10], unsigned position, unsigned view_position_x, unsigned view_position_y);
// void Dis_Bool_Camera(Boolean img[][80], unsigned position);
// void Zip_Camera(u8 img[][10], unsigned position);
// void Dis_Handle_Camera(u8 img[][10], unsigned position);
// void Line_Middle_Camera(u8 img[][10], unsigned position);
// void Line_Gradient_Camera(u8 img[][10], unsigned position);
// void oledshow();
#endif