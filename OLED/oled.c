#include "oled.h"
#include "ASCII.h"
#include <stdio.h>


extern const uint8_t F6x8[][6];
extern const uint8_t F8X16[];

 /*     -------------  
        |   Xm=127  |
        |   Ym=7    |
        |竖着存数据 |
        |   8192    |
        -------------     */
// 使用：
    // RCS_Delay_Init(72);
    // oled_io_init();
    // oled_config();
    // oled_wr_6x8str(0, 0, "EvanLin");


//SCL:B1  SDA:E8   RST:E10 DC:E12
void oled_io_init(void)
{
    SetGpioOutput(GPIOB, OLED_SCL_PIN);
    SetGpioOutput(GPIOE, OLED_SDA_PIN);
    SetGpioOutput(GPIOE, OLED_RST_PIN);
    SetGpioOutput(GPIOE, OLED_DC_PIN);
}

void oled_config(void)
{
    oled_io_init();
    OLED_SCL_H;
    OLED_RST_L;
    OSTimeDly(10);
    OLED_RST_H;
    oled_wr_cmd(0xae);//关闭oled显示
    oled_wr_cmd(0x00);//设置低位光标col位置
    oled_wr_cmd(0x10);//设置高位光标col位置
    oled_wr_cmd(0x40);//设置开始行位置（0->63）=>（0x40->7f）
    oled_wr_cmd(0x81);//设置对比度
    oled_wr_cmd(0xcf);//设对比度为207
    oled_wr_cmd(0xa1);//Set Segment Re-map（这个很难表达，自己测试下就知道）
    oled_wr_cmd(0xc8);//Set COM/Row Scan Direction  （这个很难表达，自己测试下就知道）
    oled_wr_cmd(0xa6);//设置正常显示模式 0xa7为反显
    oled_wr_cmd(0xa8);//--set multiplex ratio(1 to 64)
    oled_wr_cmd(0x3f);//--1/64 duty
    oled_wr_cmd(0xd3);//-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    oled_wr_cmd(0x00);//-not offset
    oled_wr_cmd(0xd5);//设置显示时钟（震荡频率，divide比例）
    oled_wr_cmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    oled_wr_cmd(0xd9);//--set pre-charge period
    oled_wr_cmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    oled_wr_cmd(0xda);//--set com pins hardware configuration
    oled_wr_cmd(0x12);
    oled_wr_cmd(0xdb);//--set vcomh
    oled_wr_cmd(0x40);//Set VCOM Deselect Level
    oled_wr_cmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    oled_wr_cmd(0x02);
    oled_wr_cmd(0x8d);//--set Charge Pump enable/disable
    oled_wr_cmd(0x14);//--set(0x10) disable
    oled_wr_cmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
    oled_wr_cmd(0xaf);//--turn on oled panel
    oled_clear(0x00);
    oled_set_pos(0, 0);
}
// void oled_wr_bit(uint8_t data)
// {
//     OLED_DC_H;
//     _DELAY;
//     OLED_SCL_L;
//     _DELAY;
//     if (data)OLED_SDA_H;
//     else OLED_SDA_L;
//     _DELAY;
//     OLED_SCL_H;
//     _DELAY;
//     OLED_SCL_L;
//     _DELAY;
//     data <<= 1;
// }
void oled_wr_dat(uint8_t data)
{
    uint8_t i = 8;
    OLED_DC_H;
    _DELAY;
    OLED_SCL_L;
    _DELAY;
    while (i--)
    {
        if (data & 0x80)OLED_SDA_H;
        else OLED_SDA_L;
        _DELAY;
        OLED_SCL_H;
        _DELAY;
        OLED_SCL_L;
        _DELAY;
        data <<= 1;
    }
}

void oled_wr_cmd(uint8_t cmd)
{
    uint8_t i = 8;
    OLED_DC_L;
    _DELAY;
    OLED_SCL_L;
    _DELAY;
    while (i--)
    {
        if (cmd & 0x80)OLED_SDA_H;
        else OLED_SDA_L;
        _DELAY;
        OLED_SCL_H;
        _DELAY;
        OLED_SCL_L;
        _DELAY;
        cmd <<= 1;
    }
}
void oled_PutPixel(uint8_t x,uint8_t y)
{
    uint8_t data1;  //data1当前点的数据 

    oled_set_pos(x,y); 
    data1 = 0x01<<(y%8);    
    oled_wr_cmd(0xb0+(y>>3));
    oled_wr_cmd(((x&0xf0)>>4)|0x10);
    oled_wr_cmd((x&0x0f)|0x00);
    oled_wr_dat(data1);       
}
void oled_set_pos(uint8_t x, uint8_t y)
{
    oled_wr_cmd(0xb0 + y);
    oled_wr_cmd(((x & 0xf0) >> 4) | 0x10);
    oled_wr_cmd((x & 0x0f) | 0x01);
}
void oled_wr_8x16str(uint8_t x, uint8_t y, uint8_t *str)
{
    uint8_t ch, i, j;
    for (i = 0; str[i] != '\0'; i++)
    {
        ch = str[i] - 32;
        if (x > 120)
        {
            x = 0;
            y++;
        }
        oled_set_pos(x, y);
        for (j = 0; j < 8; j++)
            oled_wr_dat(F8X16[ch * 16 + j]);
        oled_set_pos(x, y + 1);
        for (j = 0; j < 8; j++)
            oled_wr_dat(F8X16[ch * 16 + j + 8]);
        x += 8;
    }
}

void oled_wr_6x8str(uint8_t x, uint8_t y, char *str)
{
    uint8_t ch, i, j;
    for (i = 0; str[i] != '\0'; i++)
    {
        ch = str[i] - 32;
        if (x > 122)
        {
            x = 0;
            y++;
        }
        oled_set_pos(x, y);
        for (j = 0; j < 6 ; j++)
            oled_wr_dat(F6x8[ch][j]);
        x += 6;
    }
}

void oled_Display_float_to_char(uint8_t x, uint8_t y, float goal_value)//显示动态数值
{
    char buffer[10];
    char *dis_value = &buffer[0];
    sprintf(buffer,"%.3f",goal_value);  //其他转字符串sprintf，字符串转其他scanf
    oled_wr_6x8str(x, y, dis_value);
}

void oled_Display_int_to_char(uint8_t x, uint8_t y, int goal_value)//显示动态数值
{
    char buffer[10];
    char *dis_value = &buffer[0];
    sprintf(buffer,"%d",goal_value);  //其他转字符串sprintf，字符串转其他scanf
    oled_wr_6x8str(x, y, dis_value);
}

void oled_clear(uint8_t ch)
{
    uint8_t i, j;
    for (i = 0; i < 8; i++)
    {
        oled_set_pos(0, i);
        for (j = 0; j < 128; j++)
            oled_wr_dat(ch);
    }
}

void oled_left_sroll(void)
{
    oled_wr_cmd(0x27);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0x01);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0xff);
    oled_wr_cmd(0x2f);   //0x2f 水平滚动开始，0x2e水平滚动停止
}

void oled_up_down_sroll(void)
{
    oled_wr_cmd(0x29);   //[29h]: Vertical and Right Horizontal Scroll;[2Ah]: Vertical and Left Horizontal Scroll
    oled_wr_cmd(0x00);   //Dummy byte(Set as 00h)
    oled_wr_cmd(0x00);   //Define start page address(0~7)
    oled_wr_cmd(0x01);   //Set time interval between each scroll step in terms of  frame frequency
    oled_wr_cmd(0x01);   //Define end page address(0~7),The value of start page address must be larger or equal to end page address
    oled_wr_cmd(0x00);   //Vertical scrolling offset(0~63)
    oled_wr_cmd(0x2f);   //Activate scroll
}

void oled_right_sroll(void)
{
    oled_wr_cmd(0x26);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0x04);
    oled_wr_cmd(0x01);
    oled_wr_cmd(0x00);
    oled_wr_cmd(0xff);
    oled_wr_cmd(0x2f);
}

void oled_convert_num_to_str(int num,uint8_t *str)
{
    uint8_t tmp[10];  //这里不能改成uint8_t * tmp ，自己体会原因。
    int j = 0, i = 0;
    if (num < 0)
    {
        str[j++] = '-';
        num = -num;
    }
    do
    {
        tmp[i++] = num % 10 + '0';
        num /= 10;
    }
    while (num);
    while (i)
        str[j++] = tmp[--i];
    str[j] = '\0';
}
// void oledshow()
// {        
//     oled_wr_6x8str(0, 0, "TurAgl:");
//     oled_wr_6x8str(0, 1, "RacAgl:");
//     oled_wr_6x8str(0, 2, "StaPoX:");
//     while(1)
//     {    
//       OSTimeDly(10);  
//       // oled_Display_float_to_char(43, 0, 0.000);
//       // oled_Display_float_to_char(43, 1, 1.8707);
//       // oled_Display_float_to_char(43, 2, 2000.0);
//       oled_Display_float_to_char(43, 0, GetAencoder1Data());
//       oled_Display_float_to_char(43, 1, GetAencoder2Data());
//       oled_Display_float_to_char(43, 2, (float)GetEncoder1Count() * 294.7 /2000.0);
//   }
// }
// void Dis_Camera_Picture(unsigned char img[], unsigned char position)
// {
//     unsigned char temp_img_64x40[64*40];
//     unsigned char temp_img_8x40[8*40];
//     unsigned char *p_temp_img_64x40 = temp_img_64x40;//取内容地址
//     signed int i = 0, j = 0, k = 0;
//     signed int count = 0;

//     for(i = 0; i < 64; i++)
//     {
//         if(i < 60)
//         {
//             for(j = 0; j < 40; j++)
//             {
//                 count = 0;
//                 for(k = 0; k < 4; k++)
//                 {
//                     if( (*(img+40*(4*i+k)+j)) == 0 )
//                         count++;
//                     else
//                         break;
//                 }
//                 if(count == 4)
//                     (*p_temp_img_64x40) = 1;
//                 else
//                     (*p_temp_img_64x40) = 0;

//                 p_temp_img_64x40++;
//             }           
//         }
//         else
//             for(j = 0; j < 40; j++)
//             {
//                 (*p_temp_img_64x40) = 0;
//                 p_temp_img_64x40++;
//             }
//         }

//         for(i = 0; i < 8; i++)
//         {
//             for(j = 0; j < 40; j++)
//             {
//                 (*(temp_img_8x40+40*i+j)) = 0;
//                 for(k = 0; k < 8; k++)
//                 {
//                     (*(temp_img_8x40+40*i+j)) += ( (*(temp_img_64x40+40*(8*i+k)+j)) << k );     
//                 }
//             }
//         }

//         Dis_Play_Data8x40(temp_img_8x40,position);
//     }
//     void Dis_Play_Data8x40(unsigned char arr[], unsigned position)
//     {
//         signed int i = 0, j = 0;
//         unsigned char *p_arr = arr;
//         for(i = 0; i < 8; i++)
//         {
//             oled_set_pos(position,i);
//             for(j = 0; j < 40; j++)
//             {
//                 oled_wr_dat(*p_arr);
//                 p_arr++;
//             }
//         }   
//     }

// /*------------摄像头算法处理---------------*/
// /**
//   * @brief  Display the picture.
//   * @param  img[][240]: Inserts a double matriul[][240].
//             position:Inserts a number.
//   * @retval None
//   */
//   void Dis_uint8_t_Camera(u8 img[][10], unsigned position, unsigned view_position_x, unsigned view_position_y)
//   {
//     u8 JpegBufferNew[640];
//     int t,p,m,d;
//     for (int n = 0; n < 8; ++n)
//     {
//         for (int i = 0; i < 10; ++i)
//         {
//              /*---------二维数组--------*/
//             t =view_position_x+8*n;//行++
//             p =8*i+80*n;  
//             d =view_position_y+i;//列++
//             for (int x = 0; x < 8; ++x)
//             {
//                 JpegBufferNew[x+p]= ((img[0+t][d] & 0x80) >>7) | ((img[1+t][d] & 0x80) >>6)| 
//                 ((img[2+t][d] & 0x80) >>5) | ((img[3+t][d] & 0x80) >>4) | ((img[4+t][d] & 0x80)>>3) |
//                 ((img[5+t][d] & 0x80) >>2) | ((img[6+t][d] & 0x80) >>1) | ((img[7+t][d] & 0x80)>>0);
//                 img[0+t][d] <<= 1;
//                 img[1+t][d] <<= 1;
//                 img[2+t][d] <<= 1;
//                 img[3+t][d] <<= 1;
//                 img[4+t][d] <<= 1;
//                 img[5+t][d] <<= 1;
//                 img[6+t][d] <<= 1;
//                 img[7+t][d] <<= 1;
//             }
//         } 
//     }  
//     for (int k = 0; k < 8; k++)
//     {
//         oled_set_pos(position, k);
//         for (int j = 0; j < 80; j++)
//         {
//           m = j+80*k;
//           oled_wr_dat(JpegBufferNew[m]);
//       }
//   }
// }
// void Dis_Bool_Camera(Boolean img[][80], unsigned position)
// {
//     uint8_t JpegBufferNew[640];
//     for (int row = 0; row < 8; ++row)
//     {
//      oled_set_pos(position, row);
//      for (int col = 0; col < 80; ++col)
//      {
//         JpegBufferNew[col+80*row]= 
//         ((img[0+8*row][col]& 0x01) ) | ((img[1+8*row][col]& 0x01) <<1) | 
//         ((img[2+8*row][col]& 0x01) <<2) | ((img[3+8*row][col]& 0x01) <<3) | ((img[4+8*row][col]& 0x01) <<4) |
//         ((img[5+8*row][col]& 0x01) <<5) | ((img[6+8*row][col]& 0x01) <<6) | ((img[7+8*row][col]& 0x01) <<7);
//               /*---------OLED显示--------*/
//         oled_wr_dat(JpegBufferNew[col+80*row]);
//     }
// }
// }
// Boolean img_exg[64][80], img_dis[64][80];
// void Dis_Handle_Camera(u8 img[][10], unsigned position)//写封装
// {
//     /*********写一个数组缓存********/
//     uint8_t JpegBufferNew[640];/*--------不用static-------*/
//     int x1,x2,y1,y2,x1r,y1r;/*--------x代表列，y代表行-------*/
//     // static Boolean img_exg[64][80];
//     /*---------桶形失真矫正--------*/
//     /*--------算法坐标原点应为图形中点-------*/  
//     signed char i,x;
//     for ( y1 = 0; y1 < 64; ++y1)//行++
//     {
//          y1r = -y1+Middle_zero_y;//坐标轴移位
//         for (i = 0; i < 10; ++i)//列++
//         {
//             /*---------二维数组--------*/
//             for (x = 0; x < 8; ++x)
//             {
//                 x1r = x+8*i-Middle_zero_x; //坐标轴移位             
//                 /*---------有符号运算--------*/
//                 x2 = (k1*(1+k2x*y1r*y1r)*x1r+Middle_real_x);
//                 y2 = (-k1*(1+k2y*x1r*x1r)*y1r+Middle_real_y);
//                 if(x2>=80 || y2>=64||y1>=64||x2<0||y2<0||y1<0)
//                     while(1);
//                 img_exg[y2][x2]=((img[y1][i] & 0x80) >>7);
//                 // img_exg[y2][x2]=0x01;
//                 img[y1][i] <<= 1;
//             }
//         } 
//     }  
//     /*---------返回u8数组--------*/
//     Dis_Bool_Camera(img_exg,0);
// }
// void Zip_Camera(u8 img[][10], unsigned position)
// {
//     u8 JpegBufferNew[640];
//     u8 img_10x64[8];
//     int t,p,m;
//     for (int n = 0; n < 8; ++n)
//     {
//         for (int i = 0; i < 10; ++i)
//         {
//              /*---------二维数组--------*/
//             t = 8*n;
//             p = 8*i+80*n; 
//             for (int h = 0; h < 8; ++h)
//             {
//                 img_10x64[h] = (img[0+4*i][100+t+h] & img[1+4*i][100+t+h] & img[2+4*i][100+t+h] & img[3+4*i][100+t+h]);
//             }
//             for (int x = 0; x < 8; ++x)
//             {
//                 JpegBufferNew[x+p]= 
//                 ((img_10x64[0] & 0x80) >>7) | ((img_10x64[1] & 0x80) >>6) | 
//                 ((img_10x64[2] & 0x80) >>5) | ((img_10x64[3] & 0x80) >>4) |
//                 ((img_10x64[4] & 0x80) >>3) | ((img_10x64[5] & 0x80) >>2) |
//                 ((img_10x64[6] & 0x80) >>1) | ((img_10x64[7] & 0x80) >>0);
//                 img_10x64[0] <<= 1;
//                 img_10x64[1] <<= 1;
//                 img_10x64[2] <<= 1;
//                 img_10x64[3] <<= 1;
//                 img_10x64[4] <<= 1;
//                 img_10x64[5] <<= 1;
//                 img_10x64[6] <<= 1;
//                 img_10x64[7] <<= 1;
//             }
//         } 
//     }  
//     for (int k = 0; k < 8; k++)
//     {
//       oled_set_pos(position, k);
//       for (int j = 0; j < 80; j++)
//       {
//         m = j+80*k;
//         oled_wr_dat(JpegBufferNew[m]);
//     }
// }
// }

// float Line_gradient;//斜率
// void Line_Middle_Camera(u8 img[][10], unsigned position)//写封装
// {
//     uint8_t JpegBufferNew[640];/*--------不用static-------*/
//     // static Boolean img_exg[64][80], img_dis[64][80];
//     int x,i,x1,x2,y1,y2,Line_end_x = 0,Line_end_y = 0,Line_x = -1,Line_y = -1;/*--------x代表列，y代表行-------*/
//     int First_Point = 1,circle_find = 1;
//     uint8_t Line_number = 0;
//     int count = 0, img_count = 0, position_min_y = 0, position_max_y = 0, Middle_y = 0;
//     uint8_t choose_the_line = 0,break_flag = 0;

//     /*-------CODE SEGMENT------*/
//     for ( y1 = 0; y1 < 64; ++y1)//行++
//     { 
//         for (i = 0; i < 10; ++i)//列++
//         {
//             /*---------二维数组--------*/
//             for (x = 0; x < 8; ++x)
//             {      
//                 x1 = x+8*i; 
//                 img_exg[y1][x1]=((img[y1][i] & (0x80>>x)) >>(7-x));
//                 // img[y1][i] <<= 1;
//             }
//         }
//     }
//     /*-------*/
//     for (y1 = 0; y1 < 64; ++y1)
//     {
//         for (x1 = 0; x1 < 80; ++x1)
//         {
//             img_dis[y1][x1]=0x00;//黑屏处理
//         }
//     }
//     /*----求中点---*/
//     for (x2 = 0; x2 < 80; ++x2)
//     {
//         First_Point = 1;
//         count = 0;
//         Middle_y = 0;
//         for (y2 = 0; y2 < 58; ++y2)
//         {
//             /*-----之后的点是否0----*/ 
//             if (First_Point == 0)
//             {
//                 if (img_exg[y2][x2] == 0)//如果接下去不是0，继续找第一点
//                 {
//                     count++;    //计数        
//                     if (count > 150)
//                     {
//                         // while(1);
//                         count = 0;
//                         continue;
//                     }              
//                 }
//                 else
//                 {
//                     if (count <=10 && count >=2)//如果长于2小于50暗
//                     {
//                         // Line_number++;  //  判断有几条线，取最下面的线
//                         position_max_y = y2;
//                         if (position_max_y >=64)//assert y<64
//                             while(1);
//                         Middle_y=((position_max_y + position_min_y)/2);//取图形中点
//                         if (Middle_y >= 58 || x2 >= 80)//assert y<58,x<80
//                             while(1);
//                         img_dis[Middle_y][x2]=0x01;  //变亮
//                         // img_dis[y2][x2]=0x01;
//                     }
//                     First_Point = 1;
//                     count = 0;
//                 }
//             }
//              /*-----第一个点是否0----*/ 
//             if (First_Point == 1)//等待第一个黑点
//             {
//                 if (img_exg[y2][x2] == 0)//第一次黑点
//                 {
//                     position_min_y = y2; 
//                     if (position_max_y >=64)//assert y<64
//                         while(1);
//                     First_Point = 0; 
//                 }                   
//             }
//         } 
//     } 
//     /*---------对采集直线进行优化处理--------*/
//     while (circle_find)
//     {
//         for (x2 = Line_x+1; x2 < 80; ++x2)
//         {
//             for (y2 = Line_y+1; y2 < 64; ++y2)
//             {
//                 if (img_dis[y2][x2] == 0x01)//第一次黑点
//                 {
//                     Line_y = y2;
//                     Line_x = x2;
//                     img_count++;
//                     break_flag = 1;
//                     break;
//                 }
//             }
//             if (break_flag == 1)
//             {
//                 break_flag = 0;
//                 break;
//             }
//         }
//         for (x2 = 0; x2 < 80; ++x2)
//         {
//             for ( y2 = Line_y -range; y2 < Line_y +range; ++y2)
//             {
//                 if (img_dis[y2][x2] == 0x01)//第一次黑点
//                 {
//                     img_count++;
//                     Line_end_y = y2;
//                     Line_end_x = x2;                                    
//                     if (img_count >= 70)//计算点数有多少个
//                     {
//                         circle_find = 0;
//                         choose_the_line = 1;
//                         img_count = 0;
//                     }
//                 }
//             }
//         }
//         img_count = 0;
//         Line_number++;
//         if (Line_number >= 3)//超过三条直线
//         {
//             circle_find = 0;
//         }
//         if (choose_the_line == 1)//如果有满足条件的直线，计算斜率
//         {
//             Line_gradient = ((float)Line_end_y - Line_y)/(Line_end_x - Line_x);
//         }
//     }//Line_y,Line_x
//     choose_the_line = 0;
//     /*---------返回八位数组显示--------*/
//     Dis_Bool_Camera(img_dis,0);
// }
// void Line_Gradient_Camera(u8 img[][10], unsigned position)//写封装
// {
//     uint8_t JpegBufferNew[640];/*--------不用static-------*/
//     // static Boolean img_exg[64][80], img_dis[64][80];
//     static int axis_x[50],axis_y[50];
//     int x,i,x1,x2,y1,y2,x2_correct=0,y2_correct=0,
//         Line_end_x=0,Line_end_y=0,Line_start_x=-1,Line_start_y=-1;/*--------x代表列，y代表行-------*/
//     int First_Point = 1,circle_find = 1;
//     uint8_t Line_number = 0,choose_the_line = 0,break_flag = 0;
//     int axis_count=0,count=0,img_count=0,position_min_y=0,position_max_y=0,Middle_y=0;
//     /*-------CODE SEGMENT------*/
//     for ( y1 = 0; y1 < 64; ++y1)//行++
//     { 
//         for (i = 0; i < 10; ++i)//列++
//         {
//             ---------二维数组--------
//             for (x = 0; x < 8; ++x)
//             {      
//                 x1 = x+8*i; 
//                 img_exg[y1][x1]=((img[y1][i] & (0x80>>x)) >>(7-x));
//             }
//         }
//     }
//     /*-------*/
//     for (y1 = 0; y1 < 64; ++y1)
//     {
//         for (x1 = 0; x1 < 80; ++x1)
//         {
//         img_dis[y1][x1]=0;//黑屏处理
//     }
// }
//     /*---------------------------直线采集做成通用的函数封装--------------------------*/
//     /*--------求中点直线--------*/
// for (x2 = 0; x2 < 80; ++x2)
// {
//     First_Point = 1;
//     count = 0;
//     Middle_y = 0;
//     for (y2 = 0; y2 < 58; ++y2)
//     {
//             /*-----之后的点是否0----*/ 
//         if (First_Point == 0)
//         {
//                 if (img_exg[y2][x2] == 0)//如果接下去不是0，继续找第一点
//                 {
//                     count++;    //计数        
//                     if (count > 20)
//                     {
//                         count = 0;
//                         break;
//                     }              
//                 }
//                 else
//                 {
//                     if (count <=10 && count >=2)//如果长于2小于10暗
//                     {
//                         position_max_y = y2;
//                         if (position_max_y >=64)//assert y<64
//                             while(1);
//                         Middle_y=((position_max_y + position_min_y)/2);//取图形中点
//                         if (Middle_y >= 58 || x2 >= 80)//assert y<58,x<80
//                             while(1);
//                         img_dis[Middle_y][x2]=1;  //变亮
//                     }
//                     First_Point = 1;
//                     count = 0;
//                 }
//             }
//              /*-----第一个点是否0----*/ 
//             if (First_Point == 1)//等待第一个黑点
//             {
//                 if (img_exg[y2][x2] == 0)//第一次黑点
//                 {
//                     position_min_y = y2; 
//                     if (position_max_y >=64)//assert y<64
//                         while(1);
//                     First_Point = 0; 
//                 }                   
//             }
//         } 
//     } 
//     for (y1 = 0; y1 < 64; ++y1)
//     {
//         for (x1 = 0; x1 < 80; ++x1)
//         {
//         img_exg[y1][x1]=0;//黑屏处理
//     }
// }
//     /*---------对采集直线进行优化处理--------*/
// while (circle_find)
// {
//     for (x2 = Line_start_x+1; x2 < 80; ++x2)
//     {
//         for (y2 = Line_start_y+1; y2 < 64; ++y2)
//         {
//                 if (img_dis[y2][x2] == 1)//第一次亮点
//                 {
//                     Line_start_y = y2;
//                     Line_start_x = x2;
//                     img_count++;
//                     break_flag = 1;
//                     break;
//                 }
//             }
//             if (break_flag == 1)//选择一条直线
//             {
//                 break_flag = 0;
//                 break;
//             }
//         }
//         Line_end_y = Line_start_y;
//         Line_end_x = Line_start_x;
//         for (x2 = Line_start_x+1; x2 < 80; ++x2)
//         {
//             for ( y2 = Line_end_y -range; y2 < Line_end_y +range; ++y2)
//             {
//                 if (img_dis[y2][x2] == 1)
//                 {
//                     img_count++;
//                     Line_end_y = y2;
//                     Line_end_x = x2;
//                     /*-----矫正桶形失真------*/
//                     y1 = -y2+Middle_zero_y;//坐标轴移位
//                     x1 = x2-Middle_zero_x; //坐标轴移位
//                     /*---------有符号运算--------*/
//                     x2_correct = (k1*(1+k2x*y1*y1)*x1+Middle_real_x);
//                     y2_correct = (-k1*(1+k2y*x1*x1)*y1+Middle_real_y);
//                     /*-----矫正-----*/
//                     if (axis_count <= math_numer)//采集n个点进行最小二值处理
//                     {
//                     //axis_x[axis_count]=x2;
//                     //axis_y[axis_count]=y2;
//                         axis_x[axis_count] = x2_correct;
//                         axis_y[axis_count] = y2_correct;
//                         axis_count++;
//                     }
//                     // img_exg[y2][x2] = img_dis[Line_end_y][Line_end_x];//无桶形失真
//                     img_exg[y2_correct][x2_correct] = img_dis[Line_end_y][Line_end_x];                                  
//                     if (img_count >= collect_numer)//计算点数有多少个
//                     {
//                         circle_find = 0;
//                         choose_the_line = 1;
//                     }
//                     break;
//                 }
//             }
//         }
//         img_count = 0;
//         Line_number++;
//         if (Line_number >= 3)//如果超过三条直线，跳出
//         {
//             circle_find = 0;
//         }
//         if (choose_the_line == 1)//如果有满足条件的直线，计算斜率,显示直线附近图
//         {
//             /*-----最小二乘算法-----*/
//             axis_count = 0;
//             //Line_gradient=min_double_mul(axis_x,axis_y,math_numer);//算斜率
//             // Line_gradient=((float)Line_end_y-Line_start_y)/(Line_end_x-Line_start_x);简易算法
//         }
//         else
//         {
//             for (y1 = 0; y1 < 64; ++y1)
//             {
//                 for (x1 = 0; x1 < 80; ++x1)
//                 {
//                 img_exg[y1][x1]=0;//黑屏处理
//             }
//         }
//     }   
//     }//Line_y,Line_x
//     /*-----------------------------------------------------------------------*/
//     choose_the_line = 0;
//     /*---------返回八位数组显示--------*/
//     Dis_Bool_Camera(img_exg,0);
// }

