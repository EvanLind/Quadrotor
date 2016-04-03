#include "rcs.h"
#include "IIC.h"



void delay_us_me(uint32_t m)
{
	
	while(m--)
	{
	}
}	 
/*启动i2c*/
void I2C_Start_Me(void)
{
	SCL_L_Me;//SCL=0;
	delay_us_me(5);
	SDA_H_Me;//SDA=1;
	delay_us_me(5);
	SCL_H_Me;//SCL=1;
	delay_us_me(5);
	SDA_L_Me;//SDA=0;
	delay_us_me(5);
	SCL_L_Me;//SCL=0;
	delay_us_me(5);
}
/*结束i2c*/
void I2C_Stop_Me(void)
{ 
	SCL_L_Me;//SCL=0; 
	delay_us_me(5);
	SDA_L_Me;//SDA=0;
	delay_us_me(5);
	SCL_H_Me;//SCL=1;
	delay_us_me(5);
	SDA_H_Me;//SDA=1;
	delay_us_me(5);
	SCL_L_Me;//SCL=0; 
	delay_us_me(5);
}
/*主机检查从机的响应信号*/
int I2C_Slave_ACK_Me(void)    
{ 
 	int ACK;
	SCL_L_Me;//SCL=0;
	SCL_H_Me;//SCL=1;
	if (SDA_Read_Me)
	{ACK =1;}
	else
	{ACK = 0;}
	SCL_L_Me;//SCL=0;
	return(ACK);
}
/*主机发送i2c总线上一个字节数据*/
void I2C_SendByte_Me(uint8_t data)   
{
	uint8_t i = 8;
    while (i--) {
        SCL_L_Me;
        delay_us_me(5);
        if (data & 0x80)
            SDA_H_Me;
        else
            SDA_L_Me;
        data <<= 1;
        delay_us_me(5);
        SCL_H_Me;
        delay_us_me(5);
    }
    SCL_L_Me;
}   
/*主机接收i2c总线上一个字节数据*/
uint8_t I2C_ReciveByte_Me(void)            
{
	  uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H_Me;
    while (i--) {
        byte <<= 1;
        SCL_L_Me;
        delay_us_me(5); 
        SCL_H_Me;
        delay_us_me(5); 
        if (SDA_Read_Me) {
            byte |= 0x01;
        }
    }
    SCL_L_Me;
    return byte;
}
/*主机向i2c总线发送连续读信号*/
void I2C_ack_Me(void)          
{  

	SCL_L_Me;//SCL=0;
	delay_us_me(5); 
	SDA_L_Me;//SDA=0;
	delay_us_me(5);
	SCL_H_Me;  //SCL=1;
	delay_us_me(5); 
	SCL_L_Me;//SCL=0;
	delay_us_me(5); 
}
/*主机向i2c总线发送不连续读信号*/
void I2C_nack_Me(void)         
{  

	SCL_L_Me;//SCL=0;
	delay_us_me(5); 
	SDA_H_Me;//SDA=1;
	delay_us_me(5);
	SCL_H_Me;  //SCL=1;
	delay_us_me(5);
	SCL_L_Me;//SCL=0;
	delay_us_me(5); 
}

/*在指定地址指定寄存器写值*/
void I2C_Write_Me(uint8_t addr,uint8_t reg,uint8_t data)
{
	I2C_Start_Me();
	
	I2C_SendByte_Me(addr);
	while(I2C_Slave_ACK_Me());
	
	I2C_SendByte_Me(reg);
	while(I2C_Slave_ACK_Me());	

	I2C_SendByte_Me(data);
	while(I2C_Slave_ACK_Me());	

	I2C_Stop_Me();
}

/*在指定地址写命令*/
void I2C_Write_Me_MS5611(uint8_t addr,uint8_t commands)
{
	I2C_Start_Me();
	
	I2C_SendByte_Me(addr);
	while(I2C_Slave_ACK_Me());
	
	I2C_SendByte_Me(commands);
 	while(I2C_Slave_ACK_Me());	
	
	I2C_Stop_Me();
}

/*在指定地址指定寄存器读值*/
uint8_t I2C_Read_Me(uint8_t addr1,uint8_t addr2,uint8_t reg)
{
	uint8_t p;
	I2C_Start_Me();
	
	I2C_SendByte_Me(addr1);
	while(I2C_Slave_ACK_Me());
	
	I2C_SendByte_Me(reg);
	while(I2C_Slave_ACK_Me());
	
	I2C_Stop_Me();
	I2C_Start_Me();
	
	I2C_SendByte_Me(addr2);
	while(I2C_Slave_ACK_Me());
	
	p=I2C_ReciveByte_Me();
	I2C_nack_Me();
	I2C_Stop_Me();
	return p;
}

/*在指定地址读16位数*/
uint16_t I2C_Read_Me_MS5611_16BIT(uint8_t addr)
{
	uint16_t temp[2];
	I2C_Start_Me();
	
	I2C_SendByte_Me(addr);
	while(I2C_Slave_ACK_Me());
	
	temp[0]=I2C_ReciveByte_Me();
	I2C_ack_Me();
	
	temp[1]=I2C_ReciveByte_Me();
	I2C_nack_Me();
	
	I2C_Stop_Me();
	return (temp[0]<<8)+temp[1];
}

/*在指定地址读24位数*/
uint32_t I2C_Read_Me_MS5611_24BIT(uint8_t addr)
{
	uint32_t temp[3];
	I2C_Start_Me();
	
	I2C_SendByte_Me(addr);
	while(I2C_Slave_ACK_Me());

	temp[0]=I2C_ReciveByte_Me();
	I2C_ack_Me();
	
	temp[1]=I2C_ReciveByte_Me();
	I2C_ack_Me();
	
	temp[2]=I2C_ReciveByte_Me();
	I2C_nack_Me();
	I2C_Stop_Me();
	return (temp[0]<<16)|(temp[1]<<8)|temp[2];
}