#include "bsp.h"
#include "rcs.h"
#include <math.h>
#include "MPUI2C.h"




void I2CInit(GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_SCL, uint32_t _GPIO_PinX_SDA)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_GPIOx), ENABLE);

    // GPIO_PinAFConfig(_GPIOx, GetRCS_GPIO_PinSource(_GPIO_PinX_SCL), GetRCS_I2C_AF(_I2Cx));
    // GPIO_PinAFConfig(_GPIOx, GetRCS_GPIO_PinSource(_GPIO_PinX_SDA), GetRCS_I2C_AF(_I2Cx));

    GPIO_StructInit( &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  _GPIO_PinX_SCL | _GPIO_PinX_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(_GPIOx, &GPIO_InitStructure);

	// I2C_SCL_Set;
	// I2C_SDA_Set;
	// I2C_DELAY;
}

void I2C_Delay(uint32_t dly)
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

uint8_t I2C_START(void)
{ 
	I2C_SDA_Set; 
 	I2C_NOP;
  // 
 	I2C_SCL_Set; 
 	I2C_NOP;    
	//
 	if(!I2C_SDA_STATE) return I2C_BUS_BUSY;
	//
 	I2C_SDA_Res;
 	I2C_NOP;
  //
 	I2C_SCL_Res;  
 	I2C_NOP; 
	//
 	if(I2C_SDA_STATE) return I2C_BUS_ERROR;
	//
 	return I2C_READY;
}

void I2C_STOP(void)
{
 	I2C_SDA_Res; 
 	I2C_NOP;
  // 
 	I2C_SCL_Set; 
 	I2C_NOP;    
	//
 	I2C_SDA_Set;
 	I2C_NOP;
}

void I2C_SendACK(void)
{
 	I2C_SDA_Res;
 	I2C_NOP;
 	I2C_SCL_Set;
 	I2C_NOP;
 	I2C_SCL_Res; 
 	I2C_NOP;  
}

void I2C_SendNACK(void)
{
	I2C_SDA_Set;
	I2C_NOP;
	I2C_SCL_Set;
	I2C_NOP;
	I2C_SCL_Res; 
	I2C_NOP;  
}

uint8_t I2C_SendByte(uint8_t i2c_data)
{
 	uint8_t i;
 	
	I2C_SCL_Res;
 	for(i=0;i<8;i++)
 	{  
  		if(i2c_data&0x80) I2C_SDA_Set;
   		else I2C_SDA_Res;
			//
  		i2c_data<<=1;
  		I2C_NOP;
			//
  		I2C_SCL_Set;
  		I2C_NOP;
  		I2C_SCL_Res;
  		I2C_NOP; 
 	}
	//
 	I2C_SDA_Set; 
 	I2C_NOP;
 	I2C_SCL_Set;
 	I2C_NOP;   
 	if(I2C_SDA_STATE)
 	{
  		I2C_SCL_Res;
  		return I2C_NACK;
 	}
 	else
 	{
  		I2C_SCL_Res;
  		return I2C_ACK;  
 	}    
}

uint8_t I2C_ReceiveByte(void)
{
	uint8_t i,i2c_data;
	//
 	I2C_SDA_Set;
 	I2C_SCL_Res; 
 	i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		I2C_SCL_Set;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_Res;  
  		I2C_NOP;         
 	}
	I2C_SendNACK();
 	return i2c_data;
}

uint8_t I2C_ReceiveByte_WithACK(void)
{
	uint8_t i,i2c_data;
	//
 	I2C_SDA_Set;
 	I2C_SCL_Res; 
 	i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		I2C_SCL_Set;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_Res;  
  		I2C_NOP;         
 	}
	I2C_SendACK();
 	return i2c_data;
}

void I2C_Receive6Bytes(uint8_t *i2c_data_buffer)
{
	uint8_t i,j;
	uint8_t i2c_data;

	for(j=0;j<5;j++)
	{
		I2C_SDA_Set;
		I2C_SCL_Res; 
		i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		I2C_SCL_Set;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_Res;  
  		I2C_NOP;         
		}
		i2c_data_buffer[j]=i2c_data;
		I2C_SendACK();
	}
	//
	I2C_SDA_Set;
	I2C_SCL_Res; 
	i2c_data=0;
	for(i=0;i<8;i++)
	{
  	I2C_SCL_Set;
  	I2C_NOP; 
  	i2c_data<<=1;
			//
  	if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  	I2C_SCL_Res;  
  	I2C_NOP;         
	}
	i2c_data_buffer[5]=i2c_data;
	I2C_SendNACK();
}

void I2C_Receive12Bytes(uint8_t *i2c_data_buffer)
{
	uint8_t i,j;
	uint8_t i2c_data;

	for(j=0;j<11;j++)
	{
		I2C_SDA_Set;
		I2C_SCL_Res; 
		i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		I2C_SCL_Set;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_Res;  
  		I2C_NOP;         
		}
		i2c_data_buffer[j]=i2c_data;
		I2C_SendACK();
	}
	//
	I2C_SDA_Set;
	I2C_SCL_Res; 
	i2c_data=0;
	for(i=0;i<8;i++)
	{
  	I2C_SCL_Set;
  	I2C_NOP; 
  	i2c_data<<=1;
			//
  	if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  	I2C_SCL_Res;  
  	I2C_NOP;         
	}
	i2c_data_buffer[11]=i2c_data;
	I2C_SendNACK();
}

void I2C_Receive14Bytes(uint8_t *i2c_data_buffer)
{
	uint8_t i,j;
	uint8_t i2c_data;

	for(j=0;j<13;j++)
	{
		I2C_SDA_Set;
		I2C_SCL_Res; 
		i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		I2C_SCL_Set;
  		I2C_NOP; 
  		i2c_data<<=1;
			//
  		if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  		I2C_SCL_Res;  
  		I2C_NOP;         
		}
		i2c_data_buffer[j]=i2c_data;
		I2C_SendACK();
	}
	//
	I2C_SDA_Set;
	I2C_SCL_Res; 
	i2c_data=0;
	for(i=0;i<8;i++)
	{
  	I2C_SCL_Set;
  	I2C_NOP; 
  	i2c_data<<=1;
			//
  	if(I2C_SDA_STATE)	i2c_data|=0x01; 
  
  	I2C_SCL_Res;  
  	I2C_NOP;         
	}
	i2c_data_buffer[13]=i2c_data;
	I2C_SendNACK();
}

void DMP_Delay_us(uint32_t dly)
{
	uint8_t i;
	while(dly--) for(i=0;i<10;i++);
}
//
void DMP_Delay_ms(uint32_t dly)
{
	while(dly--) DMP_Delay_us(1000);
}
//


uint8_t DMP_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t i2c_len, uint8_t *i2c_data_buf)
{		
		uint8_t i;
		I2C_START();
		I2C_SendByte(dev_addr << 1 | I2C_Direction_Transmitter);					//圆点博士:发送陀螺仪写地址
		I2C_SendByte(reg_addr);  //圆点博士:发送陀螺仪PWM地址
		for (i=0;i<i2c_len;i++) I2C_SendByte(i2c_data_buf[i]); //圆点博士:发送陀螺仪PWM值
		I2C_STOP();
		return 0x00;
}

uint8_t DMP_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t i2c_len, uint8_t *i2c_data_buf)
{
	
	I2C_START();
	I2C_SendByte(dev_addr << 1 | I2C_Direction_Transmitter);			//圆点博士:发送陀螺仪写地址
	I2C_SendByte(reg_addr);  //圆点博士:发送陀螺仪ID地址
	I2C_START();
	I2C_SendByte(dev_addr << 1 | I2C_Direction_Receiver);      //圆点博士:发送陀螺仪读地址
		//
    while (i2c_len)
	{
		if (i2c_len==1) *i2c_data_buf =I2C_ReceiveByte();  
		else *i2c_data_buf =I2C_ReceiveByte_WithACK();
		i2c_data_buf++;
		i2c_len--;
    }
		I2C_STOP();
    return 0x00;
}

/*I2C向指定设备指定地址写入u8数据-----------------------*/
void Single_WriteI2C(uint8_t dev_addr, unsigned char reg_addr,unsigned char REG_data)//单字节写入
{
    I2C_START();
    I2C_SendByte(dev_addr);   //发送设备地址+写信号//I2C_SendByte(((reg_addr & 0x0700) >>7) | dev_addr & 0xFFFE);//设置高起始地址+器件地址 
    I2C_SendByte(reg_addr );   //设置低起始地址        
    I2C_SendByte(REG_data);  
    I2C_STOP(); 
}

/*I2C向指定设备指定地址读出u8数据-----------------------*/
unsigned char Single_ReadI2C(uint8_t dev_addr, unsigned char reg_addr)//读取单字节
{   
    unsigned char REG_data;         
    I2C_START();
    I2C_SendByte(dev_addr); //I2C_SendByte(((reg_addr & 0x0700) >>7) | reg_addr & 0xFFFE);//设置高起始地址+器件地址 
    I2C_SendByte(reg_addr);   //设置低起始地址      
    I2C_START();
    I2C_SendByte(dev_addr+1);
   
    REG_data= I2C_ReceiveByte();
    I2C_STOP();
    //return TRUE;
    return REG_data;
}




// #include "bsp.h"
// #include "rcs.h"
// #include <math.h>
// #include "I2C.h"



// void I2CInit(GPIO_TypeDef *_sclPort, uint32_t _sclPin, GPIO_TypeDef *_sdaPort, uint32_t _sdaPin)
// {
// 	SetGpioOutput(_sclPort, _sclPin);
// 	SetGpioOutput(_sdaPort, _sdaPin);
// }

// /*I2C的延时函数-----------------------------------------*/
// void I2C_delay(void)
// {
// 	unsigned char i=100; //这里可以优化速度    ，经测试最低到5还能写入
// 	while(i) 
// 	{ 
// 		i--; 
// 	}  
// }

// /*I2C启动函数-------------------------------------------*/
// Boolean I2C_Start(void)
// {
// 	SDA_H;
// 	SCL_H;
// 	I2C_delay();
// 	if(!SDA_read)return FALSE;    //SDA线为低电平则总线忙,退出
// 	SDA_L;
// 	I2C_delay();
// 	if(SDA_read) return FALSE;    //SDA线为高电平则总线出错,退出
// 	SDA_L;
// 	I2C_delay();
// 	return TRUE;
// }

// /*I2C停止函数-------------------------------------------*/
// void I2C_Stop(void)
// {
// 	SCL_L;
// 	I2C_delay();
// 	SDA_L;
// 	I2C_delay();
// 	SCL_H;
// 	I2C_delay();
// 	SDA_H;
// 	I2C_delay();
// } 

// /*I2C的ACK函数------------------------------------------*/
// void I2C_Ack(void)
// {    
// 	SCL_L;
// 	I2C_delay();
// 	SDA_L;
// 	I2C_delay();
// 	SCL_H;
// 	I2C_delay();
// 	SCL_L;
// 	I2C_delay();
// }   

// /*I2C的NoACK函数----------------------------------------*/
// void I2C_NoAck(void)
// {    
// 	SCL_L;
// 	I2C_delay();
// 	SDA_H;
// 	I2C_delay();
// 	SCL_H;
// 	I2C_delay();
// 	SCL_L;
// 	I2C_delay();
// } 

// /*I2C等待ACK函数----------------------------------------*/
// Boolean I2C_WaitAck(void)      //返回为:=1有ACK,=0无ACK
// {
// 	SCL_L;
// 	I2C_delay();
// 	SDA_H;            
// 	I2C_delay();
// 	SCL_H;
// 	I2C_delay();
// 	if(SDA_read)
// 	{
// 		SCL_L;
// 		I2C_delay();
// 		return FALSE;
// 	}
// 	SCL_L;
// 	I2C_delay();
// 	return TRUE;
// }

// /*I2C发送一个unsigned char数据函数---------------------------------*/
// void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
// {
// 	unsigned char i=8;
// 	while(i--)
// 	{
// 		SCL_L;
// 		I2C_delay();
// 		if(SendByte&0x80)
// 			SDA_H;  
// 		else 
// 			SDA_L;   
// 		SendByte<<=1;
// 		I2C_delay();
// 		SCL_H;
// 		I2C_delay();
// 	}
// 	SCL_L;
// }  

// /*I2C读取一个unsigned char数据函数---------------------------------*/
// unsigned char I2C_RadeByte(void)  //数据从高位到低位//
// { 
// 	unsigned char i=8;
// 	unsigned char ReceiveByte=0;
	
// 	SDA_H;                
// 	while(i--)
// 	{
// 		ReceiveByte<<=1;      
// 		SCL_L;
// 		I2C_delay();
// 		SCL_H;
// 		I2C_delay();    
// 		if(SDA_read)
// 		{
// 			ReceiveByte|=0x01;
// 		}
// 	}
// 	SCL_L;
// 	return ReceiveByte;
// }  

// /*I2C向指定设备指定地址写入unsigned char数据-----------------------*/
// void SingleWriteI2C(unsigned char slaveAddr, unsigned char regAddr,unsigned char sendData)//单字节写入
// {
// 	if(!I2C_Start())return;
// 	I2C_SendByte(slaveAddr);   //发送设备地址+写信号//I2C_SendByte(((regAddr & 0x0700) >>7) | slaveAddr & 0xFFFE);//设置高起始地址+器件地址 
// 	if(!I2C_WaitAck()){I2C_Stop(); return;}
// 	I2C_SendByte(regAddr);   //设置低起始地址      
// 	I2C_WaitAck();    
// 	I2C_SendByte(sendData);
// 	I2C_WaitAck();   
// 	I2C_Stop();
// }

// /*I2C向指定设备指定地址读出unsigned char数据-----------------------*/
// unsigned char SingleReadI2C(unsigned char slaveAddr, unsigned char regAddr)//读取单字节
// {   
// 	unsigned char recvData;         
// 	if(!I2C_Start())return FALSE;
// 	I2C_SendByte(slaveAddr);
// 	if(!I2C_WaitAck())
// 	{
// 		I2C_Stop(); 
// 		return FALSE;
// 	}
// 	I2C_SendByte((unsigned char) regAddr);   //设置低起始地址      
// 	I2C_WaitAck();
// 	I2C_Start();
// 	I2C_SendByte(slaveAddr+1);
// 	I2C_WaitAck();
	
// 	recvData= I2C_RadeByte();
// 	I2C_NoAck();
// 	I2C_Stop();
// 	return recvData;
// }






// /*********************************************************硬件IIc******************************************************************************/
// // void SingleWriteI2C(I2C_TypeDef* I2Cx, unsigned char slaveAddr,  unsigned char regAddr, unsigned char sendData)
// // {
// // 	I2C_GenerateSTART(I2Cx, ENABLE);/* Send START condition */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));/* Test on EV5 and clear it */
// // 	I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);/* Send MPU6050 address for write */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
// // 	I2C_SendData(I2Cx, regAddr);/* Send the MPU6050's internal address to write to */
// // 	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
// // 	I2C_SendData(I2Cx, sendData);/* Send the byte to be written */
// // 	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
// // 	I2C_GenerateSTOP(I2Cx, ENABLE);
// // }


// // void SingleReadI2C(I2C_TypeDef* I2Cx, unsigned char slaveAddr, unsigned char regAddr, unsigned char* recvData, unsigned short numByteToRead)
// // {
// // 	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));/* While the bus is busy */
// // 	I2C_GenerateSTART(I2Cx, ENABLE);/* Send START condition */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));/* Test on EV5 and clear it */
// // 	I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter); /* Send MPU6050 address for write */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));/* Test on EV6 and clear it */
// // 	I2C_Cmd(I2Cx, ENABLE);/* Clear EV6 by setting again the PE bit */
// // 	I2C_SendData(I2Cx, regAddr);/* Send the MPU6050's internal address to write to */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));/* Test on EV8 and clear it */
// // 	I2C_GenerateSTART(I2Cx, ENABLE);/* Send STRAT condition a second time */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));/* Test on EV5 and clear it */
// // 	I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);/* Send MPU6050 address for read */
// // 	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));/* Test on EV6 and clear it */
// // 	while(numByteToRead)/* While there is data to be read */
// // 	{
// // 		if(numByteToRead == 1)
// // 		{
// // 			I2C_AcknowledgeConfig(I2Cx, DISABLE);
// // 			I2C_GenerateSTOP(I2Cx, ENABLE);
// // 		}
// // 		if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))/* Test on EV7 and clear it */
// // 		{
// // 			*recvData = I2C_ReceiveData(I2Cx);/* Read a byte from the MPU6050 */
// // 			recvData++;/* Point to the next location where the byte read will be saved */
// // 			numByteToRead--;/* Decrement the read bytes counter */
// // 		}
// // 	}
// // 	I2C_AcknowledgeConfig(I2Cx, ENABLE);/* Enable Acknowledgement to be ready for another reception */
// // }