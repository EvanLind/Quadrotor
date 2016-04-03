#ifndef _MPUI2C_H_
#define _MPUI2C_H_



//
#define MPUI2C_SCL_Pin 		GPIO_Pin_6
#define MPUI2C_SDA_Pin 		GPIO_Pin_7
#define MPUI2C_PORT   		GPIOB
//
#define I2C_SCL_Res 		GPIO_ResetBits(MPUI2C_PORT, MPUI2C_SCL_Pin)
#define I2C_SCL_Set 		GPIO_SetBits(MPUI2C_PORT, MPUI2C_SCL_Pin)
#define I2C_SDA_Res 		GPIO_ResetBits(MPUI2C_PORT, MPUI2C_SDA_Pin)
#define I2C_SDA_Set   		GPIO_SetBits(MPUI2C_PORT, MPUI2C_SDA_Pin)
//
#define I2C_SDA_STATE   	GPIO_ReadInputDataBit(MPUI2C_PORT, MPUI2C_SDA_Pin)
#define I2C_DELAY 			I2C_Delay(100000)
#define I2C_NOP				I2C_Delay(10) 
//
#define I2C_READY		0x00
#define I2C_BUS_BUSY	0x01	
#define I2C_BUS_ERROR	0x02
//
#define I2C_NACK	  0x00 
#define I2C_ACK		0x01
//




void Single_WriteI2C(uint8_t dev_addr, unsigned char reg_addr,unsigned char REG_data);//单字节写入
unsigned char Single_ReadI2C(uint8_t dev_addr, unsigned char reg_addr);//读取单字节


void I2CInit(GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_SCL, uint32_t _GPIO_PinX_SDA);
void I2C_Delay(uint32_t dly);
uint8_t I2C_START(void);
void I2C_STOP(void);
void I2C_SendACK(void);
void I2C_SendNACK(void);
uint8_t I2C_SendByte(uint8_t i2c_data);
uint8_t I2C_ReceiveByte_WithACK(void);
uint8_t I2C_ReceiveByte(void);
void I2C_Receive12Bytes(uint8_t *i2c_data_buffer);
void I2C_Receive6Bytes(uint8_t *i2c_data_buffer);
void I2C_Receive14Bytes(uint8_t *i2c_data_buffer);
uint8_t DMP_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t i2c_len, uint8_t *i2c_data_buf);
uint8_t DMP_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t i2c_len, uint8_t *i2c_data_buf);
void DMP_Delay_us(uint32_t dly);
void DMP_Delay_ms(uint32_t dly);


#endif


// #ifndef _I2C_H_
// #define _I2C_H_


// // //IO方向设置
// // #define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
// // #define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
// // //IO操作函数
// // #define IIC_SCL    PBout(8) //SCL
// // #define IIC_SDA    PBout(9) //SDA
// // #define READ_SDA   PBin(9)  //输入SDA

// // //IIC所有操作函数
// // void IIC_Init(void);                //初始化IIC的IO口
// // void IIC_Start(void);				//发送IIC开始信号
// // void IIC_Stop(void);	  			//发送IIC停止信号
// // void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
// // uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
// // uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
// // void IIC_Ack(void);					//IIC发送ACK信号
// // void IIC_NAck(void);				//IIC不发送ACK信号

// // void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
// // uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);

// // /*********************************************************硬件IIc******************************************************************************/

// // void SingleWriteI2C(I2C_TypeDef* I2Cx, unsigned char slaveAddr, unsigned char regAddr, unsigned char sendData);
// // void SingleReadI2C(I2C_TypeDef* I2Cx, unsigned char slaveAddr, unsigned char regAddr, unsigned char* recvData, unsigned short numByteToRead);



// /*模拟IIC端口输出输入定义*/
// #define I2CPort		GPIOB
// #define I2CSclPin	GPIO_Pin_6
// #define I2CSdaPin	GPIO_Pin_7
// #define SCL_H		I2CPort->BSRRL = I2CSclPin
// #define SCL_L       I2CPort->BSRRH  = I2CSclPin 
// #define SDA_H       I2CPort->BSRRL = I2CSdaPin
// #define SDA_L       I2CPort->BSRRH  = I2CSdaPin
// #define SCL_read    I2CPort->IDR  & I2CSclPin
// #define SDA_read    I2CPort->IDR  & I2CSdaPin



// void I2CInit(GPIO_TypeDef *_sclPort, uint32_t _sclPin, GPIO_TypeDef *_sdaPort, uint32_t _sdaPin);
// void I2C_delay(void);
// Boolean I2C_Start(void);
// void I2C_Stop(void);
// void I2C_Ack(void);
// void I2C_NoAck(void);
// Boolean I2C_WaitAck(void);
// void I2C_SendByte(unsigned char SendByte);
// unsigned char I2C_RadeByte(void);
// void SingleWriteI2C(unsigned char slaveAddr, unsigned char regAddr,unsigned char sendData);
// unsigned char SingleReadI2C(unsigned char slaveAddr, unsigned char regAddr);


// #endif