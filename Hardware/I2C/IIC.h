#ifndef _IIC_H_
#define _IIC_H_


#ifndef AUXILIARY_IIC

#define HCMI2C_SCL_Pin 		GPIO_Pin_4
#define HCMI2C_SDA_Pin 		GPIO_Pin_5
#define HCMI2C_PORT   		GPIOB


#define SCL_L_Me        GPIO_ResetBits(HCMI2C_PORT, HCMI2C_SCL_Pin)
#define SCL_H_Me  		GPIO_SetBits(HCMI2C_PORT, HCMI2C_SCL_Pin)
#define SDA_L_Me  		GPIO_ResetBits(HCMI2C_PORT, HCMI2C_SDA_Pin)
#define SDA_H_Me  		GPIO_SetBits(HCMI2C_PORT, HCMI2C_SDA_Pin)
#define SCL_Read_Me     GPIO_ReadInputDataBit(HCMI2C_PORT , HCMI2C_SCL_Pin) 
#define SDA_Read_Me     GPIO_ReadInputDataBit(HCMI2C_PORT , HCMI2C_SDA_Pin)

#else

#define SCL_L_Me        GPIO_ResetBits(MPUI2C_PORT, MPUI2C_SCL_Pin)
#define SCL_H_Me  		GPIO_SetBits(MPUI2C_PORT, MPUI2C_SCL_Pin)
#define SDA_L_Me  		GPIO_ResetBits(MPUI2C_PORT, MPUI2C_SDA_Pin)
#define SDA_H_Me  		GPIO_SetBits(MPUI2C_PORT, MPUI2C_SDA_Pin)
#define SCL_Read_Me     GPIO_ReadInputDataBit(MPUI2C_PORT , MPUI2C_SCL_Pin) 
#define SDA_Read_Me     GPIO_ReadInputDataBit(MPUI2C_PORT , MPUI2C_SDA_Pin)

#endif

void delay_us_me(uint32_t m);
void I2C_Start_Me(void);
void I2C_Stop_Me(void);
int I2C_Slave_ACK_Me(void);
void I2C_SendByte_Me(uint8_t data);
uint8_t I2C_ReciveByte_Me(void);
void I2C_ack_Me(void);
void I2C_nack_Me(void);
void I2C_Write_Me(uint8_t addr,uint8_t reg,uint8_t data);
uint8_t I2C_Read_Me(uint8_t addr1,uint8_t addr2,uint8_t reg);
void I2C_Write_Me_MS5611(uint8_t addr,uint8_t commands);
uint16_t I2C_Read_Me_MS5611_16BIT(uint8_t addr);
uint32_t I2C_Read_Me_MS5611_24BIT(uint8_t addr);


#endif