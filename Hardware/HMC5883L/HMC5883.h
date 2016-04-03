#ifndef _HMC5883_H_
#define _HMC5883_H_

/* HMC5883L Register Address ------------------------------------------------------------*/

#define	HMC5883L_CONFIG_A					0x00
#define	HMC5883L_CONFIG_B					0x01
#define	HMC5883L_MODE						0x02
#define	HMC5883L_X_MSB						0x03
#define	HMC5883L_X_LSB						0x04
#define	HMC5883L_Z_MSB						0x05
#define	HMC5883L_Z_LSB						0x06
#define	HMC5883L_Y_MSB						0x07
#define	HMC5883L_Y_LSB						0x08
#define	HMC5883L_STATUS						0x09
#define	HMC5883L_IDEN_REG_A				0x10
#define	HMC5883L_IDEN_REG_B				0x11
#define	HMC5883L_IDEN_REG_C				0x12
#define HMC5883L_READ_ADDRESS     0x3d
#define HMC5883L_WRITE_ADDRESS    0x3c

#define MPU6050_INT_PIN_CFG       0x37//第二位设为1
#define MPU6050_I2C_BYPASS_EN     0x42//原始：0x02

#define MPU6050_USER_CTRL       0x6A    //用户配置寄存器 打开值：0x40  AUX_DA的辅助I2C
#define MPU6050_I2C_MAS_DIS		0x40

// Single_Write(MPU6050_Addr,INT_PIN_CFG, 0x42);   //使能旁路I2C
// Single_Write(MPU6050_Addr,USER_CTRL, 0x40);     //使能旁路I2C


void 	HMC5883L_Init(void);
uint8_t GetData_HMC5883L(u8 REG_Address);
float Read_HMC5883L(void);


typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;

extern S_INT16_XYZ HMC5883L_MAGN_LAST;


#endif
