#include "bsp.h"
#include "HMC5883.h"
#include "math.h"
#include "IIC.h"


/**************************************
 * 函数名：void HMC5883L_Init(void)
 * 描述  ：初始化HMC5883L
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void HMC5883L_Init(void)
{	

#ifdef AUXILIARY_IIC		
	I2C_Write_Me(0xD0, MPU6050_USER_CTRL, MPU6050_I2C_MAS_DIS);	
	I2C_Write_Me(0xD0, MPU6050_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN);	
#endif

	OSTimeDly(10);
	I2C_Write_Me(HMC5883L_WRITE_ADDRESS,HMC5883L_CONFIG_A,0x78);	
	OSTimeDly(10);
	I2C_Write_Me(HMC5883L_WRITE_ADDRESS,HMC5883L_MODE,0x00);
	OSTimeDly(10);
}




/*************************************
 * 函数名：GetData_HMC5883L
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 ************************************/

uint8_t GetData_HMC5883L(u8 REG_Address)
{
	return I2C_Read_Me(HMC5883L_WRITE_ADDRESS,HMC5883L_READ_ADDRESS,REG_Address);
}

S_INT16_XYZ HMC5883L_MAGN_LAST;

/*************************************
 * 函数名：Read_HMC5883L
 * 描述  ：获得16位数据，求得磁场的X,Y,Z三轴的分量
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 ************************************/
float Read_HMC5883L(void)//读HMC5883L的数据，输出为16位经过整合的
{
	int16_t x,y,z;
	x=(int16_t)(((int16_t)GetData_HMC5883L(HMC5883L_X_MSB)<<8)|(GetData_HMC5883L(HMC5883L_X_LSB)));
	y=(int16_t)(((int16_t)GetData_HMC5883L(HMC5883L_Y_MSB)<<8)|(GetData_HMC5883L(HMC5883L_Y_LSB)));
	z=(int16_t)(((int16_t)GetData_HMC5883L(HMC5883L_Z_MSB)<<8)|(GetData_HMC5883L(HMC5883L_Z_LSB)));
	HMC5883L_MAGN_LAST.X=(x>0x7fff?x-=0xffff:x);
	HMC5883L_MAGN_LAST.Y=(y>0x7fff?y-=0xffff:y);
	HMC5883L_MAGN_LAST.Z=(z>0x7fff?z-=0xffff:z);
	return atan2(HMC5883L_MAGN_LAST.Y,HMC5883L_MAGN_LAST.X)*(57.2957796);
}








