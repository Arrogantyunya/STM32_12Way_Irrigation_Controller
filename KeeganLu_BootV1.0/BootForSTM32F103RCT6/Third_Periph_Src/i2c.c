
#include "../Third_Periph_Inc/i2c.h"

void Delay_us(unsigned int us)
{
	unsigned int i;
	for (i = 0; i < (us * 72); i++);
}

/*
 @brief     : 初始化I2C总线
 @para      : 无
 @return    : 无
 */
void I2C_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	I2C_SCL_RCC_CLK_ENABLE;
	I2C_SDA_RCC_CLK_ENABLE;
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	GPIO_InitStruct.Pin = I2C_SCL_GPIO_PIN;
	HAL_GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = I2C_SDA_GPIO_PIN;
	HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStruct);

	I2C_SCL_H;
	I2C_SDA_H;
}

/*
 @brief     : I2C数据线配置输出模式
 @para      : 无
 @return    : 无
 */
void I2C_SDA_Out(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	GPIO_InitStruct.Pin = I2C_SDA_GPIO_PIN;
	HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/*
 @brief     : I2C数据线配置输入模式
 @para      : 无
 @return    : 无
 */
void I2C_SDA_In(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	
	GPIO_InitStruct.Pin = I2C_SDA_GPIO_PIN;
	HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/*
 @brief     : I2C开始传输数据时序
 @para      : 无
 @return    : 无
 */
void I2C_Start(void)
{
	I2C_SDA_Out();
	I2C_SDA_H;
	I2C_SCL_H;
	Delay_us(5);
	I2C_SDA_L;
	Delay_us(6);
	I2C_SCL_L;
}

/*
 @brief     : I2C结束传输数据时序
 @para      : 无
 @return    : 无
 */
void I2C_Stop(void)
{
	I2C_SDA_Out();
	I2C_SCL_L;
	I2C_SDA_L;
	I2C_SCL_H;
	Delay_us(6);
	I2C_SDA_H;
	Delay_us(6);
}

/*
 @brief     : 主机数据线拉低应答
 @para      : 无
 @return    : 无
 */
void I2C_Ack(void)
{
	I2C_SCL_L;
	I2C_SDA_Out();
	I2C_SDA_L;
	Delay_us(2);
	I2C_SCL_H;
	Delay_us(5);
	I2C_SCL_L;
}

/*
 @brief     : 主机数据线拉高非应答
 @para      : 无
 @return    : 无
 */
void I2C_NAck(void)
{
	I2C_SCL_L;
	I2C_SDA_Out();
	I2C_SDA_H;
	Delay_us(2);
	I2C_SCL_H;
	Delay_us(5);
	I2C_SCL_L;
}

/*
 @brief     : 等待从机应答， 从机返回1接收应答失败，返回0接收应答成功
 @para      : 无
 @return    : 等待应答结果
 */
unsigned char I2C_Wait_Ack(void)
{
	unsigned char tempTime = 0;
	I2C_SDA_In();
	I2C_SDA_H;
	Delay_us(1);
	I2C_SCL_H;
	Delay_us(1);
	while (HAL_GPIO_ReadPin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN))
	{
			tempTime++;
			if (tempTime > 250) //等待从机返回0失败
			{ 
					I2C_Stop();
					return 1;
			}
	}
	I2C_SCL_L;
	return 0;
}

/*
 @brief     : I2C发送一个字节
 @para      : 1byte
 @return    : 无
 */
void I2C_Send_Byte(unsigned char data)
{
	unsigned char i = 0;
	I2C_SDA_Out();
	I2C_SCL_L; //拉低时钟线，允许数据线上电平变化

	for (i = 0; i < 8; i++)
	{
			(data & 0x80) > 0 ? I2C_SDA_H : I2C_SDA_L;//从一个字节的高位开始传送
			data <<= 1;
			I2C_SCL_H; //时钟线拉高，这时数据线电平不能变化，让从机读取线上的电平
			Delay_us(2);
			I2C_SCL_L;
			Delay_us(2);
	}
}

/*
 @brief     : I2C读取一个字节
 @para      : 选择应答或非应答
 @return    : 1 byte
 */
unsigned char I2C_Read_Byte(unsigned char ack)
{
	unsigned char i = 0, receive = 0;
	I2C_SDA_In();

	for (i = 0; i < 8; i++)
	{
			I2C_SCL_L;
			Delay_us(2);
			I2C_SCL_H; //拉高时钟线，去读从机回过来的数据
			receive <<= 1;

			if (HAL_GPIO_ReadPin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN))
					receive++;
			Delay_us(1);
	}
	ack == 0 ? I2C_NAck() : I2C_Ack();

	return receive;
}
