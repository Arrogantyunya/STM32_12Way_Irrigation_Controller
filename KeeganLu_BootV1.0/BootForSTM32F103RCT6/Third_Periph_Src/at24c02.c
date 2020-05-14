/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/8/27
 * 
 * 该文件主要功能是操作EEPROM储存芯片，提供了初始化I2C总线，读取一个字节、写一个字节等函数接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "../Third_Periph_Inc/at24Cxx.h"

/*
 @brief     : 从EEPROM读取一个字节
 @param     : 无
 @return    : 1 byte
 */
unsigned char AT24Cxx_ReadOneByte(unsigned int addr)
{
	unsigned char Temp = 0;
	I2C_Start();

	if (EE_TYPE > AT24C16)
	{
		I2C_Send_Byte(0xA0);
		I2C_Wait_Ack();
		I2C_Send_Byte(addr >> 8);
	}
	else
		I2C_Send_Byte(0xA0 + ((addr / 256) << 1)); //设备地址+数据地址

	I2C_Wait_Ack();
	I2C_Send_Byte(addr % 256);
	I2C_Wait_Ack();

	I2C_Start();
	I2C_Send_Byte(0xA1);
	I2C_Wait_Ack();

	Temp = I2C_Read_Byte(0); 
	I2C_NAck();
	I2C_Stop();
	return Temp;
}

/*
 @brief     : 写一个字节到EEPROM
 @param     : 1.写入数据地址
              2.数据值
 @return    : 无
 */
void AT24Cxx_WriteOneByte(unsigned int addr, unsigned char dt)
{
    I2C_Start();

    if (EE_TYPE > AT24C16)
    {
        I2C_Send_Byte(0xA0);
        I2C_Wait_Ack();
        I2C_Send_Byte(addr >> 8);
    }
    else
        I2C_Send_Byte(0xA0 + ((addr / 256) << 1));

    I2C_Wait_Ack();
    I2C_Send_Byte(addr % 256);
    I2C_Wait_Ack();

    I2C_Send_Byte(dt);
    I2C_Wait_Ack();
    I2C_Stop();
    HAL_Delay(10);
}

#if USE_EP_WP
void EEPROM_WP_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	EP_WP_RCC_CLK_ENABLE;
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = EP_WP_GPIO_PIN;
	HAL_GPIO_Init(EP_WP_GPIO_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(EP_WP_GPIO_PORT, EP_WP_GPIO_PIN, GPIO_PIN_SET);
}

void EEPROM_WP_En(void)
{
	HAL_GPIO_WritePin(EP_WP_GPIO_PORT, EP_WP_GPIO_PIN, GPIO_PIN_RESET);
}

void EEPROM_WP_Dis(void)
{
	HAL_GPIO_WritePin(EP_WP_GPIO_PORT, EP_WP_GPIO_PIN, GPIO_PIN_SET);
}
#endif
