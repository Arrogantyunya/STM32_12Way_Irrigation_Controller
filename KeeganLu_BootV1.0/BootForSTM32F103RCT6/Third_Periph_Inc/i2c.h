
#ifndef _I2C_H
#define _I2C_H

#include "../bt_inc/boot_config.h"


#define I2C_SDA_RCC_CLK_ENABLE			__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C_SDA_GPIO_PIN						GPIO_PIN_7
#define I2C_SDA_GPIO_PORT						GPIOB

#define I2C_SCL_RCC_CLK_ENABLE			__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C_SCL_GPIO_PIN						GPIO_PIN_6
#define I2C_SCL_GPIO_PORT						GPIOB


#define I2C_SDA_H       HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, GPIO_PIN_SET)
#define I2C_SDA_L       HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, GPIO_PIN_RESET)
#define I2C_SCL_H       HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, GPIO_PIN_SET)
#define I2C_SCL_L       HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, GPIO_PIN_RESET)


void I2C_Init(void);
void I2C_SDA_Out(void);
void I2C_SDA_In(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
unsigned char I2C_Wait_Ack(void);
void I2C_Send_Byte(unsigned char data);
unsigned char I2C_Read_Byte(unsigned char ack);

#endif

