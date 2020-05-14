#ifndef _AT24CXX_H
#define _AT24CXX_H

#include "../bt_inc/boot_config.h"
#include "../Third_Periph_Inc/i2c.h"


/*EEPROM chip*/
#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767

/*选择的设备为AT24C02*/
#define EE_TYPE     AT24C02

#define USE_EP_WP			1	//使用EEPROM芯片的写保护引脚

#if USE_EP_WP
#define EP_WP_RCC_CLK_ENABLE				__HAL_RCC_GPIOB_CLK_ENABLE()
#define EP_WP_GPIO_PIN							GPIO_PIN_5
#define EP_WP_GPIO_PORT							GPIOB

void EEPROM_WP_Init(void);
void EEPROM_WP_En(void);
void EEPROM_WP_Dis(void);
#endif

unsigned char AT24Cxx_ReadOneByte(unsigned int addr);
void AT24Cxx_WriteOneByte(unsigned int addr, unsigned char dt);

#endif
