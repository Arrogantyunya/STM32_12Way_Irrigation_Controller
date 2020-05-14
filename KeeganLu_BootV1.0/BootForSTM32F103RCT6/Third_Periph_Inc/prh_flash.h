#ifndef _PRH_FLASH_

#include "../bt_inc/boot_config.h"

void Read_Flash(unsigned int read_addr, unsigned short *pdata_buffer, unsigned int data_len);
void Write_Flash_Buffer(unsigned int write_addr, unsigned short *pdata_buf, unsigned int data_len);

#endif
