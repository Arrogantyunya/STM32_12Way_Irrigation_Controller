#include "../Third_Periph_Inc/app_mem.h"
#include "../Third_Periph_Inc/at24Cxx.h"
#include "../Third_Periph_Inc/crc8.h"
#include "../Third_Periph_Inc/debug_uart.h"


uint8_t BT_MEM_Read_Bytes(uint8_t base_addr, uint8_t *buf, uint8_t len)
{
	uint32_t i;
	for (i = 0; i < len; i++)
		buf[i] = AT24Cxx_ReadOneByte(base_addr++);
		
	return 1;
}

uint8_t BT_MEM_Write_Bytes(uint8_t base_addr, uint8_t *buf, uint8_t len)
{
	uint32_t i;
	EEPROM_WP_En();
	for (i = 0; i < len; i++)
		AT24Cxx_WriteOneByte(base_addr++, buf[i]);
	EEPROM_WP_Dis();

	return 1;
}

/**
 @brief		: 比较要写入的数据和上一次写入的数据是否相同，
						如果相同，原则上来说就不需要再次写入。
						
 @param		: 1.要写入的缓存。
						2.要写入的起始地址。
						3.要写入的数据的长度。
						
 @return	: 相同返回1，不相同返回0
 */
uint8_t app_mem_check_idtc_content(uint8_t *new_buf, uint8_t saved_addr, uint8_t fctrl_num)
{
	uint8_t i;
	uint8_t tmp_buf[fctrl_num];
	
	for (i = 0; i < fctrl_num; i++)
	{
		tmp_buf[i] = AT24Cxx_ReadOneByte(saved_addr + i);
		if (new_buf[i] != tmp_buf[i])
			return 0;
	}
	return 1;
}

uint8_t app_mem_check_crc(uint8_t new_crc, uint8_t saved_addr, uint8_t fctrl_num)
{
	uint8_t tmp_buf[fctrl_num];
	uint8_t i;
	
	for (i = 0; i < fctrl_num; i++)
		tmp_buf[i] = AT24Cxx_ReadOneByte(saved_addr + i);
		
	if (new_crc == GetCRC8(&tmp_buf[0], fctrl_num))
		return 1;
	
  
	EEPROM_WP_Dis();
	return 0;
}

uint8_t app_mem_Is_Saved_Check(uint8_t flag_addr)
{
	return AT24Cxx_ReadOneByte(flag_addr);
}

uint8_t app_mem_Save_SN_Code(uint8_t *sn_buf)
{
	uint8_t sn_crc_val;
	uint8_t saved_flag = 0x55;
	uint8_t i;
	
	if (app_mem_check_idtc_content(sn_buf, SN_BASE_ADDR, 9)) return 1;
	
  EEPROM_WP_En();
  for (i = 0; i < 9; i++)
		AT24Cxx_WriteOneByte(SN_BASE_ADDR + i, sn_buf[i]);

	sn_crc_val = GetCRC8(&sn_buf[0], 9); //得到SN码的CRC8校验
	AT24Cxx_WriteOneByte(SN_CRC_ADDR, sn_crc_val);
	
	if (!app_mem_check_crc(sn_crc_val, SN_BASE_ADDR, 9))
	{
		EEPROM_WP_Dis();
		return 0;
	}
	if (app_mem_check_idtc_content(&saved_flag, SN_SAVE_FLAG_ADDR, 1))
	{
		EEPROM_WP_Dis();
		return 1;
	}
	
	AT24Cxx_WriteOneByte(SN_SAVE_FLAG_ADDR, saved_flag);
	return 1;
}

uint8_t app_mem_Read_SN_Code(uint8_t *sn_buf)
{
	uint8_t crc_val;
	uint8_t i;
	
	crc_val = AT24Cxx_ReadOneByte(SN_CRC_ADDR);
	if (!app_mem_check_crc(crc_val, SN_BASE_ADDR, 9))
		return 0;
	
	for (i = 0; i < 9; i++)
		sn_buf[i] = AT24Cxx_ReadOneByte(SN_BASE_ADDR + i);

	return 1;
}

uint8_t app_mem_Save_Region_Code(uint8_t r_code)
{
	uint8_t crc_val;
	uint8_t saved_flag = 0x55;
	
	if (app_mem_check_idtc_content(&r_code, REGION_ADDR, 1)) return 1;
	
  EEPROM_WP_En();
	AT24Cxx_WriteOneByte(REGION_ADDR, r_code);

	crc_val = GetCRC8(&r_code, 1);
	AT24Cxx_WriteOneByte(REGION_CRC_ADDR, crc_val);
	
	if (!app_mem_check_crc(crc_val, REGION_ADDR, 1))
	{
		EEPROM_WP_Dis();
		return 0;
	}
	if (app_mem_check_idtc_content(&saved_flag, REGION_SAVE_FLAG_ADDR, 1))
	{
		EEPROM_WP_Dis();
		return 1;
	}
	
	AT24Cxx_WriteOneByte(REGION_SAVE_FLAG_ADDR, saved_flag);
	return 1;
}

uint8_t app_mem_Read_Region_Code(uint8_t *r_code)
{
	uint8_t crc_val;
	
	crc_val = AT24Cxx_ReadOneByte(REGION_CRC_ADDR);
	if (!app_mem_check_crc(crc_val, REGION_ADDR, 1))
		return 0;
	
	*r_code = AT24Cxx_ReadOneByte(REGION_ADDR);
	return 1;
}

uint8_t app_mem_Save_Group_Code(uint8_t *group_buf)
{
	uint8_t crc_val;
	uint8_t saved_flag = 0x55;
	uint8_t i;
	
	if (app_mem_check_idtc_content(group_buf, GROUP_NUM_BASE_ADDR, 5)) return 1;
	
  EEPROM_WP_En();
  for (i = 0; i < 5; i++)
		AT24Cxx_WriteOneByte(GROUP_NUM_BASE_ADDR + i, group_buf[i]);

	crc_val = GetCRC8(&group_buf[0], 5);
	AT24Cxx_WriteOneByte(GROUP_NUM_CRC_ADDR, crc_val);
	
	if (!app_mem_check_crc(crc_val, GROUP_NUM_BASE_ADDR, 5))
	{
		EEPROM_WP_Dis();
		return 0;
	}
	if (app_mem_check_idtc_content(&saved_flag, GROUP_NUM_SAVE_FLAG_ADDR, 1))
	{
		EEPROM_WP_Dis();
		return 1;
	}
	
	AT24Cxx_WriteOneByte(GROUP_NUM_SAVE_FLAG_ADDR, saved_flag);
	return 1;
}

uint8_t app_mem_Read_Group_Code(uint8_t *group_buf)
{
	uint8_t crc_val;
	uint8_t i;
	
	crc_val = AT24Cxx_ReadOneByte(GROUP_NUM_CRC_ADDR);
	if (!app_mem_check_crc(crc_val, GROUP_NUM_BASE_ADDR, 5))
		return 0;
	
	for (i = 0; i < 5; i++)
		group_buf[i] = AT24Cxx_ReadOneByte(GROUP_NUM_BASE_ADDR + i);
	return 1;	
}

