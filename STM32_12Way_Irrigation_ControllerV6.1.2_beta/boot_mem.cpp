#include "boot_mem.h"
#include "boot_crc8.h"

/**
 @brief		: The write and read data functions required by the boot protocol.These two functions operate on the memory chip.
						Developers are required to implement hardware read-write methods.

 @param		: 1.Start address for read/write.
						2.Read/write data cache package.
						3.Length of read/write bytes.
						
 @return	: Operation returns 1 as normal, operation error returns 0.
 */
__attribute__((weak)) bt_u8 BT_MEM_Write_Bytes(bt_u8 base_addr, bt_u8 *buf, bt_u8 len) { return 1; }
__attribute__((weak)) bt_u8 BT_MEM_Read_Bytes(bt_u8 base_addr, bt_u8 *buf, bt_u8 len) { return 1; }

/**
 @brief		: Save the periodic query value for the query software version.
 @param		: Unit: seconds.
 @return	: 1:OK; 0:ERR.
 */
bt_u8 BT_Save_Query_Cycle_Value(bt_u16 cyc_val) 
{
	bt_u8 flag = 0x55;
	bt_u8 buf[2];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_QUERY_CYCLE_BASE_ADDR, &buf[0], 2);
	if ((buf[0] == (cyc_val >> 8)) && (buf[1] == (cyc_val & 0xFF)))
		return 1;
	
	buf[0] = cyc_val >> 8;
	buf[1] = cyc_val & 0xFF;
	crc1 = BT_GetCRC8(&buf[0], 2);
	
	BT_MEM_Write_Bytes(BT_QUERY_CYCLE_BASE_ADDR, &buf[0], 2);
	BT_MEM_Write_Bytes(BT_QUERY_CYCLE_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_QUERY_CYCLE_BASE_ADDR, &buf[0], 2);
	crc2 = BT_GetCRC8(&buf[0], 2);
	
	if (crc1 != crc2)	return 0;
	
	BT_MEM_Write_Bytes(BT_QUERY_CYCLE_SAVE_FLAG_ADDR, &flag, 1);
	return 1; 
}

/**
 @brief		: Gets the periodic query value for the query software version.
 @param		: Returns the value in seconds.
 @return	: 1:OK; 0:ERR.
 */
bt_u8 BT_Read_Query_Cycle_Value(bt_u16 *cyc_val) 
{
	bt_u8 flag;
	bt_u8 buf[2];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_QUERY_CYCLE_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) 
	{
		*cyc_val = 60;	//If there is no query cycle value, temporarily set it to 10 minutes.
	}
	else
	{
		BT_MEM_Read_Bytes(BT_QUERY_CYCLE_BASE_ADDR, &buf[0], 2);
		crc1 = BT_GetCRC8(&buf[0], 2);
		BT_MEM_Read_Bytes(BT_QUERY_CYCLE_CRC_ADDR, &crc2, 1);
		
		if (crc1 != crc2)
			*cyc_val = (10 * 60);	//If validation is incorrect, 10 minutes is the default.
		else
			*cyc_val = (buf[0] << 8) | buf[1];
	}
	return 1;
}

/**
 @brief		: Save the fetch firmware package cycle time.
 @param		: Unit: seconds.
 @return	: 1:OK; 0:ERR.
 */
bt_u8 BT_Save_GetDP_Cycle(bt_u16 cyc_val)
{
	bt_u8 flag = 0x55;
	bt_u8 buf[2];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_GET_DP_CYCLE_BASE_ADDR, &buf[0], 2);
	if ((buf[0] == (cyc_val >> 8)) && (buf[1] == (cyc_val & 0xFF)))
		return 1;
	
	buf[0] = cyc_val >> 8;
	buf[1] = cyc_val & 0xFF;
	crc1 = BT_GetCRC8(&buf[0], 2);
	
	BT_MEM_Write_Bytes(BT_GET_DP_CYCLE_BASE_ADDR, &buf[0], 2);
	BT_MEM_Write_Bytes(BT_GET_DP_CYCLE_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_GET_DP_CYCLE_BASE_ADDR, &buf[0], 2);
	crc2 = BT_GetCRC8(&buf[0], 2);
	
	if (crc1 != crc2)	return 0;
	
	BT_MEM_Write_Bytes(BT_GET_DP_CYCLE_SAVE_FLAG_ADDR, &flag, 1);
	return 1; 
}

/**
 @brief		: Read the fetch firmware package cycle time.
 @param		: Returns the value in seconds.
 @return	: 1:OK; 0:ERR.
 */
bt_u8 BT_Read_GetDP_Cycle(bt_u16 *cyc_val) 
{
	bt_u8 flag;
	bt_u8 buf[2];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_GET_DP_CYCLE_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) 
	{
		*cyc_val = 10;	//default 10S
	}
	else
	{
		BT_MEM_Read_Bytes(BT_GET_DP_CYCLE_BASE_ADDR, &buf[0], 2);
		crc1 = BT_GetCRC8(&buf[0], 2);
		BT_MEM_Read_Bytes(BT_GET_DP_CYCLE_CRC_ADDR, &crc2, 1);
		
		if (crc1 != crc2)
			*cyc_val = 10;	//default 10S
		else
			*cyc_val = (buf[0] << 8) | buf[1];
	}
	return 1;
}

/**
 @brief		: Save the current write to FLASH address.
 @param		: current write address.
 @return	: 1:OK; 0:Err
 */
bt_u8 BT_Save_Write_Flash_Addr(bt_u32 write_addr) 
{
	bt_u8 flag;
	bt_u8 buf[4];
	bt_u8 crc1;
	
	buf[0] = write_addr >> 24;
	buf[1] = write_addr >> 16;
	buf[2] = write_addr >> 8;
	buf[3] = write_addr & 0xFF;
	
	BT_MEM_Write_Bytes(BT_W_ADDR_BASE_ADDR, &buf[0], 4);
	
	crc1 = BT_GetCRC8(&buf[0], 4);
	
	BT_MEM_Write_Bytes(BT_W_ADDR_CRC_ADDR, &crc1, 1);
		
	BT_MEM_Read_Bytes(BT_W_ADDR_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55)
	{
		flag = 0x55;
		BT_MEM_Write_Bytes(BT_W_ADDR_SAVE_FLAG_ADDR, &flag, 1);
	}
	
	return 1;
}

/**
 @brief		: Read the current write to FLASH address.
 @param		: current write address.
 @return	: 1:OK; 0:Err
 */
bt_u8 BT_Read_Write_Flash_Addr(bt_u32 *write_addr)
{
	bt_u8 flag;
	bt_u8 buf[4];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_W_ADDR_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) 
	{
		*write_addr = BT_DP_SAVE_BASE_ADDR;	
	}
	else
	{
		BT_MEM_Read_Bytes(BT_W_ADDR_BASE_ADDR, &buf[0], 4);
		crc1 = BT_GetCRC8(&buf[0], 4);
		BT_MEM_Read_Bytes(BT_W_ADDR_CRC_ADDR, &crc2, 1);
		
		if (crc1 != crc2)
			*write_addr = BT_DP_SAVE_BASE_ADDR;	
		else
		{
			*write_addr = buf[0] << 24;
			*write_addr |= buf[1] << 16;
			*write_addr |= buf[2] << 8;
			*write_addr |= buf[3] & 0xFF;
			
			if (*write_addr < BT_DP_SAVE_BASE_ADDR)
				*write_addr = BT_DP_SAVE_BASE_ADDR;
		}
	}
	return 1;
}

/**
 @brief		: Save the current package index.
 @param		: Current package index value.
 @return	: 1:OK; 0:Err
 */
bt_u8 BT_Save_DataPack_Index(bt_u16 p_index) 
{ 
	bt_u8 flag;
	bt_u8 buf[2];
	bt_u8 crc1;
	
	BT_MEM_Read_Bytes(BT_DP_INDEX_BASE_ADDR, &buf[0], 2);
	
	if ((buf[0] == ((p_index) >> 8)) && (buf[1] == ((p_index) & 0xFF)))
		return 1;
	
	buf[0] = (p_index) >> 8;
	buf[1] = (p_index) & 0xFF;
	crc1 = BT_GetCRC8(&buf[0], 2);
	
	BT_MEM_Write_Bytes(BT_DP_INDEX_BASE_ADDR, &buf[0], 2);
	BT_MEM_Write_Bytes(BT_DP_INDEX_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_DP_INDEX_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55)
	{
		flag = 0x55;
		BT_MEM_Write_Bytes(BT_DP_INDEX_SAVE_FLAG_ADDR, &flag, 1);
	}
	return 1;
}

/**
 @brief		: Gets the current package order.
 @param		: bt_u16 *p_index.
 @return	: 1:OK; 0:Err
 */
bt_u8 BT_Read_DataPack_Index(bt_u16 *p_index) 
{ 
	bt_u8 flag;
	bt_u8 buf[2];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_DP_INDEX_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) 
	{
		*p_index = 1;	
	}
	else
	{
		BT_MEM_Read_Bytes(BT_DP_INDEX_BASE_ADDR, &buf[0], 2);
		crc1 = BT_GetCRC8(&buf[0], 2);
		BT_MEM_Read_Bytes(BT_DP_INDEX_CRC_ADDR, &crc2, 1);
		
		if (crc1 != crc2)
			*p_index = 1;	//f validation is incorrect, reset the package index.
		else
			*p_index = (buf[0] << 8) | buf[1];
	}
	return 1;
}

/**
 @brief		: Save the boot software version.
 @param		: software version buffer(2 bytes).
 @return	: 1:OK; 0:Err
 */
bt_u8 BT_Save_BT_Soft_Version(bt_u8 *v_buf)
{
	bt_u8 flag = 0x55;
	bt_u8 crc1;
	bt_u8 tem_buf[2];
	
	BT_MEM_Read_Bytes(BT_BOOT_SOFT_VER_BASE_ADDR, &tem_buf[0], 2);
	
	if ((tem_buf[0] == v_buf[0]) && (tem_buf[1] == v_buf[1])) return 1;
		
	BT_MEM_Write_Bytes(BT_BOOT_SOFT_VER_BASE_ADDR, &v_buf[0], 2);
	crc1 = BT_GetCRC8(&v_buf[0], 2);
	BT_MEM_Write_Bytes(BT_BOOT_SOFT_VER_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_BOOT_SOFT_VER_FLAG_ADDR, &flag, 1);
	if (flag != 0x55)
	{
		flag = 0x55;
		BT_MEM_Write_Bytes(BT_BOOT_SOFT_VER_FLAG_ADDR, &flag, 1);
	}
	return 1;
}

/**
 @brief		: Read the runing software version.
 @param		: runing software version buffer(2 bytes)
 @return	: 1:OK; 0:ERR
 */
bt_u8 BT_Read_BT_Soft_Version(bt_u8 *v_buf) 
{ 
	bt_u8 flag;
	bt_u8 crc1, crc2;
	bt_u8 i;
	
	BT_MEM_Read_Bytes(BT_BOOT_SOFT_VER_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) goto v_err;

	BT_MEM_Read_Bytes(BT_BOOT_SOFT_VER_BASE_ADDR, &v_buf[0], 2);
	crc1 = BT_GetCRC8(&v_buf[0], 2);
	BT_MEM_Read_Bytes(BT_BOOT_SOFT_VER_CRC_ADDR, &crc2, 1);
	
	if (crc1 != crc2)	goto v_err;
	
	return 1;
	
	v_err:
		for (i = 0; i < 2; i++)
			v_buf[i] = 0;
	if (flag != 0x55)	return 0;
	else	return 1;	
}

/**
 @brief		: Save the software version obtained from the server.
						The software version is temporary because all the data packets 
						are not currently available and the version may change on the server.
 @param		: Software version(2 bytes)
 @return	: Save successfully 1;Save failed 0.
 */
bt_u8 BT_Save_Temp_S_Version(bt_u8 *v_buf)
{
	bt_u8 flag = 0x55;
	bt_u8 crc1;
	bt_u8 tem_buf[2];
	
	BT_MEM_Read_Bytes(BT_TEMP_S_VER_BASE_ADDR, &tem_buf[0], 2);
	
	if ((tem_buf[0] == v_buf[0]) && (tem_buf[1] == v_buf[1])) 
		return 1;
		
	BT_MEM_Write_Bytes(BT_TEMP_S_VER_BASE_ADDR, &v_buf[0], 2);
	crc1 = BT_GetCRC8(&v_buf[0], 2);
	BT_MEM_Write_Bytes(BT_TEMP_S_VER_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_TEMP_S_VER_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55)
	{
		flag = 0x55;
		BT_MEM_Write_Bytes(BT_TEMP_S_VER_SAVE_FLAG_ADDR, &flag, 1);
	}
	return 1;
}

/**
 @brief		: Read the software version saved in the staging area.
 @param		: software version buffer(2 bytes)
 @return	: 1:OK; 0:ERR
 */
bt_u8 BT_Read_Temp_S_Version(bt_u8 *v_buf)
{
	bt_u8 flag;
	bt_u8 crc1, crc2;
	bt_u8 i;
	
	BT_MEM_Read_Bytes(BT_TEMP_S_VER_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) goto v_err;

	BT_MEM_Read_Bytes(BT_TEMP_S_VER_BASE_ADDR, &v_buf[0], 2);
	crc1 = BT_GetCRC8(&v_buf[0], 2);
	BT_MEM_Read_Bytes(BT_TEMP_S_VER_CRC_ADDR, &crc2, 1);
	
	if (crc1 != crc2)	goto v_err;
	
	return 1;
	
	v_err:
		for (i = 0; i < 2; i++)
			v_buf[i] = 0;
	if (flag == 0x55)	return 0;
	else	return 1;		
}

/**
 @brief		: Save the running software version.
 @param		: software version buffer(2 bytes).
 @return	: 1:OK; 0:ERR
 */
bt_u8 BT_Save_S_Version(bt_u8 *v_buf)
{
	bt_u8 flag = 0x55;
	bt_u8 crc1;
	bt_u8 tem_buf[2];
	
	BT_MEM_Read_Bytes(BT_SOFT_VER_BASE_ADDR, &tem_buf[0], 2);
	
	if ((tem_buf[0] == v_buf[0]) && (tem_buf[1] == v_buf[1])) return 1;
		
	BT_MEM_Write_Bytes(BT_SOFT_VER_BASE_ADDR, &v_buf[0], 2);
	crc1 = BT_GetCRC8(&v_buf[0], 2);
	BT_MEM_Write_Bytes(BT_SOFT_VER_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_SOFT_VER_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55)
	{
		flag = 0x55;
		BT_MEM_Write_Bytes(BT_SOFT_VER_SAVE_FLAG_ADDR, &flag, 1);
	}
	return 1;
}

/**
 @brief		: Read the runing software version.
 @param		: runing software version buffer(2 bytes)
 @return	: 1:OK; 0:ERR
 */
bt_u8 BT_Read_S_Version(bt_u8 *v_buf) 
{ 
	bt_u8 flag;
	bt_u8 crc1, crc2;
	bt_u8 i;
	
	BT_MEM_Read_Bytes(BT_SOFT_VER_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) goto v_err;

	BT_MEM_Read_Bytes(BT_SOFT_VER_BASE_ADDR, &v_buf[0], 2);
	crc1 = BT_GetCRC8(&v_buf[0], 2);
	BT_MEM_Read_Bytes(BT_SOFT_VER_CRC_ADDR, &crc2, 1);
	
	if (crc1 != crc2)	goto v_err;
	
	return 1;
	
	v_err:
		for (i = 0; i < 2; i++)
			v_buf[i] = 0;
	if (flag != 0x55)	return 0;
	else	return 1;	
}

#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
bt_u8 BT_Set_DP_Copy_Flag(void)
{
	bt_u8 temp = 0;
	
	BT_MEM_Read_Bytes(BT_DP_CP_JUMP_ADDR, &temp, 1);
	if (temp == 1) return 1;
		
	temp = 1;
	BT_MEM_Write_Bytes(BT_DP_CP_JUMP_ADDR, &temp, 1);

	return 1;
}

bt_u8 BT_Clear_DP_Copy_Flag(void)
{
	bt_u8 temp = 1;
	
	BT_MEM_Read_Bytes(BT_DP_CP_JUMP_ADDR, &temp, 1);
	if (temp == 0) return 1;
		
	temp = 0;
	BT_MEM_Write_Bytes(BT_DP_CP_JUMP_ADDR, &temp, 1);

	return 1;	
}

bt_u8 BT_Read_DP_Copy_Flag(void) 
{ 
	bt_u8 flag;
	BT_MEM_Read_Bytes(BT_DP_CP_JUMP_ADDR, &flag, 1);
	if (flag != 0 && flag != 1) return 0;
	return flag;
}

#else
bt_u8 BT_Set_Cover_Update_Flag(void)
{
	bt_u8 temp = 0;
	
	BT_MEM_Read_Bytes(BT_COVER_UD_FLAG_ADDR, &temp, 1);
	if (temp == 1) return 1;
		
	temp = 1;
	BT_MEM_Write_Bytes(BT_COVER_UD_FLAG_ADDR, &temp, 1);

	return 1;
}

bt_u8 BT_Clear_Cover_Update_Flag(void)
{
	bt_u8 temp = 1;
	
	BT_MEM_Read_Bytes(BT_COVER_UD_FLAG_ADDR, &temp, 1);
	if (temp == 0) return 1;
		
	temp = 0;
	BT_MEM_Write_Bytes(BT_COVER_UD_FLAG_ADDR, &temp, 1);

	return 1;	
}

bt_u8 BT_Read_Cover_Update_Flag(void) 
{ 
	bt_u8 flag;
	BT_MEM_Read_Bytes(BT_COVER_UD_FLAG_ADDR, &flag, 1);
	if (flag != 0 && flag != 1) return 0;
	return flag;
}

bt_u8 BT_Set_Update_OK_Flag(void)
{
	bt_u8 temp = 0;
	
	BT_MEM_Read_Bytes(BT_UD_OK_BASE_ADDR, &temp, 1);
	if (temp == 1) return 1;

	temp = 1;
	BT_MEM_Write_Bytes(BT_UD_OK_BASE_ADDR, &temp, 1);

	return 1;
}

bt_u8 BT_Clear_Update_OK_Flag(void)
{
	bt_u8 temp = 1;
	
	BT_MEM_Read_Bytes(BT_UD_OK_BASE_ADDR, &temp, 1);
	if (temp == 0) return 1;
		
	temp = 0;
	BT_MEM_Write_Bytes(BT_UD_OK_BASE_ADDR, &temp, 1);

	return 1;	
}

bt_u8 BT_Read_Update_OK_Flag(void) 
{ 
	bt_u8 flag;
	BT_MEM_Read_Bytes(BT_UD_OK_BASE_ADDR, &flag, 1);
	return flag;
}
#endif

#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
void BT_Save_SaveArea_DP_Size(bt_u32 dp_size) 
{
	bt_u8 flag;
	bt_u8 buf[4];
	bt_u8 crc1;
	
	buf[0] = dp_size >> 24;
	buf[1] = dp_size >> 16;
	buf[2] = dp_size >> 8;
	buf[3] = dp_size & 0xFF;
	
	BT_MEM_Write_Bytes(BT_DP_SIZE_BASE_ADDR, &buf[0], 4);
	
	crc1 = BT_GetCRC8(&buf[0], 4);
	
	BT_MEM_Write_Bytes(BT_DP_SIZE_CRC_ADDR, &crc1, 1);
	
	BT_MEM_Read_Bytes(BT_DP_SIZE_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55)
	{
		flag = 0x55;
		BT_MEM_Write_Bytes(BT_DP_SIZE_SAVE_FLAG_ADDR, &flag, 1);
	}
}

void BT_Read_SaveArea_DP_Size(bt_u32 *dp_size)
{
	bt_u8 flag;
	bt_u8 buf[4];
	bt_u8 crc1, crc2;
	
	BT_MEM_Read_Bytes(BT_DP_SIZE_SAVE_FLAG_ADDR, &flag, 1);
	if (flag != 0x55) 
	{
		*dp_size = 0;	
	}
	else
	{
		BT_MEM_Read_Bytes(BT_DP_SIZE_BASE_ADDR, &buf[0], 4);
		crc1 = BT_GetCRC8(&buf[0], 4);
		BT_MEM_Read_Bytes(BT_DP_SIZE_CRC_ADDR, &crc2, 1);
		
		if (crc1 != crc2)
			*dp_size = 0;	
		else
		{
			*dp_size = buf[0] << 24;
			*dp_size |= buf[1] << 16;
			*dp_size |= buf[2] << 8;
			*dp_size |= buf[3] & 0xFF;
		}
	}
}

#endif

bt_u8 BT_Clear_All_BOOT_Mem(void)
{
	BT_MEM_Write_Bytes(BT_STORE_OFFSET_ADDR, 0x00, BT_STORE_SIZE);
	return 1;
}
