#ifndef _BOOT_MEM_H_
#define _BOOT_MEM_H_

#include "boot_config.h"

#define BT_STORE_OFFSET_ADDR						        220
#define BT_STORE_SIZE										35 //bytes

/* Periodically query the version store */
#define BT_QUERY_CYCLE_BASE_ADDR				(BT_STORE_OFFSET_ADDR + 1)
#define BT_QUERY_CYCLE_END_ADDR					(BT_STORE_OFFSET_ADDR + 2)
#define BT_QUERY_CYCLE_CRC_ADDR					(BT_STORE_OFFSET_ADDR + 3)
#define BT_QUERY_CYCLE_SAVE_FLAG_ADDR 	(BT_STORE_OFFSET_ADDR + 4)

/* Firmware package index storage area */
#define BT_DP_INDEX_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 5)
#define BT_DP_INDEX_END_ADDR						(BT_STORE_OFFSET_ADDR + 6)
#define BT_DP_INDEX_CRC_ADDR						(BT_STORE_OFFSET_ADDR + 7)
#define BT_DP_INDEX_SAVE_FLAG_ADDR			(BT_STORE_OFFSET_ADDR + 8)

/* Current firmware download address storage area */
#define BT_W_ADDR_BASE_ADDR							(BT_STORE_OFFSET_ADDR + 9)
#define BT_W_ADDR_END_ADDR							(BT_STORE_OFFSET_ADDR + 12)
#define BT_W_ADDR_CRC_ADDR							(BT_STORE_OFFSET_ADDR + 13)
#define BT_W_ADDR_SAVE_FLAG_ADDR				(BT_STORE_OFFSET_ADDR + 14)

/* Run the software version store */
#define BT_SOFT_VER_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 15)
#define BT_SOFT_VER_END_ADDR						(BT_STORE_OFFSET_ADDR + 16)
#define BT_SOFT_VER_CRC_ADDR						(BT_STORE_OFFSET_ADDR + 19)
#define BT_SOFT_VER_SAVE_FLAG_ADDR			(BT_STORE_OFFSET_ADDR + 20)

/* Temporary software version storage area */
#define BT_TEMP_S_VER_BASE_ADDR					(BT_STORE_OFFSET_ADDR + 21)
#define BT_TEMP_S_VER_END_ADDR					(BT_STORE_OFFSET_ADDR + 22)
#define BT_TEMP_S_VER_CRC_ADDR					(BT_STORE_OFFSET_ADDR + 23)
#define BT_TEMP_S_VER_SAVE_FLAG_ADDR		(BT_STORE_OFFSET_ADDR + 24)

/* Firmware package copy jump flag storage */
#define BT_DP_CP_JUMP_ADDR							(BT_STORE_OFFSET_ADDR + 25)

/* Total size storage area for firmware packages */
#define BT_DP_SIZE_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 26)
#define BT_DP_SIZE_END_ADDR							(BT_STORE_OFFSET_ADDR + 29)
#define BT_DP_SIZE_CRC_ADDR							(BT_STORE_OFFSET_ADDR + 30)
#define BT_DP_SIZE_SAVE_FLAG_ADDR				(BT_STORE_OFFSET_ADDR + 31)

/* The firmware package retrieves the periodic storage area */
#define BT_GET_DP_CYCLE_BASE_ADDR               (BT_STORE_OFFSET_ADDR + 32)
#define BT_GET_DP_CYCLE_END_ADDR                (BT_STORE_OFFSET_ADDR + 33)
#define BT_GET_DP_CYCLE_CRC_ADDR                (BT_STORE_OFFSET_ADDR + 34)
#define BT_GET_DP_CYCLE_SAVE_FLAG_ADDR          (BT_STORE_OFFSET_ADDR + 35)

bt_u8 BT_Save_Query_Cycle_Value(bt_u16 cyc_val);
bt_u8 BT_Read_Query_Cycle_Value(bt_u16 *cyc_val);

bt_u8 BT_Save_DataPack_Index(bt_u16 p_index);
bt_u8 BT_Read_DataPack_Index(bt_u16 *p_index);

bt_u8 BT_Save_Temp_S_Version(bt_u8 *v_buf);
bt_u8 BT_Read_Temp_S_Version(bt_u8 *v_buf);

bt_u8 BT_Save_S_Version(bt_u8 *v_buf);
bt_u8 BT_Read_S_Version(bt_u8 *v_buf);

void BT_Save_Write_Flash_Addr(bt_u32 write_addr);
void BT_Read_Write_Flash_Addr(bt_u32 *write_addr);

bt_u8 BT_Set_DP_Copy_Flag(void);
bt_u8 BT_Clear_DP_Copy_Flag(void);
bt_u8 BT_Read_DP_Copy_Flag(void);

void BT_Save_SaveArea_DP_Size(bt_u32 dp_size);
void BT_Read_SaveArea_DP_Size(bt_u32 *dp_size);

bt_u8 BT_Save_GetDP_Cycle(bt_u16 cyc_val);
bt_u8 BT_Read_GetDP_Cycle(bt_u16 *cyc_val);

#endif
