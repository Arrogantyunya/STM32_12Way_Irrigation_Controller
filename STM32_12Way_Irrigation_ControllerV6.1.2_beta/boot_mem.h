#ifndef _BOOT_MEM_H_
#define _BOOT_MEM_H_

#include "boot_config.h"

#define BT_STORE_OFFSET_ADDR						1000

#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
#define BT_STORE_SIZE										(40 + 1) //bytes
#else
#define BT_STORE_SIZE										(42 + 1) //bytes
#endif

/* Periodically query the version store */
#define BT_QUERY_CYCLE_BASE_ADDR				(BT_STORE_OFFSET_ADDR + 0)
#define BT_QUERY_CYCLE_END_ADDR					(BT_STORE_OFFSET_ADDR + 1)
#define BT_QUERY_CYCLE_CRC_ADDR					(BT_STORE_OFFSET_ADDR + 2)
#define BT_QUERY_CYCLE_SAVE_FLAG_ADDR 	(BT_STORE_OFFSET_ADDR + 3)
/* The firmware package retrieves the periodic storage area */
#define BT_GET_DP_CYCLE_BASE_ADDR       (BT_STORE_OFFSET_ADDR + 4)
#define BT_GET_DP_CYCLE_END_ADDR        (BT_STORE_OFFSET_ADDR + 5)
#define BT_GET_DP_CYCLE_CRC_ADDR        (BT_STORE_OFFSET_ADDR + 6)
#define BT_GET_DP_CYCLE_SAVE_FLAG_ADDR  (BT_STORE_OFFSET_ADDR + 7)
/* Firmware package index storage area */
#define BT_DP_INDEX_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 8)
#define BT_DP_INDEX_END_ADDR						(BT_STORE_OFFSET_ADDR + 9)
#define BT_DP_INDEX_CRC_ADDR						(BT_STORE_OFFSET_ADDR + 10)
#define BT_DP_INDEX_SAVE_FLAG_ADDR			(BT_STORE_OFFSET_ADDR + 11)
/* Current firmware download address storage area */
#define BT_W_ADDR_BASE_ADDR							(BT_STORE_OFFSET_ADDR + 12)
#define BT_W_ADDR_END_ADDR							(BT_STORE_OFFSET_ADDR + 15)
#define BT_W_ADDR_CRC_ADDR							(BT_STORE_OFFSET_ADDR + 16)
#define BT_W_ADDR_SAVE_FLAG_ADDR				(BT_STORE_OFFSET_ADDR + 17)
/* The software version number of the boot */
#define BT_BOOT_SOFT_VER_BASE_ADDR			(BT_STORE_OFFSET_ADDR + 18)
#define BT_BOOT_SOFT_VER_END_ADDR				(BT_STORE_OFFSET_ADDR + 19)
#define BT_BOOT_SOFT_VER_CRC_ADDR				(BT_STORE_OFFSET_ADDR + 20)
#define BT_BOOT_SOFT_VER_FLAG_ADDR			(BT_STORE_OFFSET_ADDR + 21)
/* Run the software version store */
#define BT_SOFT_VER_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 22)
#define BT_SOFT_VER_END_ADDR						(BT_STORE_OFFSET_ADDR + 23)
#define BT_SOFT_VER_CRC_ADDR						(BT_STORE_OFFSET_ADDR + 24)
#define BT_SOFT_VER_SAVE_FLAG_ADDR			(BT_STORE_OFFSET_ADDR + 25)
/* The hardware version number of the device */
#define BT_HARD_VER_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 26)
#define BT_HARD_VER_END_ADDR						(BT_STORE_OFFSET_ADDR + 27)
#define BT_HARD_VER_CRC_ADDR						(BT_STORE_OFFSET_ADDR + 28)
#define BT_HARD_VER_SAVE_FLAG_ADDR			(BT_STORE_OFFSET_ADDR + 29)
/* Temporary software version storage area */
#define BT_TEMP_S_VER_BASE_ADDR					(BT_STORE_OFFSET_ADDR + 30)
#define BT_TEMP_S_VER_END_ADDR					(BT_STORE_OFFSET_ADDR + 31)
#define BT_TEMP_S_VER_CRC_ADDR					(BT_STORE_OFFSET_ADDR + 32)
#define BT_TEMP_S_VER_SAVE_FLAG_ADDR		(BT_STORE_OFFSET_ADDR + 33)
/* Firmware package copy jump flag storage */
#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
#define BT_DP_CP_JUMP_ADDR							(BT_STORE_OFFSET_ADDR + 34)
/* Total size storage area for firmware packages */
#define BT_DP_SIZE_BASE_ADDR						(BT_STORE_OFFSET_ADDR + 35)
#define BT_DP_SIZE_END_ADDR							(BT_STORE_OFFSET_ADDR + 38)
#define BT_DP_SIZE_CRC_ADDR							(BT_STORE_OFFSET_ADDR + 39)
#define BT_DP_SIZE_SAVE_FLAG_ADDR				(BT_STORE_OFFSET_ADDR + 40)
#else
#define BT_COVER_UD_FLAG_ADDR						(BT_STORE_OFFSET_ADDR + 41)
#define BT_UD_OK_BASE_ADDR							(BT_STORE_OFFSET_ADDR + 42)
#endif

bt_u8 BT_Save_Query_Cycle_Value(bt_u16 cyc_val);
bt_u8 BT_Read_Query_Cycle_Value(bt_u16 *cyc_val);

bt_u8 BT_Save_GetDP_Cycle(bt_u16 cyc_val);
bt_u8 BT_Read_GetDP_Cycle(bt_u16 *cyc_val);

bt_u8 BT_Save_Write_Flash_Addr(bt_u32 write_addr);
bt_u8 BT_Read_Write_Flash_Addr(bt_u32 *write_addr);

bt_u8 BT_Save_DataPack_Index(bt_u16 p_index);
bt_u8 BT_Read_DataPack_Index(bt_u16 *p_index);

bt_u8 BT_Save_BT_Soft_Version(bt_u8 *v_buf);
bt_u8 BT_Read_BT_Soft_Version(bt_u8 *v_buf);

bt_u8 BT_Save_Temp_S_Version(bt_u8 *v_buf);
bt_u8 BT_Read_Temp_S_Version(bt_u8 *v_buf);

bt_u8 BT_Save_S_Version(bt_u8 *v_buf);
bt_u8 BT_Read_S_Version(bt_u8 *v_buf);

#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
bt_u8 BT_Set_DP_Copy_Flag(void);
bt_u8 BT_Clear_DP_Copy_Flag(void);
bt_u8 BT_Read_DP_Copy_Flag(void);
void BT_Save_SaveArea_DP_Size(bt_u32 dp_size);
void BT_Read_SaveArea_DP_Size(bt_u32 *dp_size);
#else
bt_u8 BT_Set_Cover_Update_Flag(void);
bt_u8 BT_Clear_Cover_Update_Flag(void);
bt_u8 BT_Read_Cover_Update_Flag(void);

bt_u8 BT_Set_Update_OK_Flag(void);
bt_u8 BT_Clear_Update_OK_Flag(void);
bt_u8 BT_Read_Update_OK_Flag(void);
#endif

bt_u8 BT_Clear_All_BOOT_Mem(void);

#endif
