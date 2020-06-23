
#include "boot_protocol.h"
#include "boot_mem.h"
#include "boot_crc16.h"
#include "boot_crc8.h"
#include <stdlib.h>

/**
 *Function declaration
 */
enum BT_R_RESULT BT_DataPack_Handler(bt_u8 *d_buf);

#if USE_TIM_RCV_MSG
bt_u8 BT_Timer_Handler(bt_u16 *rcv_len, bt_u16 *tim_num);
#else
bt_u8 BT_Receive_Handler(bt_u16 *rcv_len, bt_u8 *rcvd_flag);
#endif

#if (BT_IS_BOOT_ZONE && (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
bt_u8 BT_Copy_DataPack(bt_u32 dp_size);
#endif

bt_u8 BT_HighByte(bt_u16 var);
bt_u8 BT_LowByte(bt_u16 var);
bt_u8 BT_Hex_To_Dec(bt_u8 dat);
bt_u8 BT_Dec_To_Hex(bt_u8 dat);

struct BT_pcb bt_pcb;

char BT_PrintDBG_Buf[PRINT_DBG_LEN];	//Print debugging.


bt_u16 BT_Rcv_Tim_Num = 0;	//Receive data, timeout processing count variables
bt_u32 BT_Query_CycVal = 0; //Periodic query count variables
bt_u32 BT_Rand_Seed_Num;	//Random number seed count variable

/**
 * @Usage:
 * These functions with the label of __attribute__((weak) are to be implemented 
 * by the secondary developer according to the description of each function.
 * Remote upgrade protocol calls these functions back and forth.
 * These functions print out "[XXXXX]function is not implemented!" by default., 
 * which means the secondary developer did not implement the XXXXX function. 
 */

/**
 @brief		: It is up to the developer to fill in the reserved fields required by the application.
 @param		: The developer fills in the re_buf array cache.
 @return	: None
 */
__attribute__((weak)) void BT_Fill_ReservedByte(bt_u8 *re_buf) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Fill_ReservedByte] function is not implemented!\n"));
}

/**
 @brief		: When the server instruction is received, the developer determines the method of checking the reserved field.
 
 @param		: Protocol type of server instruction.
 
 @return	: Correct checks return BT_R_OK and error returns BT_R_RE_CHECK_ERR.
						refer to BT_R_RESULT.
 */
__attribute__((weak)) bt_u8 BT_ReservedByte_Check(bt_u16 bt_type, bt_u8 *re_buf) 
{ 
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_ReservedByte_Check] function is not implemented!\n"));
	return BT_R_OK; 
}

/**
 @brief		: Writes the firmware package to FLASH.
						The developer implements the function.
 @param		: 1.The starting address of the write.
						2.Write to the data cache (the developer needs to determine the type of BT_DATAPACK_TYPE and do the appropriate write).
						3.Length of data to write.
 @return	: Write correctly returns 1, error returns 0.
 */
__attribute__((weak)) bt_u8 BT_Write_DP_To_Flash(bt_u32 write_addr, BT_DATAPACK_TYPE *app_buffer, bt_u32 app_len) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Write_DP_To_Flash] function is not implemented!\n"));
	return 0;
}

/**
 @brief		: Read the firmware package to FLASH.
						The developer implements the function.
 @param		: 1.The starting address of the read.
						2.read to the data cache (the developer needs to determine the type of BT_DATAPACK_TYPE and do the appropriate read).
						3.Length of data to read.
 @return	: Write correctly returns 1, error returns 0.
 */
__attribute__((weak)) bt_u8 BT_Read_DP_From_Flash(bt_u32 write_addr, BT_DATAPACK_TYPE *app_buffer, bt_u32 app_len)
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Read_DP_From_Flash] function is not implemented!\n"));
	return 0;	
}

/**
 @brief		: Before jumping to the newly downloaded application, 
						it is up to the developer to implement a check to see if the downloaded application is correct
 @param		: Check address.
 @return	: Detection returns 1 on success and 0 on failure.
 */ 
__attribute__((weak)) bt_u8 BT_APP_Check(bt_u32 app_addr) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_APP_Check] function is not implemented!\n"));
	return 0;
}

/**
 @brief		: Stop all interrupts that have been started.
						The developer implements the function.
 @param		: None
 @return	: None
 */
__attribute__((weak)) void BT_Stop_Interrupt(void) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Stop_Interrupt] function is not implemented!\n"));
}

/**
 @brief		: Start all interrupts that have been stoped.
						The developer implements the function.
 @param		: None
 @return	: None
 */
__attribute__((weak)) void BT_Start_Interrupt(void)
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Start_Interrupt] function is not implemented!\n"));
}

/**
 @brief		: Jump to the application.
						The developer implements the function.
 @param		: Jump to the FLASH address.
 @return	: None
 */
__attribute__((weak)) void BT_Jump_APP_Handler(bt_u32 jump_addr) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Jump_APP_Handler] function is not implemented!\n"));
}

/**
 @brief		: Jump to the boot zone.
						The developer implements the function.
 @param		: Jump to the boot zone address.
 @return	: None
 */
__attribute__((weak)) void BT_Jump_BOOT(bt_u32 boot_addr) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Jump_BOOT] function is not implemented!\n"));
}

/**
 @brief		: The boot exception is triggered.
						Means there is a problem in the application area, disable the jump, 
						stay in the boot area, clear all tags, and wait for the firmware update.
 @param		: None
 @return	: 1 : trigger; 0 : No trigger.
 */
__attribute__((weak)) bt_u8 BT_Exception_Trigger(void) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Exception_Trigger] function is not implemented!\n"));
	return 0;
}

/**
 @brief		: The boot exception is triggered.
						Means there is a problem in the application area, disable the jump, 
						stay in the boot area, clear all tags, and wait for the firmware update.
 @param		: None
 @return	: 1 : trigger; 0 : No trigger.
 */
__attribute__((weak)) bt_u8 BT_Entry_Program_Trigger(void) 
{
	BT_DEBUGF(BT_CB_DBG, (BT_PrintDBG_Buf, "warning: [BT_Entry_Program_Trigger] function is not implemented!\n"));
	return 0;	
}

/**
 @brief		: Set/read the status of the remote upgrade protocol running process.
						Read the running status function, can be applied in the LED or buzzer status prompt, self-development.
 @param		: refer to enum BT_RUN_STA
 @return	: refer to enum BT_RUN_STA
 */
bt_u8 BT_Get_RunStatus(void){return bt_pcb.run_sta_process;}
void BT_Set_RunStatus(bt_u8 run_sta){bt_pcb.run_sta_process = run_sta;}

/**
 @brief		: Mount the communication module send function.
						At the same time, some variables of the protocol control module are initialized.
						Communication module send function needs the developer to implement.
 @param		: The communication module sends function Pointers.
						parameter: (bt_u8*, bt_u16)
						The first is the sent data cache passed in by the protocol layer, and the second is the data cache length��
 @return	: None
 */
void BT_Mount_Dev_And_Init(bt_s_h_func s_h_func)
{
	BT_Set_RunStatus(IDLE);

	bt_pcb.s_h_func = s_h_func;
	
	/* Get the reserved bit information, and later fill the package to send */
	BT_Fill_ReservedByte(&bt_pcb.re_buf[0]);
	
	bt_pcb.dp_buf_offset = 0;
	bt_pcb.d_pack_end_flag = 0;
	bt_pcb.cyc_que_sw_start_flag = 0;
	bt_pcb.d_pack_en_get_flag = 0;
	bt_pcb.no_app_flag = 0;
	/* Fill in the hardware version of the device */
	bt_pcb.hw_v_buf[0] = DEVICE_HARD_VERSION_H;
	bt_pcb.hw_v_buf[1] = DEVICE_HARD_VERSION_L;
	/* Fill in the software version of the boot */
	bt_pcb.bt_sw_v_buf[0] = BOOT_SOFT_VERSION_H;
	bt_pcb.bt_sw_v_buf[1] = BOOT_SOFT_VERSION_L;
	BT_Save_BT_Soft_Version(&bt_pcb.bt_sw_v_buf[0]);
	BT_Read_BT_Soft_Version(&bt_pcb.bt_sw_v_buf[0]);
	/* Get the version query cycle and the firmware package fetch cycle */
	BT_Read_Query_Cycle_Value(&bt_pcb.que_v_cyc_val);
	BT_Read_GetDP_Cycle(&bt_pcb.dp_get_cyc_val);
	/* Read the current write flash address and the firmware pacage index*/
	BT_Read_Write_Flash_Addr(&bt_pcb.dp_w_addr);
	BT_Read_DataPack_Index(&bt_pcb.d_pack_index);
	
	#if (!BT_IS_BOOT_ZONE && !(BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
	BT_Set_Update_OK_Flag();
	#endif
	
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "BOOT hardware version : %d-%d\n", bt_pcb.hw_v_buf[0], bt_pcb.hw_v_buf[1]));
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "BOOT software version : %d-%d\n", bt_pcb.bt_sw_v_buf[0], bt_pcb.bt_sw_v_buf[1]));
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Currently write FLASH address : 0x%X\n", bt_pcb.dp_w_addr));
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Current package sequence : %d\n", bt_pcb.d_pack_index));
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "software version query cycle value : %d s\n", bt_pcb.que_v_cyc_val));
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Firmware get cycle value : %d s\n", bt_pcb.dp_get_cyc_val));
	
	/* After initialization, a trigger exception is detected and entry into the application is disabled */
	if (BT_Exception_Trigger())	
	{
		BT_Clear_All_BOOT_Mem();
		BT_Set_RunStatus(APP_EXP);
		return;
	}
	
	if (BT_Entry_Program_Trigger()) 
	{
		#if (BT_IS_BOOT_ZONE && !(BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
		BT_Set_Update_OK_Flag();
		#endif
	}
}


#if BT_IS_BOOT_ZONE
/**
 @brief		: This function can only be used in bootloader programs. It is not allowed in applications!
						This function handles detecting and loading applications.
 @param		: None
 @return	: None
 */
void BT_APP_Init(void)
{
	bt_u8 s_v_buf[2] = {0, 0};
	#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
	bt_u32 dp_size = 0;
	#endif

	if (BT_Get_RunStatus() == APP_EXP) goto reset_dp;
	
	#if (!(BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
	if (BT_Read_Cover_Update_Flag())
	{
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "[YQEU OS] Prepare to wait for update firmware... (Cover Update)\n"));
		return;
	}
	#endif
	
	if (!BT_APP_Check(BT_APP_RUN_ADDR))	//If the program run area check failed.
	{ 
		#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
		if (BT_Read_DP_Copy_Flag())	//If there is a copy request flag(update app)
		{
			if (!BT_APP_Check(BT_DP_SAVE_BASE_ADDR)) goto reset_dp;
			//If the program storage area check success.
			BT_Set_RunStatus(CP_DP);
			BT_Read_SaveArea_DP_Size(&dp_size);
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "(Run APP excp) New Program storage area size : %d bytes\n", dp_size));
			BT_Copy_DataPack(dp_size);
			BT_Clear_DP_Copy_Flag();//Clear the copy request flag.
			if (!BT_APP_Check(BT_APP_RUN_ADDR)) goto reset_dp;
			goto pre_jump;
		}
		/* No application under the breakpoint continuation handler */
		if (BT_APP_Check(BT_DP_SAVE_BASE_ADDR))
		{
			bt_pcb.no_app_flag = 1;
			return;
		}
		else 
			goto reset_dp;
			
		#else
			goto reset_dp;
		#endif
	}
	else
	{
		#if (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR)
		if (BT_Read_DP_Copy_Flag())	//If there is a copy request flag(update app)
		{
			BT_Clear_DP_Copy_Flag();//Clear the copy request flag.
			
			if (!BT_APP_Check(BT_DP_SAVE_BASE_ADDR)) goto reset_dp;	/* !!!!!!!!!!!!!! */
			//If the program storage area check success.
			BT_Set_RunStatus(CP_DP);
			BT_Read_SaveArea_DP_Size(&dp_size);
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "(Run APP OK) New Program storage area size : %d bytes\n", dp_size));
			BT_Copy_DataPack(dp_size);
			if (!BT_APP_Check(BT_APP_RUN_ADDR)) goto reset_dp;
			BT_Read_Temp_S_Version(&s_v_buf[0]);
			BT_Save_S_Version(&s_v_buf[0]);
		}
		#else
			if (!BT_Read_Update_OK_Flag()) return;
		#endif
		
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "[YQEU OS] The APP has been detected in BOOT and is ready to mount the BOOT APP...\n"));
		goto pre_jump;
	}
	
	pre_jump:
		BT_Set_RunStatus(PRE_JUMP);
		BT_Stop_Interrupt();
		BT_Jump_APP_Handler(BT_APP_RUN_ADDR);
		/* If the jump fails, perform the follwing */
		BT_Start_Interrupt();		
		goto reset_dp;

	reset_dp :
		bt_pcb.d_pack_index = 1;
		bt_pcb.dp_w_addr = BT_DP_SAVE_BASE_ADDR;
		BT_Save_Temp_S_Version(&s_v_buf[0]);
		BT_Save_S_Version(&s_v_buf[0]);
		bt_pcb.no_app_flag = 1;	//Set the app exception flag.
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "[YQEU OS]No APP is detected, or APP exception is detected, the software version is cleared, and the firmware is to be retrieved...\n"));
		return;
}
#endif

/**
 @brief		: Handle the upgrade operation based on the software version obtained from the server.
 @param		: 1.soft version buffer.
						2.rollback flag.
 @return	: None						
 */
void BT_Version_Handle(bt_u8 *v_buf, bt_u8 is_rollback) 
{
	bt_u8 s_v_buf[2];
	bt_u16 read_s_v_value;
	bt_u16 rcv_s_v_value;
	
	v_buf[0] = BT_Hex_To_Dec(v_buf[0]);
	v_buf[1] = BT_Hex_To_Dec(v_buf[1]);
	rcv_s_v_value = ((v_buf[0] << 8) | v_buf[1]);
	BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "Software version number received : %d-%d\n", v_buf[0], v_buf[1]));  
	/* Fetch the saved running software version and compare it with the received one */
	BT_Read_S_Version(&s_v_buf[0]);	
	read_s_v_value = ((s_v_buf[0] << 8) | s_v_buf[1]);
	BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "Running software version : %d-%d\n", read_s_v_value >> 8, read_s_v_value & 0xFF));
	
	if ((rcv_s_v_value <= read_s_v_value) && (!is_rollback)) 
	{
		BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "\nNo new firmware downloads...\n"));
		return;
	}
	
	/* 
	 *Determine whether the version of the server is the same as the temporary one. 
	 *If not, reset the package order (set 1) and reset the FLASH download address. 
	 *Then save the version of the server and get it again from the first package.
	 */
	BT_Read_Temp_S_Version(&s_v_buf[0]);
	BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "Software version of temporary storage : %d-%d\n", s_v_buf[0], s_v_buf[1]));
	if ((s_v_buf[0] != v_buf[0]) || (s_v_buf[1] != v_buf[1]))
	{
		BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "The received software version number is different from the temp area. Resave and reset the package index and write address...\n"));
		BT_Save_Temp_S_Version(&v_buf[0]);
		bt_pcb.dp_buf_offset = 0;
		bt_pcb.dp_w_addr = BT_DP_SAVE_BASE_ADDR;
		BT_Save_Write_Flash_Addr(bt_pcb.dp_w_addr);
		bt_pcb.d_pack_index = 1;
		BT_Save_DataPack_Index(bt_pcb.d_pack_index);
	}
	BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "Get the new software version number and prepare to send the get firmware package\n"));

	/*
	 *These three conditions can be met in the current program to get the packet:
	 *1. In the BOOT area.
   *2. In the BOOT area, and there is only one application area.
   *3. In the application area, and there is an application staging area.
	 */
	#if ((BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR) || BT_IS_BOOT_ZONE)
		bt_pcb.d_pack_en_get_flag = 1;	//Enable to get the firmware package.
	#else
		/*
		 *In the application area, if there is only one running area at the moment and there is no need to copy, 
		 *jump to the BOOT area to upgrade the device.
		 */
	BT_DEBUGF(BT_V_DBG, (BT_PrintDBG_Buf, "Prepare jump to boot for cover updata...\n"));
	BT_Set_Cover_Update_Flag();
	BT_Stop_Interrupt();
	BT_Jump_BOOT(BT_BOOT_ADDR);
	BT_Start_Interrupt();
	#endif
	
	#if (!(BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR) && BT_IS_BOOT_ZONE)
		BT_Clear_Update_OK_Flag();
	#endif
}


/**
 @brief		: Depending on the frame type, the frame cache is filled with the contents required for different frame types.
						The contents filled in are sent to the server.
 @param		: 1.	frame_index.
						2.	Frame valid data part length.
						3.	refer to enum FRAME_TYPE.
 @return	: None
 */
void BT_Fill_Frame_Content(bt_u16 *pframe_index, bt_u16 pframe_data_len, bt_u16 frame_type)
{
	bt_u16 frame_crc_val = 0;
	bt_u16 index = 0;
	bt_u8 i = 0;
	
	bt_pcb.psend_frame[index++] = BT_FRAME_HEAD;
	bt_pcb.psend_frame[index++] = BT_HighByte(pframe_data_len);
	bt_pcb.psend_frame[index++] = BT_LowByte(pframe_data_len);
	/* The frame serial number */
	for (i = 0; i < 5; i++)
		bt_pcb.psend_frame[index++] = bt_pcb.f_index_buf[i];
	bt_pcb.psend_frame[index++] = BT_HighByte(frame_type);
	bt_pcb.psend_frame[index++] = BT_LowByte(frame_type);
	bt_pcb.psend_frame[index++] = BT_HighByte(BT_DEVICE_ID);
	bt_pcb.psend_frame[index++] = BT_LowByte(BT_DEVICE_ID);
	bt_pcb.psend_frame[index++] = BT_M_S_DEV_FLAG;
	
	switch (frame_type)
	{ 
		case BT_REPLY_DEV_VER_FRAME :
			/* Remote rollback of the flag */
			bt_pcb.psend_frame[index++] = BT_ROLLBACK_FLAG;
			/* Query cycle value */
			BT_Read_Query_Cycle_Value(&bt_pcb.que_v_cyc_val);
			bt_pcb.psend_frame[index++] = BT_HighByte(bt_pcb.que_v_cyc_val);	
			bt_pcb.psend_frame[index++] = BT_LowByte(bt_pcb.que_v_cyc_val);
			/* Boot software version */
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.bt_sw_v_buf[0]);
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.bt_sw_v_buf[1]);
			bt_pcb.psend_frame[index++] = 0xE1;
			/* Run software version and hardware version */
			BT_Read_S_Version(&bt_pcb.sw_v_buf[0]);
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.sw_v_buf[0]);
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.sw_v_buf[1]);
			bt_pcb.psend_frame[index++] = 0xE1;

			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.hw_v_buf[0]);
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.hw_v_buf[1]);
			bt_pcb.psend_frame[index++] = 0xE1;
		break;
		
		case BT_ACQUIRE_DATA_FRAME :
			BT_Read_Temp_S_Version(&bt_pcb.sw_v_buf[0]);
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.sw_v_buf[0]);
			bt_pcb.psend_frame[index++] = BT_Dec_To_Hex(bt_pcb.sw_v_buf[1]);
			bt_pcb.psend_frame[index++] = 0xE1;
			/* Firmware package length */
			bt_pcb.psend_frame[index++] = BT_HighByte(BT_DP_SIZE);
			bt_pcb.psend_frame[index++] = BT_LowByte(BT_DP_SIZE);
			/* Firmware package index */
			bt_pcb.psend_frame[index++] = BT_HighByte(bt_pcb.d_pack_index);
			bt_pcb.psend_frame[index++] = BT_LowByte(bt_pcb.d_pack_index);
		break;
		
		case BT_GENERAL_REPLY_FRAME :
			/* Receipt result */
			bt_pcb.psend_frame[index++] = bt_pcb.r_result;	
		break;
	}
	
	/* Use length for reserved bits */
	bt_pcb.psend_frame[index++] = RESERVED_LEN;
	/* Reserve bit data buffer */
	for (i = 0; i < RESERVED_LEN; i++)
		bt_pcb.psend_frame[index++] = bt_pcb.re_buf[i];
		
	bt_pcb.psend_frame[index++] = BT_DATE_USE_FLAG;
	
	#if BT_DATE_USE_FLAG
		for (i = 0; i < BT_DATE_LEN; i++)
			bt_pcb.psend_frame[(*pframe_index)++] = bt_pcb.date_buf[i];
	#endif
	
	frame_crc_val = BT_CRC16_XMODEM(&bt_pcb.psend_frame[3], pframe_data_len);
	bt_pcb.psend_frame[index++] = BT_HighByte(frame_crc_val);
	bt_pcb.psend_frame[index++] = BT_LowByte(frame_crc_val);
	
	/* Frame tail */
	for (i = 0; i < BT_FRAME_END_LEN; i++)
		bt_pcb.psend_frame[index++] = (i % 2 == 0) ? 0x0D : 0x0A;
		
	*pframe_index = index;
	printf("end pframe_index = %d\n", *pframe_index);
}

/**
 @brief		: Different protocol frame data messages are sent to the server according to 
						the different protocol frame types that are passed in.
 @param		: 1. refer to enum FRAME_TYPE.
 @return	: refer to enum BT_ERR_STA.
 */
enum BT_ERR_STA 
BT_Send_Protocol_Msg(bt_u16 frame_type)
{	
	bt_u16 frame_index = 0;
	bt_u16 frame_data_len = 0;
	
	BT_Set_RunStatus(SEND_DATA);
	
	frame_data_len = RESERVED_LEN;	//Fill the reserved bit length.
	if (frame_data_len > 256) return BT_RE_ERR;	//It is illegal to reserve field length.
	
	/* Determine whether to enter the date */
	#if (BT_DATE_USE_FLAG)
	{
		switch (frame_type)
		{
			case BT_QUERY_DEV_VER_FRAME :
				frame_data_len += (BT_QUERY_DEV_VER_FRAME_MAIN_LEN + BT_DATE_LEN);		break;
			case BT_REPLY_DEV_VER_FRAME :
				frame_data_len += (BT_REPLY_DEV_VER_FRAME_MAIN_LEN + BT_DATE_LEN);		break;
			case BT_ACQUIRE_DATA_FRAME :
				frame_data_len += (BT_ACQUIRE_GET_DATA_FRAME_MAIN_LEN + BT_DATE_LEN);	break;
			case BT_GENERAL_REPLY_FRAME :
				frame_data_len += (BT_GENERAL_REPLY_FRAME_MAIN_LEN + BT_DATE_LEN);		break;			
		}
	}
	#else
	{
		switch (frame_type)
		{
			case BT_QUERY_DEV_VER_FRAME :
				frame_data_len += BT_QUERY_DEV_VER_FRAME_MAIN_LEN;		break;
			case BT_REPLY_DEV_VER_FRAME :
				frame_data_len += BT_REPLY_DEV_VER_FRAME_MAIN_LEN;		break;
			case BT_ACQUIRE_DATA_FRAME :
				frame_data_len += BT_ACQUIRE_GET_DATA_FRAME_MAIN_LEN;	break;
			case BT_GENERAL_REPLY_FRAME :
				frame_data_len += BT_GENERAL_REPLY_FRAME_MAIN_LEN;		break;
		}	
	}
	#endif
	BT_DEBUGF(BT_SD_DBG, (BT_PrintDBG_Buf, "Send frame size = %d\n", frame_data_len));
	
	bt_pcb.psend_frame = (bt_u8 *)malloc(frame_data_len * sizeof(bt_u8));	//Request memory space for send data frames.
	if(bt_pcb.psend_frame == NULL)
	{
		BT_ERROR_ASSERT("pframe apply for memory err");
		return BT_MEM_ERR;
	}
	
	/*
	 *Get the true data part length.
	 *3 is to remove the two bytes occupied by the frame header and data length.
	 */
	frame_data_len -= (3 + BT_CRC_LEN + BT_FRAME_END_LEN);	

	//if (frame_type != BT_REPLY_DEV_VER_FRAME)
	BT_Fill_Frame_Content(&frame_index, frame_data_len, frame_type);
		
	bt_pcb.s_h_func(bt_pcb.psend_frame, frame_index);	//The data frames are sent via the mounted communication hardware.
	free(bt_pcb.psend_frame);
	
	return BT_OK;
}

/**
 @brief		: Based on the timer, cycle through the software version query and obtain the firmware package.
						Loop the query based on the timer count provided by the developer.
 @param		: The BT_Query_CycVal variable is added to the timer handler in the developer's own implementation
 @return	: None
 */
void BT_Cycle_Query_SV_and_DP(void)
{
	static bt_u32 cyc_val;
	static bt_u32 cyc_last_val;
	static bt_u8 que_v_to_get_dp_flag = 0;
	bt_u32 cyc_temp;
	
	bt_pcb.d_pack_en_get_flag ? BT_Set_RunStatus(CYC_QUE_DP) : BT_Set_RunStatus(CYC_QUE_VER);
	
	/* The first assignment cycle query after boot */
	if (!bt_pcb.cyc_que_sw_start_flag)
	{
		BT_SET_RAND_SEED(BT_Rand_Seed_Num);
		cyc_val = BT_GET_RAND_VALUE(BT_CYCLE_QUERY_RAND_RANGE);
		cyc_last_val = cyc_val;
		BT_DEBUGF(BT_RAND_DBG, (BT_PrintDBG_Buf, "Query value of the first cycle after boot : %ds and %dms\n", cyc_val / 1000, cyc_val % 1000));
		bt_pcb.cyc_que_sw_start_flag = 1;
		BT_Query_CycVal = 0;
	}

	if (que_v_to_get_dp_flag && bt_pcb.d_pack_en_get_flag)
	{
		cyc_temp = 3000;
			BT_SET_RAND_SEED(BT_Rand_Seed_Num);
			cyc_val = BT_GET_RAND_VALUE(BT_CYCLE_QUERY_RAND_RANGE) + cyc_temp;
		que_v_to_get_dp_flag = 0;
	}
	
	/* No query time, exit. */
	if ((BT_Query_CycVal * BT_TIMER_PERIOD) < cyc_val) return;
	
	/*
   *plus random value fetch in the 5S range, gets the packet.
	 */
	if (bt_pcb.d_pack_en_get_flag)
	{	
		BT_DEBUGF(BT_RAND_DBG, (BT_PrintDBG_Buf, "Get the period value of the packet : %ds and %dms\n", cyc_val / 1000, cyc_val % 1000));
		BT_Send_Protocol_Msg(BT_ACQUIRE_DATA_FRAME);
		BT_Read_GetDP_Cycle(&bt_pcb.dp_get_cyc_val);
		cyc_temp = bt_pcb.dp_get_cyc_val * 1000;
	}
	/* Query device software version */
	else
	{
		BT_DEBUGF(BT_RAND_DBG, (BT_PrintDBG_Buf, "Query device software version cycle values : %ds and %dms\n", cyc_val / 1000, cyc_val % 1000));
		BT_Send_Protocol_Msg(BT_QUERY_DEV_VER_FRAME);
		que_v_to_get_dp_flag = 1;
		/* 
     *When no application is detected, or the application is abnormal, 
		 *it is necessary to quickly check the software version and download the program, 
		 *so it is not the same as the normal cycle cycle query time, here 2S plus 5S random range.
		 */
		BT_Read_Query_Cycle_Value(&bt_pcb.que_v_cyc_val);
		cyc_temp = (bt_pcb.no_app_flag > 0) ? 2000 : (bt_pcb.que_v_cyc_val * 1000);
	}
	
	do{
		BT_SET_RAND_SEED(BT_Rand_Seed_Num);
		cyc_val = BT_GET_RAND_VALUE(BT_CYCLE_QUERY_RAND_RANGE) + cyc_temp;
	}while (BT_OPT_RAND(cyc_last_val, cyc_val));
	cyc_last_val = cyc_val;	
	BT_Query_CycVal = 0;
}

/**
 @brief		: The frame header and the frame tail conforming to the protocol specification 
						are found from the received data cache and copied to the receiving protocol frame pointer for further processing.
						
 @param		: 1.The first address of the received data buffer.
						2.Length of data cache received.
 
 @return	: refer to enum BT_ERR_STA
 */
bt_u8 BT_Frame_Format_Check(bt_u8 *rcv_buf ,bt_u16 *rcv_len)
{
	bt_u16 frame_head_addr = 0, frame_end_addr = 0;
	bt_u8 frame_end_num = 0;	//Count the number of frames tail
	bt_u16 i;
	
	for (i = 0; i < *rcv_len; i++)
	{
		if (rcv_buf[i] / 16 == 0)
			BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "0%X ", rcv_buf[i]));
		else
		BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "%X ", rcv_buf[i]));
		/* Locate the frame head */
		if (!frame_head_addr)
		{
			if (rcv_buf[0] == 0xFE)	//The first byte is the frame header
				frame_head_addr = 0xFFFF;	//Temporarily assign the value to facilitate later determination of whether to get the frame head
			else if (rcv_buf[i] == 0xFE)	//The i byte is the frame header
				frame_head_addr = i; //Get the position of the frame head
		}
		/* The frame head has been found. Find the position of the frame tail */
		else if (frame_head_addr)	//If the frame header is obtained successfully
		{
			/* Frame tail : 0x0D 0x0A 0x0D 0x0A 0x0D 0x0A */
			if ((rcv_buf[i] == 0x0D) && (frame_end_num % 2 == 0))
				frame_end_num += 1;
			else if ((rcv_buf[i] == 0x0D) && (frame_end_num % 2 == 1))
				frame_end_num = 1;
			else if ((rcv_buf[i] == 0x0A) && (frame_end_num % 2 == 1))
				frame_end_num += 1;
			else
				frame_end_num = 0;

			if (frame_end_num == 6)	//Find the frame tail
			{
				frame_end_addr = i;
				break;
			}
		}
	}
	
	BT_DEBUGF(BT_RD_DBG ,(BT_PrintDBG_Buf, " }\n"));
	
	if (!frame_end_addr) return BT_CHECK_ERR;
	
	if (frame_head_addr == 0xFFFF) frame_head_addr = 0;
	*rcv_len = frame_end_addr - frame_head_addr + 1;
	bt_pcb.prcv_frame = (bt_u8 *)malloc(sizeof(bt_u8) * (*rcv_len));
	if (bt_pcb.prcv_frame == NULL)	//Application for memory failed!
	{
		BT_ERROR_ASSERT("pframe apply for memory err");
		return BT_MEM_ERR;
	}
	for (i = 0; i < *rcv_len; i++)
		bt_pcb.prcv_frame[i] = rcv_buf[i];
	
	return BT_OK;
}

/**
 @brief		: Check that the general format of data frames is correct.
						Check CRC.
						
 @param		: 1.received data length.
 
 @return	: refer to enum BT_ERR_STA
 */
enum BT_R_RESULT BT_Frame_Check(bt_u16 rcv_len)
{
	bt_u16 frame_data_len = (((bt_u16)bt_pcb.prcv_frame[1] << 8) | (bt_u16)bt_pcb.prcv_frame[2]);
	bt_u16 crc16_temp = (((bt_u16)bt_pcb.prcv_frame[rcv_len - BT_FRAME_END_LEN - 2] << 8) | 
												(bt_u16)bt_pcb.prcv_frame[rcv_len - BT_FRAME_END_LEN - 1]);
												
	if (frame_data_len > (rcv_len - 3)) return BT_R_CRC_ERR;
	
	BT_DEBUGF(BT_CRC_DBG, (BT_PrintDBG_Buf, "crc16 = %X %X\n", crc16_temp >> 8, crc16_temp & 0xFF));
	BT_DEBUGF(BT_CRC_DBG, (BT_PrintDBG_Buf, "cacu crc16 = %X %X\n", (BT_CRC16_XMODEM(&bt_pcb.prcv_frame[3], frame_data_len) >> 8), 
	BT_CRC16_XMODEM(&bt_pcb.prcv_frame[3], frame_data_len) & 0xFF));
	/* Check CRC */
	if (crc16_temp != (BT_CRC16_XMODEM(&bt_pcb.prcv_frame[3], frame_data_len)))
	{
		BT_DEBUGF(BT_CRC_DBG, (BT_PrintDBG_Buf, "CRC16 check err\n"));
		return BT_R_CRC_ERR;
	}
	
	/* Check device type */
	if ((bt_pcb.prcv_frame[10] != (BT_DEVICE_ID >> 8)) || ((bt_pcb.prcv_frame[11] != (BT_DEVICE_ID & 0xFF))))
		return BT_R_DEV_TYPE_ERR; 
		
	return BT_R_OK;
}

/**
 @brief		: Gets the content of the reserved bit.
 @param		: refer to  enum FRAME_TYPE.
 @return	: 1:OK; 0:Err.
 */
bt_u8 BT_Get_Rcv_Reserved(bt_u16 frame_type)
{
	bt_u8 i;
	bt_u8 re_len, re_base_index;

	switch (frame_type)
	{
		case BT_QUERY_DEV_VER_FRAME : re_base_index = 14; break;
		case BT_REPLY_DEV_VER_FRAME : re_base_index = 23; break;
		case BT_FORCE_UPDATA_FRAME 	: re_base_index = 19; break;
		case BT_SEND_DATA_FRAME			: re_base_index = 24; break;
		case BT_GENERAL_REPLY_FRAME	: re_base_index = 15; break;

		default : return 0;
	}

	re_len = bt_pcb.prcv_frame[re_base_index - 1];
	if (re_len != RESERVED_LEN)
	{
		BT_DEBUGF(BT_RE_DBG, (BT_PrintDBG_Buf, "The length of Reserved bit is : %d, Error!\n", re_len));
		return 0;
	}

	bt_pcb.prcv_re_buf = (bt_u8 *)malloc(RESERVED_LEN * sizeof(bt_u8));
	if (bt_pcb.prcv_re_buf == NULL)
	{
		BT_ERROR_ASSERT("Memory application failed!");
		return 0;
	}

	BT_DEBUGF(BT_RE_DBG, (BT_PrintDBG_Buf, "\nThe data of Reserved bit is : \n"));
	for (i = 0; i < RESERVED_LEN; i++)
	{
		bt_pcb.prcv_re_buf[i] = bt_pcb.prcv_frame[re_base_index + i];
		BT_DEBUGF(BT_RE_DBG, (BT_PrintDBG_Buf, "%X ", bt_pcb.prcv_re_buf[i]));
	}
	BT_DEBUGF(BT_RE_DBG, (BT_PrintDBG_Buf, "\n"));	

	return 1;
}

/**
 @brief		: Receive data and verify the processing.
 @param		: 1.The first address of the received data buffer.
						2.Length of data received.
 @return	: refer to enum BT_ERR_STA 
 */
#if USE_TIM_RCV_MSG
enum BT_ERR_STA  BT_Receive_Protocol_Msg(bt_u8 *rcv_buf, bt_u16 *rcv_len)
#else
enum BT_ERR_STA  BT_Receive_Protocol_Msg(bt_u8 *rcv_buf, bt_u16 *rcv_len, bt_u8 *rcvd_flag)
#endif
{
	bt_u16 frame_type;
	bt_u8 i = 0;

	BT_Set_RunStatus(RCV_DATA);
	
	/* Idle or receiving data */
	#if USE_TIM_RCV_MSG
	if (!BT_Timer_Handler(rcv_len, &BT_Rcv_Tim_Num)) return BT_OK;
	#else
	if (!BT_Receive_Handler(rcv_len, rcvd_flag)) return BT_OK;
	#endif
	
	if (!bt_pcb.rcv_buf_len) return BT_OK;	//No data.
	
	/* 
	 *Copy the received data and check the frame format.
	 *Note that if BT_OK is returned, memory has been requested, remember to free later.
	 */	
	if (BT_Frame_Format_Check(&rcv_buf[0], &bt_pcb.rcv_buf_len) != BT_OK) return BT_CHECK_ERR;
	
	/* Check the master-slave flag */	
	if ((bt_pcb.prcv_frame[12] != 1) && (bt_pcb.prcv_frame[12] != 0x55))	goto _check_err;
	
	/* Check the entire data frame */
	bt_pcb.r_result = BT_Frame_Check(bt_pcb.rcv_buf_len);
	if (bt_pcb.r_result != BT_R_OK)
	{
		BT_Send_Protocol_Msg(BT_GENERAL_REPLY_FRAME);
		goto _check_err;
	}
	
	frame_type = ((bt_pcb.prcv_frame[8] << 8) | bt_pcb.prcv_frame[9]);
	/* Developers use it to determine whether the reserved bytes meet the application's requirements */
	if(!BT_Get_Rcv_Reserved(frame_type)) goto _check_err;
	bt_pcb.r_result = BT_ReservedByte_Check(frame_type, &bt_pcb.prcv_re_buf[0]);
	free(bt_pcb.prcv_re_buf);
	if (bt_pcb.r_result != BT_R_OK)	goto _check_err;
	
	/* Gets the frame sequence number */
	for (i = 0; i < 5; i++)
		bt_pcb.f_index_buf[i] = bt_pcb.prcv_frame[3 + i];
		
	/* Handles the content of different types of frames */
	switch (frame_type)
	{
		case BT_QUERY_DEV_VER_FRAME : 
			BT_Send_Protocol_Msg(BT_REPLY_DEV_VER_FRAME); 
			BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "\nReceived server query version frame\n"));
		break;

		case BT_REPLY_DEV_VER_FRAME :
			BT_Send_Protocol_Msg(BT_GENERAL_REPLY_FRAME);
			BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "\nReceived Software version frame\n"));
			bt_pcb.sw_v_buf[0] = bt_pcb.prcv_frame[16];
			bt_pcb.sw_v_buf[1] = bt_pcb.prcv_frame[17];

			BT_Save_Query_Cycle_Value((bt_pcb.prcv_frame[14] << 8 | bt_pcb.prcv_frame[15]));
			BT_Version_Handle(&bt_pcb.sw_v_buf[0], bt_pcb.prcv_frame[13]); 
		break;
		
		case BT_FORCE_UPDATA_FRAME : 
			BT_Send_Protocol_Msg(BT_GENERAL_REPLY_FRAME); 
			BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "\nreceives the force upgrade frame\n"));
			for (i = 0; i < 2; i++)
				bt_pcb.sw_v_buf[i] = bt_pcb.prcv_frame[14 + i];
			BT_Version_Handle(&bt_pcb.sw_v_buf[0], bt_pcb.prcv_frame[13]);
		break;
		
		case BT_SEND_DATA_FRAME : 
			if (bt_pcb.d_pack_en_get_flag)
			{
				bt_pcb.r_result = BT_DataPack_Handler(bt_pcb.prcv_frame);
				BT_Send_Protocol_Msg(BT_GENERAL_REPLY_FRAME);
				BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "\nReceived data packet frame\n"));
				if (bt_pcb.r_result != BT_R_OK) goto _check_err;
			}
		break;
		
		case BT_GENERAL_REPLY_FRAME :
			BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "Received the general receipt of the server, the processing result code : %X\n", bt_pcb.prcv_frame[13]));
			if (bt_pcb.prcv_frame[13] == BT_R_S_VER_ERR)
			{
				BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "Software version number error getting data from server, ready to correct!\n"));
				bt_pcb.d_pack_en_get_flag = 0;	//Re-query the software version.
			}
		break;
			
		default : 
			goto _check_err;
	}
	
	free(bt_pcb.prcv_frame);
	return BT_OK;
	
	_check_err :
	free(bt_pcb.prcv_frame);
	return BT_CHECK_ERR;
}

/**
 @brief		: Process and write the obtained firmware package.
 @param		: data package buffer.
 @return	: refer to enum BT_R_RESULT
 */
enum BT_R_RESULT BT_DataPack_Handler(bt_u8 *d_buf) 
{ 
	bt_u16 dp_len, dp_index;
	bt_u16 dp_base_addr, i, j = 0;
	bt_u16 dp_crc_val, dp_crc_base_addr;
	bt_u8 tmp_sv[2];
	
	/* Determine if it is the required version of the software */
	BT_Read_Temp_S_Version(&tmp_sv[0]);
	d_buf[15] = BT_Hex_To_Dec(d_buf[15]);
	d_buf[16] = BT_Hex_To_Dec(d_buf[16]);
	if ((d_buf[15] != tmp_sv[0]) || (d_buf[16] != tmp_sv[1]))
	{
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "The software version of the packet is : %X-%X ERR!\n", d_buf[13], d_buf[14]));
		return BT_R_DP_ERR;
	}

	if (bt_pcb.dp_get_cyc_val != (d_buf[13] << 8 | d_buf[14]))
	{
		BT_Save_GetDP_Cycle((d_buf[13] << 8 | d_buf[14]));
		BT_Read_GetDP_Cycle(&bt_pcb.dp_get_cyc_val);
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Get new firmware cycle value: %d s\n", bt_pcb.dp_get_cyc_val));
	}
	
	/* Determine if it is the last packet of data */
	bt_pcb.d_pack_end_flag = (d_buf[22] == 1);
	
	/*Determine whether the firmware package length is legal*/
	dp_len = ((d_buf[18] << 8) | d_buf[19]);	
	if ((dp_len != BT_DP_SIZE && !bt_pcb.d_pack_end_flag)) //|| dp_len == 0
	{
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "The length of the firmware package sent by the server is illegal!\n"));
		return BT_R_DPL_ERR;
	}
	
	/* Determine whether the firmware package sequence is legal */
	dp_index = ((d_buf[20] << 8) | d_buf[21]);
	if (dp_index != bt_pcb.d_pack_index) 
	{
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "The firmware package order sent by the server is illegal!\n"));
		return BT_R_DP_ERR;
	}
	
	/* 
	 *Gets the first address of the packet.
	 *23 is the reserved bit using the first 18 bytes of length, 
	 *d_buf[18] is the reserved bit using the length of bytes, 
	 *the last 1 is the reserved bit using the length of the bit itself
	 */
	dp_base_addr = 23 + d_buf[23] + 1;
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Received the first address of the packet : %d\n", dp_base_addr));
	
	/* Check the CRC of the packet */
	if (!bt_pcb.d_pack_end_flag)
	{
		dp_crc_base_addr = dp_base_addr + BT_DP_SIZE;
		dp_crc_val = BT_CRC16_XMODEM(&d_buf[dp_base_addr], BT_DP_SIZE);
	}
	else
	{
		dp_crc_base_addr = dp_base_addr + dp_len;
		dp_crc_val = BT_CRC16_XMODEM(&d_buf[dp_base_addr], dp_len);
	}
	if (dp_crc_val != ((d_buf[dp_crc_base_addr] << 8) | d_buf[dp_crc_base_addr + 1]))
	{
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Packet check code error! The local check code is : %X\n", dp_crc_val));
		return BT_R_DP_ERR;
	}
	
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Printed data :\n"));
	switch (sizeof(BT_DATAPACK_TYPE))
	{
		case 1	:
			for (i = dp_base_addr; i < (dp_base_addr + dp_len); i++)
				bt_pcb.dp_buf[bt_pcb.dp_buf_offset++] = d_buf[i];
			break;
			
		case 2 	:
			for (i = dp_base_addr; i < (dp_base_addr + dp_len); i++)
			{
				if (j % 2 == 1)
				{
					bt_pcb.dp_buf[bt_pcb.dp_buf_offset++] |= (d_buf[i] << 8);
					BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "%X ", bt_pcb.dp_buf[bt_pcb.dp_buf_offset - 1]));
				}
				else
					bt_pcb.dp_buf[bt_pcb.dp_buf_offset] = d_buf[i];

				j++;
			}
			break;
			
		default :
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "sizeof(BT_DATAPACK_TYPE) Calculation error!\n"));
	}
	j = 0;
	
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "\nCopied %d length, %d byte type data\n", bt_pcb.dp_buf_offset, sizeof(BT_DATAPACK_TYPE)));
	
	bt_pcb.d_pack_index++;
	
	/* 
   *If the specified number is not cached after receiving the firmware package 
	 *and it is not the last package, proceed with the request for receiving.
	 */
	if (!bt_pcb.d_pack_end_flag)
	{
		if ((bt_pcb.dp_buf_offset * sizeof(BT_DATAPACK_TYPE)) != BT_DP_BUF_SIZE)
			return BT_R_OK;
	}
	
	BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Prepare to write %d size, %d byte type data to FLASH\n", bt_pcb.dp_buf_offset, sizeof(BT_DATAPACK_TYPE)));
	
	BT_Write_DP_To_Flash(bt_pcb.dp_w_addr, &bt_pcb.dp_buf[0], bt_pcb.dp_buf_offset);
	bt_pcb.dp_w_addr += (bt_pcb.dp_buf_offset * sizeof(BT_DATAPACK_TYPE));
	bt_pcb.dp_buf_offset = 0;
	BT_Save_DataPack_Index(bt_pcb.d_pack_index);
	BT_Save_Write_Flash_Addr(bt_pcb.dp_w_addr);
	
	/* 
	 *If the firmware package is all downloaded, do the following.
   *1. Check whether the downloaded program is legal: BT_APP_Check().
	 *2.As in the previous step, change the save related variables and flags.
	 *3.Stop all set interrupts.
	 *4.Copy packets, if needed.
	 *5.Jump program.
 	 */
	if (bt_pcb.d_pack_end_flag)
	{
		bt_pcb.d_pack_end_flag = 0;
		
		if (BT_APP_Check(BT_DP_SAVE_BASE_ADDR))
		{
			bt_pcb.r_result = BT_R_OK;
			BT_Send_Protocol_Msg(BT_GENERAL_REPLY_FRAME);	
			
			#if (BT_IS_BOOT_ZONE && (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
			BT_Copy_DataPack((bt_pcb.dp_w_addr - BT_DP_SAVE_BASE_ADDR));	//Move packets to run address (optional)
			#elif (!BT_IS_BOOT_ZONE && (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))	//In application zone.
			BT_Save_SaveArea_DP_Size((bt_pcb.dp_w_addr - BT_DP_SAVE_BASE_ADDR));
			BT_Set_DP_Copy_Flag();
			#elif (BT_IS_BOOT_ZONE && !(BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
			BT_Set_Update_OK_Flag();
			BT_Clear_Cover_Update_Flag();
			#endif
			
			/* Reset correlation variable */
			bt_pcb.dp_w_addr = BT_DP_SAVE_BASE_ADDR;
			BT_Save_Write_Flash_Addr(bt_pcb.dp_w_addr);	
			bt_pcb.d_pack_index = 1;
			BT_Save_DataPack_Index(bt_pcb.d_pack_index);

			#if BT_IS_BOOT_ZONE
			BT_Save_S_Version(&tmp_sv[0]);
			#endif
			
			#if BT_IS_BOOT_ZONE
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Prepare to jump new programs\n"));
			#else
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "Prepare to jump BOOT for update\n"));
			#endif
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "The second set of four byte values : %X\n", (*(volatile unsigned int*)(BT_APP_RUN_ADDR + 4))));
			BT_Stop_Interrupt();
			#if BT_IS_BOOT_ZONE
			BT_Jump_APP_Handler(BT_APP_RUN_ADDR);
			#else
			BT_Jump_BOOT(BT_BOOT_ADDR);
			#endif

			BT_Start_Interrupt();
		}
		else
		{
			bt_pcb.r_result = BT_R_DP_ERR;
			BT_Send_Protocol_Msg(BT_GENERAL_REPLY_FRAME);
			BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "\nDownload of the program calibration failed, empty the parameters, ready to get again!\n"));
			bt_pcb.dp_w_addr = BT_DP_SAVE_BASE_ADDR;
			BT_Save_Write_Flash_Addr(bt_pcb.dp_w_addr);	
			bt_pcb.d_pack_index = 1;
			BT_Save_DataPack_Index(bt_pcb.d_pack_index);
			bt_pcb.d_pack_en_get_flag = 0;
		}
	}
	
	return BT_R_OK; 
}

#if (BT_IS_BOOT_ZONE && (BT_DP_SAVE_BASE_ADDR - BT_APP_RUN_ADDR))
bt_u8 BT_Copy_DataPack(bt_u32 dp_size) 
{
	bt_u32 dp_addr_offset = BT_DP_SAVE_BASE_ADDR;
	bt_u16 dp_num = dp_size / BT_FLASH_PAGE_SIZE;	
	bt_u16 dp_remain = (dp_size % BT_FLASH_PAGE_SIZE) / sizeof(BT_DATAPACK_TYPE);	
	bt_u32 dp_write_addr_offset = BT_APP_RUN_ADDR;
	bt_u16 i;
	bt_u16 dp_tmp_buf[1024];
	//bt_u16 j, k = 0;
	
	for (i = 0; i < dp_num; i++)
	{
		BT_Read_DP_From_Flash(dp_addr_offset, &dp_tmp_buf[0], 1024);
			
		BT_DEBUGF(BT_APP_DBG, (BT_PrintDBG_Buf, "\nPrepare to copy firmware...\n"));
		BT_Write_DP_To_Flash(dp_write_addr_offset, &dp_tmp_buf[0], 1024);
		
		dp_addr_offset += BT_FLASH_PAGE_SIZE;
		dp_write_addr_offset += BT_FLASH_PAGE_SIZE;
	}
	
	BT_Read_DP_From_Flash(dp_addr_offset, &dp_tmp_buf[0], dp_remain);
	
	BT_Write_DP_To_Flash(dp_write_addr_offset, &dp_tmp_buf[0], dp_remain);
	
	return 1;
}
#endif

#if USE_TIM_RCV_MSG
/**
 @brief		: Receive data according to timer assist provided by the developer.
 @param		: 1.Received length of real-time data.
						2.The developer needs to put the BT_Rcv_Tim_Num variable into the timer handler and let the variable add itself.
 @return	: 0 : Receiving data is not complete.
						1 : Completion of receiving data.
 */
bt_u8 BT_Timer_Handler(bt_u16 *rcv_len, bt_u16 *tim_num)
{
	static bt_u16 rcv_len_temp = 0;
	static bt_u8 tim_Start_flag = 0;
	static bt_u8 second_check_num = 0;
	static bt_u8 en_check_flag = 0;
	
	if (!(*rcv_len))	return 0;
	
	if (!tim_Start_flag)
	{
		tim_Start_flag = 1;
		*tim_num = 0;	
	}
	
	/* Used as a basis for judging the data received. */
	if ((*tim_num % 2 == 0) && (!en_check_flag))
	{
		rcv_len_temp = *rcv_len;
		en_check_flag = 1;
	}
	else if ((*tim_num % 2 == 1) && en_check_flag)
	{
		/*
		 *After waiting for a cycle time of LoRa_Rcv_Tim_Num, 
		 *the received length is still the same as that of the previous cycle, 
		 *indicating that the received is completed.
		 */
		 en_check_flag = 0;
		 
		if (*rcv_len == rcv_len_temp)
		{ 
			second_check_num++;
			if (second_check_num >= 2)
			{
				second_check_num = 0;
				tim_Start_flag = 0;
				rcv_len_temp = 0;
				bt_pcb.rcv_buf_len = *rcv_len;
				*rcv_len = 0;
				BT_DEBUGF(BT_RD_DBG, (BT_PrintDBG_Buf, "Received data: { "));
				return 1;
			}
		}
	}
	/* If the specified receiving time is exceeded, the receiving time is deemed to be over! */
	if (tim_Start_flag)
	{
		if ((*tim_num * BT_TIMER_PERIOD) > BT_TIMEROUT_VALUE)
		{
			tim_Start_flag = 0;
			second_check_num = 0;
			en_check_flag = 0;
			*rcv_len = 0;
			rcv_len_temp = 0;
			BT_ERROR_ASSERT("Receive data timeout!");
		}
	}

	return 0;
}
#else
/**
 @brief		: The developer can choose not to apply the timer, 
						process the received data, and then fill in the received data length and the receive flag bit��
 @param		: 1.received data length.
						2.received ok flag.
 @return	: None
 */
bt_u8 BT_Receive_Handler(bt_u16 *rcv_len, bt_u8 *rcvd_flag)
{
	if (!(*rcvd_flag)) return 0;

	*rcvd_flag = 0;
	bt_pcb.rcv_buf_len = *rcv_len;
	return 1;
}
#endif

bt_u8 BT_HighByte(bt_u16 var){ return ((var >> 8) & 0xFF); }
bt_u8 BT_LowByte(bt_u16 var){ return (var & 0xFF); }

bt_u8 BT_Hex_To_Dec(bt_u8 dat)
{
	bt_u8 temp1, temp2;

	temp1 = dat / 16;
	temp2 = dat % 16;
	temp2 = (temp1 * 10 + temp2);
	return temp2;
}

bt_u8 BT_Dec_To_Hex(bt_u8 dat)
{
	bt_u8 temp1, temp2;
	
	temp1 = dat / 10;
	temp2 = dat % 10;
	temp2 = (temp1 * 16 + temp2);
	return temp2;
}
