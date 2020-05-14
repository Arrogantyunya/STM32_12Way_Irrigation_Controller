#include "../Third_Periph_Inc/iap.h"
#include "../Third_Periph_Inc/prh_flash.h"
#include "../bt_inc/boot_config.h"
#include "../bt_inc/boot_protocol.h"

IAPFun Jump_APP;

__asm void MSR_MSP(unsigned int addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

uint8_t BT_Write_DP_To_Flash(uint32_t write_addr, uint16_t *app_buffer, uint16_t app_len)
{
	Write_Flash_Buffer(write_addr, app_buffer, app_len);
	return 1;
}

bt_u8 BT_Read_DP_From_Flash(bt_u32 write_addr, BT_DATAPACK_TYPE *app_buffer, bt_u32 app_len)
{
	Read_Flash(write_addr, app_buffer, app_len);
	
	return 1;
}


uint8_t BT_APP_Check(uint32_t app_addr) 
{
	if (((*(volatile unsigned int *)app_addr) & 0x2FFE0000) != 0x20000000)
	{
		BT_PRINT((BT_PrintDBG_Buf, "The address at the top of the stack is illegal. The wrong address is : %X\n", ((*(volatile unsigned int *)app_addr) & 0x2FFE0000)));
		return 0;
	}
	if (((*(volatile unsigned int*)(app_addr + 4)) & 0xFF000000) != 0x08000000)
	{
		BT_PRINT((BT_PrintDBG_Buf, "It is illegal to interrupt the address of the scale. The wrong address is : %X\n", ((*(volatile unsigned int*)(app_addr + 4)) & 0xFF000000)));
		return 0;
	}
	
	if (((*(volatile unsigned int*)(app_addr + 4)) & 0xFFFF0000) != BT_APP_RUN_ADDR)
	{
		BT_PRINT((BT_PrintDBG_Buf, "The offset address of interrupt vector is illegal. The error address is : %X\n", ((*(volatile unsigned int*)(app_addr + 4)) & 0xFFFF0000)));
		return 0;
	}
	
	BT_PRINT((BT_PrintDBG_Buf, "firmware check successful\n"));
	return 1;
}

void IAP_Load_APP(unsigned int obj_addr)
{                                                                                                 
	if (((*(volatile unsigned int *)obj_addr) & 0x2FFE0000) == 0x20000000)
	{
		Jump_APP = (IAPFun)*(volatile unsigned int*)(obj_addr + 4);
		MSR_MSP(*(volatile unsigned int*)obj_addr);
		Jump_APP();
	}
}

void BT_Jump_APP_Handler(unsigned int obj_addr)
{
	IAP_Load_APP(obj_addr);
}
