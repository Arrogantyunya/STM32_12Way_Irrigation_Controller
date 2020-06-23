#include "iap.h"

#include "boot_config.h"

IAPFun Jump_APP;

void MSR_MSP(unsigned int addr)
{
  asm 
  (
    "MSR MSP, r0\n\t"
    "BX r14\n\t" 
  );
}

bt_u8 BT_APP_Check(bt_u32 app_addr) 
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

void BT_Jump_BOOT(unsigned int boot_addr)
{
	nvic_sys_reset();
}