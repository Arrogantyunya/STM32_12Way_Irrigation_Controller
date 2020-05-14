#include "../Third_Periph_Inc/prh_flash.h"
#include "../Third_Periph_Inc/debug_uart.h"

extern void    FLASH_PageErase(uint32_t PageAddress);

FLASH_EraseInitTypeDef EraseInitStruct;

unsigned int SECTORError;

/*
 *According to the chip core and chip FLASH size to determine the chip FLASH page byte size
 */
#if (BT_MCU_CORE_TYPE == BT_Cotex_M3)
	#if (BT_DEVICE_FLASH_SIZE < 256)
		#define PAGE_SIZE 1024
	#else
		#define PAGE_SIZE 2048
	#endif
#endif

/* 
 *The page size is bytes, and the content of the write/read is half a word, 
 *so the length of the array to cache a page is divided by 2 
 */
unsigned short FLASH_Page_Buffer[PAGE_SIZE / 2]; 

/**
 @brief		: Reads the half word of the specified address.
 @param		: The target address to read (this parameter must be a multiple of 2).
 @return	: Returns half - word data buffer.
 */
unsigned short Read_Flash_HalfWord(unsigned int read_addr)
{
	return *(volatile unsigned short*)read_addr;
}

/**
 @brief		: Reads a half-word of the specified length from the specified address.
 @param		: 1.The specified base address to read the data;
            2.Data cache semi-alphanumeric group;
						3.Length of data to read.
 @return	: None
 */
void Read_Flash(unsigned int read_addr, unsigned short *pdata_buffer, unsigned int data_len)
{
	unsigned int i;
	for (i = 0; i < data_len; i++)
	{
		pdata_buffer[i] = Read_Flash_HalfWord(read_addr);
		read_addr += 2;	//Offset two addresses, continue to read the following half-word data.
	}
}

/**
 @brief		: Writes the halfword data without checking to see if it needs to be erased.
 @param		: 1. Address to write data to.
						2. Packet to write.
						3. How much data to write.
 @return	: None
 */
void Write_Flash_NoCheck(unsigned int write_addr, unsigned short *pdata_buf, unsigned int data_len)
{
	unsigned int i;

	for (i = 0; i < data_len; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, write_addr, pdata_buf[i]);
		write_addr += 2;
	}
}

/**
 @brief		: Based on the address to be written, the length of the data to be written, 
						calculate which page to start and which page to end.
 @param		: 1.write flash base address.
					  2.write data buffer.(half-word).
						3.write data length.
 @return	: None
 */
void Write_Flash_Buffer(unsigned int write_addr, unsigned short *pdata_buf, unsigned int data_len)
{
	unsigned int Page_Addr; //The starting address of the page to write to.
	unsigned short Page_Offset; //The offset address of the page to write to.
	unsigned short Page_Remain; //On this page, count from the offset address how many bytes are left on the page.
	unsigned int Write_Base_Addr;		//The actual address to write to.
	unsigned int i;
	
	/* Determines if the starting address to write to is out of bounds */
	if ((write_addr < BT_DEVICE_FLASH_BASE_ADDR) || (write_addr > (BT_DEVICE_FLASH_SIZE * 1024 + BT_DEVICE_FLASH_BASE_ADDR)))
		return;
		
	BT_PRINT((BT_PrintDBG_Buf, "\n\nwrite_addr == 0x%X, end_addr == 0x%X\n\n", write_addr, write_addr + data_len * 2));
	
	HAL_FLASH_Unlock();
		
	Write_Base_Addr = write_addr - BT_DEVICE_FLASH_BASE_ADDR;
	Page_Addr = Write_Base_Addr / PAGE_SIZE;
	Page_Offset = (Write_Base_Addr % PAGE_SIZE) / 2;
	Page_Remain = (PAGE_SIZE / 2) - Page_Offset;
	
	/* All the data can be written in one page */
	if (data_len <= Page_Remain)
		Page_Remain = data_len;
		
	while (1)
	{
		                      
		Read_Flash((Page_Addr * PAGE_SIZE + BT_DEVICE_FLASH_BASE_ADDR), FLASH_Page_Buffer, PAGE_SIZE / 2);
		
		for (i = 0; i < Page_Remain; i++)
			if (FLASH_Page_Buffer[Page_Offset + i] != 0xFFFF) break;
		
		/* If any address is not cleared */
		if (i < Page_Remain)
		{
			EraseInitStruct.PageAddress = Page_Addr * PAGE_SIZE + BT_DEVICE_FLASH_BASE_ADDR;
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
			EraseInitStruct.NbPages = 1;
			HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
	
			for (i = 0; i < Page_Remain; i++)
				FLASH_Page_Buffer[Page_Offset + i] = pdata_buf[i];
			
			Write_Flash_NoCheck(Page_Addr * PAGE_SIZE + BT_DEVICE_FLASH_BASE_ADDR, FLASH_Page_Buffer, PAGE_SIZE/ 2);
		}
		else
			Write_Flash_NoCheck(write_addr, pdata_buf, Page_Remain);
		
		if (data_len == Page_Remain)	break; //The end of writing
		
		else
		{
			/*
			 *After the start page is written, 
			 *the remaining data continues to be written from the start offset of the second page
			 */
			Page_Addr += 1;
			Page_Offset = 0;
			pdata_buf += Page_Remain;
			write_addr += (Page_Remain * 2);
			data_len -= Page_Remain;
			
			Page_Remain = (data_len > (PAGE_SIZE / 2)) ? PAGE_SIZE / 2 : data_len;
		}
	};
	
	HAL_FLASH_Lock();
}
