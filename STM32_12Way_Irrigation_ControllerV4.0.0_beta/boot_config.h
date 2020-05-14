#ifndef _BOOT_CONFIG_H_
#define _BOOT_CONFIG_H_

#define USE_ARDUINO                   1

/*** User-supplied chip kernel or peripheral header file ***/
#if USE_ARDUINO
#include <Arduino.h>
#else
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "../Third_Periph_Inc/debug_uart.h"
#endif
/**************************************/

typedef unsigned char		bt_u8;
typedef unsigned short 	bt_u16;
typedef unsigned int 		bt_u32;

/* Option for ST company's Cotex series kernel */
#define BT_Cotex_M3								0
#define BT_Cotex_M4								1
#define BT_MCU_CORE_TYPE					BT_Cotex_M3


/* The FLASH size, in kilobytes, that fills the device's chip */
#define BT_DEVICE_FLASH_SIZE				256	//KB

/* The on-chip RAM size of the device chip is in KB */
#define BT_DEVICE_RAM_SIZE					64	//KB

#define BT_FLASH_PAGE_SIZE          2048  //Bytes

/* 
 *Fill in the starting address of the FLASH program on the chip of this device, 
 *which is the first address to run the program by default after the electrical reset on the chip
 */
#define BT_DEVICE_FLASH_BASE_ADDR		0x08000000

/* The starting address of the boot run */
#define BT_BOOT_ADDR								0x08000000

/* Fill in the newly acquired program temporary starting address of the chip */
#define BT_DP_SAVE_BASE_ADDR				0x08028000	   //96KB temporary Flash size.	

/* Fill in the starting address of the running application */
#define BT_APP_RUN_ADDR							0x08010000 // 96KB Run Flash size.

/* Distinguish whether the protocol is applied in the boot or application area */
#define BT_IS_BOOT_ZONE							0

/*
 *Whether to provide hardware timer for this protocol.The protocol will use the timer 
 *to complete the data receiving judgment timeout and other work
 */
#define USE_TIM_RCV_MSG             0

/*
 *Fill in the device ID, which is the device type, to distinguish it from other devices.
 *Note: limit to two bytes.
 */
#define BT_DEVICE_ID								0xC001


/***
 *Here are some Settings that involve debugging macros such as print information.
 *If you don't need to print a debug type, you can set the macro of that type to 0, or if you do, to 1. 
 *If you don't need all the debug print information at all, you can set the NO_DBG_ALL macro to 1
 */

extern char BT_PrintDBG_Buf[200];
/* Basic information debugging */
#if USE_ARDUINO
#define BT_PRINT(x) do { sprintf x; Serial.print(BT_PrintDBG_Buf);}while(0)
#else
#define BT_PRINT(x) do { sprintf x; printf("%s", BT_PrintDBG_Buf);}while(0)
#endif

/* Conditional information debugging */
#define BT_ASSERT(message, assertion) do { if (!(assertion)) { \
  BT_PRINT_DEBUG(message); }} while(0)
	
/* Type information debugging */
#define NO_DBG_ALL			0 //Mask all types
#define BT_RD_DBG				1	//Receive data information for debugging
#define BT_SD_DBG				1 //Send data information for debugging
#define BT_V_DBG				1 //Version number information debugging
#define BT_CRC_DBG			0 //CRC information debugging
#define BT_APP_DBG			1	//The application handles debugging
#define BT_RAND_DBG			1 //Random number cycle debugging
#define BT_RE_DBG       1 //Reserved bit information debugging.
#define BT_CB_DBG       1 //Callback function debugging.

#define BT_DEBUGF(debug, message) do { if ((debug & !NO_DBG_ALL)){BT_PRINT(message);}} while(0)

/* Error message debugging */
#if USE_ARDUINO
#define BT_ERROR_ASSERT(x) do {sprintf(BT_PrintDBG_Buf, "BT ERR MSG: \"%s\" error at line [%d] in %s\n", \
                                     x, __LINE__, __FILE__); Serial.print(BT_PrintDBG_Buf);} while(0)
#else
#define BT_ERROR_ASSERT(x) do {sprintf(BT_PrintDBG_Buf, "BT ERR MSG: \"%s\" error at line [%d] in %s\n", \
                                     x, __LINE__, __FILE__); printf("%s", BT_PrintDBG_Buf);} while(0)
#endif

#endif
