#ifndef _BOOT_PROTOCOL_H_
#define _BOOT_PROTOCOL_H_

#include "../bt_inc/boot_config.h"

/**
 *The value of the frame header, defined in the protocol is 0xFE.
 */
#define BT_FRAME_HEAD				0xFE

/** 
 *Master slave device flag. 
 *Distinguish whether the data frame is sent from the server or from the device.
 *0 --> slave device;1 --> server;0x55 --> group (server).
 */
#define BT_M_S_DEV_FLAG					0

/**
 *Remote rollback of the flag
 *This bit is filled in by the server, and the embedded device defaults to 0.
 */
#define BT_ROLLBACK_FLAG				0

/** 
 *Date use flag.
 *1 ---> Use date��Fill date in protocol��; 0 ---> Unused date. 
 */
#define BT_DATE_USE_FLAG					0

/**
 *The general frame part consists of the following parts:
 *Frame header, frame data length, frame sequence number, 
 *frame type, receiver/sender device type, master-slave device flag bit, 
 *reserved bit usage length, date enable bit length.
 *
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_GENERAL_CONST_LEN			15

/**
 *The size of the bytes occupied by the remote rollback flag.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_ROLLBACK_FLAG_LEN		1

/**
 *Query the length of bytes in the software version cycle.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define CHECK_VER_PERIOD_LEN		2

/**
 *The length of bytes taken by oftware version
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define SOFTWARE_VER_LEN				3

/**
 *The length of bytes taken by the hardware version
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define HARDWARE_VER_LEN				3

/**
 *The length of bytes taken by the reserved bits.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
 #define RESERVED_LEN					  11

/** 
 *The length of date.
 *The default is 7 bytes, in seconds, month, day, year.
 *example��20 20 03 25 15 21 46
 *Use BCD code representation.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_DATE_LEN							7

/** 
 *The length of CRC16
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_CRC_LEN							2

/** 
 *The length of frame tail 
 *The protocol says it takes six bytes.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_FRAME_END_LEN					6

/**
 *The length of bytes taken by the firmware package.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_DATA_PACK_LEN					2

/**
 *The length of bytes taken by the firmware index.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_DATA_PACK_INDEX_LEN		2

/**
 *The length of bytes taken by the general receipt result.
 *Function is a basis for applying the size of frame data cache of related protocol.
 */
#define BT_REPLY_RESULT_LEN				1


/**
 *The period value of the hardware timer in ms.
 *Default : 1ms.
 *The timer cycle implemented by the developer is actually synchronized with the macro definition.
 */
#define BT_TIMER_PERIOD						1

/**
 *The time allowed to receive data in ms.
 *Default : 500ms
 */
#define BT_TIMEROUT_VALUE					500

/**
 *Specifies the size of the firmware package data received per package, in bytes.
 */
#define BT_DP_SIZE					200

/**
 *Custom FLASH - compliant write data type, default unsigned short.
 *The cache size is typically 8 times larger per packet of data.
 *For example, each packet of data is 128 bytes, and the cache size is 1024 bytes.
 */
typedef unsigned short 						BT_DATAPACK_TYPE;
#define BT_DP_BUF_SIZE						(BT_DP_SIZE * 2)


#define BT_CYCLE_QUERY_RAND_RANGE		5000 //ms
#define BT_SET_RAND_SEED(n)					(srand(BT_Rand_Seed_Num))
#define BT_GET_RAND_VALUE(n)				(rand() % n)

#define BT_OPT_RAND(last, now)  ((now < (last + 500)) && (last < (now + 500)))

/** 
 *Query the length of the device version frame body (excluding the optional reserved space and date)
 */
#define BT_QUERY_DEV_VER_FRAME_MAIN_LEN			(BT_GENERAL_CONST_LEN + BT_CRC_LEN + BT_FRAME_END_LEN)

/**
 *Return receipt device version frame body length (excluding optional reserved space and date)
 */
#define BT_REPLY_DEV_VER_FRAME_MAIN_LEN			(BT_GENERAL_CONST_LEN + BT_ROLLBACK_FLAG_LEN + CHECK_VER_PERIOD_LEN \
																						 + SOFTWARE_VER_LEN + HARDWARE_VER_LEN + BT_CRC_LEN + BT_FRAME_END_LEN)

/** 
 *Gets the length of the packet frame body (excluding the optional reserved bit and date)
 */
#define BT_ACQUIRE_GET_DATA_FRAME_MAIN_LEN		(BT_GENERAL_CONST_LEN + SOFTWARE_VER_LEN + BT_DATA_PACK_LEN \
																							+ BT_DATA_PACK_INDEX_LEN + BT_CRC_LEN + BT_FRAME_END_LEN)
																							
/** 
 *General return frame body length (excluding optional reserved space and date)
 */																							
#define BT_GENERAL_REPLY_FRAME_MAIN_LEN				(BT_GENERAL_CONST_LEN + BT_REPLY_RESULT_LEN + \
																							BT_CRC_LEN + BT_FRAME_END_LEN)
	
enum FRAME_TYPE{
	BT_QUERY_DEV_VER_FRAME = 0xBAA0,	
	BT_REPLY_DEV_VER_FRAME = 0xBAB0,	
	BT_ACQUIRE_DATA_FRAME = 0xBAA1,		
	BT_FORCE_UPDATA_FRAME = 0xBAA2,	
	BT_SEND_DATA_FRAME = 0xBAA3,			
	BT_GENERAL_REPLY_FRAME = 0xBAB1
};

enum BT_ERR_STA{
	BT_OK = 0,
	BT_MEM_ERR = 1,	//Memory application failed.
	BT_RE_ERR,			//Reserved field exceeding protocol size (256bytes).
	BT_CHECK_ERR
};

enum BT_R_RESULT{
	BT_R_OK = 0,	//Successful received.
	BT_R_CRC_ERR = 0x01,	//Check CRC error.
	BT_R_FRAME_TYPE_ERR = 0x02,	//Frame type is not valid.
	BT_R_DEV_TYPE_ERR = 0x03, //Device type is illegal.
	BT_R_DP_ERR = 0x04,	//Firmware package error.
	BT_R_DPL_ERR = 0x05,	//Firmware package length is not valid.
	BT_R_REL_ERR = 0x06,	//It is illegal to reserved length.
	BT_R_DIS_UD = 0x07,	//Reject the request.
	BT_R_FLASH_ERR = 0x08,	//FLASH exception.
	BT_R_DATE_ERR = 0x09,	//Illegal date.
	BT_R_MEM_ERR = 0x0A,	//Memory application failed.
	BT_R_S_VER_ERR = 0x0B	//Software version error was detected on both sides.
};

enum BT_RUN_STA{
	IDLE,
	CP_DP,
	SEND_DATA,
	RCV_DATA,
	PRE_JUMP,
	CYC_QUE_DP,
	CYC_QUE_VER,
	APP_EXP
};

typedef void(*bt_s_h_func)(bt_u8*, bt_u16);

/**
 *Boot-related process structures
 */
struct BT_pcb{

	bt_u8 *psend_frame;			//Sent protocol frame pointer
	bt_u8 *prcv_frame;			//Received protocol frame pointer
	
	BT_DATAPACK_TYPE dp_buf[BT_DP_BUF_SIZE];	//The received firmware package cache array
	
	bt_u16 dp_buf_offset;	//Received firmware package cache offset
	
	bt_u32 dp_w_addr;	//The FLASH address to which the data is currently written
	
	bt_u8 no_app_flag;	//Application exception flag
	
	#if BT_DATE_USE_FLAG
		bt_u8 date_buf[BT_DATE_LEN];	//date
	#endif
	
	bt_u8 f_index_buf[5];	//Frame sequence number used to return to the server for timeout processing
	
	bt_u8 sw_hw_v_buf[4];	//Software version number and hardware version number
	
	bt_u8 cyc_que_sw_start_flag; //Start the loop to query the flag bits
	
	bt_u16 que_v_cyc_val;	//Query version cycle(seconds)

	bt_u16 dp_get_cyc_val;	//firmware get cycle(seconds)
	
	bt_u16 d_pack_index;	//Firmware package current index
	
	bt_u8 d_pack_end_flag;	//Firmware package tail package flag (need to be initialized to 0)
	
	bt_u8 d_pack_en_get_flag;	//Enable to start fetching packet flag
	
	bt_u8 re_buf[RESERVED_LEN];		//Reserved buffer (reserved for developers)
	bt_u8 *prcv_re_buf;			//Received reserved buffer from server.
	
	bt_u8 r_result;				//Receipt processing results
	
	bt_u16 rcv_buf_len;		//Length of received data

	bt_u8 run_sta;				//BT process run status.
	
	bt_s_h_func s_h_func;	//Mount the send data handler

};

void BT_Mount_Dev_And_Init(bt_s_h_func s_h_func);

#if BT_IS_BOOT_ZONE
void BT_APP_Init(void);
#endif

void BT_Cycle_Query_SW_Version(void);

#if USE_TIM_RCV_MSG 
enum BT_ERR_STA  BT_Receive_Protocol_Msg(bt_u8 *rcv_buf, bt_u16 *rcv_len);
#else
enum BT_ERR_STA BT_Receive_Protocol_Msg(bt_u8 *rcv_buf, bt_u16 *rcv_len, bt_u8 *rcvd_flag);
#endif

bt_u8 BT_Get_RunStatus(void);

#if DATE_USE_FLAG
extern unsigned char bt_date_buf[DATE_LEN];
#endif

extern bt_u16 BT_Rcv_Tim_Num;
extern bt_u32 BT_Query_CycVal;

extern bt_u32 BT_Rand_Seed_Num;

#endif
