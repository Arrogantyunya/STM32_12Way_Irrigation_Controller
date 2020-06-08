#ifndef _FILM_CONFIG_H
#define _FILM_CONFIG_H

#include <Arduino.h>
#include <stdio.h>

typedef unsigned char   film_u8;
typedef unsigned short  film_u16;
typedef unsigned int   film_u32;

typedef char            film_8;
typedef short           film_16;
typedef int             film_32;
typedef float           film_float;
typedef double          film_double;

#define _film_(weak) __attribute__((weak))

typedef unsigned char   film_mem_u8;  //选择储存器储存数据规格类型
#define FILM_MEM_TYPE   film_mem_u8


#define MOTOR_CHANNEL   1U //设置总共控制多少路电机卷膜


#define CURRENT_SMOOT_TIMES                 11  /* 电流滤波次数 */

#define PRINT_DBG_LEN     500
extern film_8 FM_PrintDBG_Buf[PRINT_DBG_LEN];
#define FM_PRINT(x) do { sprintf x; Serial.print(FM_PrintDBG_Buf);}while(0)

/* Type information debugging */
#define NO_DBG_ALL			0 //Mask all types
#define BT_CB_DBG           1 //Callback function debugging.

#define FILM_DEBUGF(debug, message) do { if ((debug & !NO_DBG_ALL)){FM_PRINT(message);}} while(0)

#endif
