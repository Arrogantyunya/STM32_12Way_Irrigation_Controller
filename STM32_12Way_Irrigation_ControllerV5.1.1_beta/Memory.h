#ifndef _Memory_H
#define _Memory_H

#include "AT24CXX.h"
#include <libmaple/bkp.h>
#include <Arduino.h>
#include "User_Serial.h"
// #include "boot_config.h"
// #include "boot_mem.h"
// #include "boot_protocol.h"

#define EP_WP_PIN                               PB5

/*使用EEPROM储存芯片的宏定义地址*/
/*EEPROM硬件特性*/
#define EEPROM_MIN_ADDR                         0
#define EEPROM_MAX_ADDR                         255 

/*SN码相关操作保存地址*/
#define SN_OPERATION_FLAG_ADDR                  11
#define SN_BKP_OPERATION_FLAG_ADDR              12
#define SN_BASE_ADDR                            13
#define SN_END_ADDR                             21
#define SN_VERIFY_ADDR                          22
#define SN_BKP_BASE_ADDR                        23
#define SN_BKP_END_ADDR                         31
#define SN_BKP_VERIFY_ADDR                      32
#define SN_ACCESS_NETWORK_FLAG_ADDR             33
/*软件版本和硬件版本保存地址*/
//Software version and hardware version save address.
#define SOFT_VERSION_BASE_ADDR					34
#define HARD_VERSION_BASE_ADDR					36
/*成功重置卷膜总行程标志位保存地址*/
#define ROUTE_FLAG_ADDR                         38
/*卷膜总行程时长保存地址*/
#define ROLL_TIME_HIGH_ADDR                     39
#define ROLL_TIME_LOW_ADDR                      40
#define ROLL_TIME_VERIFY_ADDR                   41
/*电机相关电压、电流、上报频率等参数保存地址*/
#define ROLL_THRESHOLD_VALUE_BASE_ADDR          42 
#define ROLL_THRESHOLD_VALUE_END_ADDR           46
/*卷膜机工作组号相关操作保存地址*/
#define GROUP_NUMBER_BASE_ADDR                  47
#define GROUP_NUMBER_END_ADDR                   51
#define GROUP_NUMBER_VERIFY_ADDR                52
#define GROUP_NUMBER_FLAG_ADDR                  53
/*卷膜机工作区域号相关操作保存地址*/
#define AREA_ADDR                               54
#define AREA_VERIFY_ADDR                        55
#define AREA_FLAG_ADDR                          56
/*卷膜机开棚工作电流保存地址*/
#define ROLL_UP_CURRENT_HIGH_ADDR               57
#define ROLL_UP_CURRENT_LOW_ADDR                58
#define ROLL_UP_CURRENT_VERIFY_ADDR             59
#define SAVE_UP_CURRENT_FLAG_ADDR               60
/*卷膜机关棚工作电流保存地址*/
#define ROLL_DOWN_CURRENT_HIGH_ADDR             61
#define ROLL_DOWN_CURRENT_LOW_ADDR              62
#define ROLL_DOWN_CURRENT_VERIFY_ADDR           63
#define SAVE_DOWN_CURRENT_FLAG_ADDR             64
/*卷膜机上一次开度值*/
#define EP_MOTOR_LAST_OPENING_ADDR              65
#define EP_MOTOR_LAST_OPENING_CRC_ADDR          66
/*卷膜机本次开度值*/
#define EP_MOTOR_RECENT_OPENING_ADDR            67
#define EP_MOTOR_RECENT_OPENING_CRC_ADDR        68
/*卷膜机实时开度值*/
#define EP_MOTOR_REALTIME_OPENING_ADDR          69
#define EP_MOTOR_REALTIME_OPENING_CRC_ADDR      70
/*卷膜相关阈值参数保存标志位保存地址*/
#define SAVE_ROLL_THRESHOLD_FLAG_ADDR           71
/*配置LoRa参数完成标志位保存地址*/
#define LORA_PARA_CONFIG_FLAG_ADDR              72  
/*卷膜工作电机电压保存地址*/
#define ROLL_VOLTAGE_HIGH_ADDR                  73            
#define ROLL_VOLTAGE_LOW_ADDR                   74
#define ROLL_VOLTAGE_VERIFY_ADDR                75

/* LoRa通信模式配置成网关还是节点 0xF0节点; 0xF1网关 */
#define LORA_COM_MODE_ADDR                      76
#define LORA_COM_MODE_FLAG_ADDR                 77
#define LORA_COM_MODE_VERIFY_ADDR               78

#define EP_LORA_ADDR_BASE_ADDR                  79
#define EP_LORA_ADDR_END_ADDR                   86
#define EP_LORA_ADDR_VERIFY_ADDR                87
#define EP_LORA_ADDR_SAVED_FLAG_ADDR            88

/* Modbus控制器的初始状态存储 */
#define INIT_STATE_FLAG_ADDR					89
#define DO_INIT_STATE_BASE_ADDR					90
#define DO_INIT_STATE_END_ADDR					97
#define DO_INIT_STATE_VERIFY_ADDR				98
#define AO_INIT_STATE_BASE_ADDR					99
#define AO_INIT_STATE_END_ADDR					114
#define AO_INIT_STATE_VERIFY_ADDR				115

/*超时时间以及其他东西的存储*/
#define Timeout_BASE_ADDR						116
#define Timeout_END_ADDR						117
#define E000Interval_BASE_ADDR					118
#define E000Interval_END_ADDR					119
#define WorkInterval_BASE_ADDR					120
#define WorkInterval_END_ADDR					121
#define StopInterval_BASE_ADDR					122
#define StopInterval_END_ADDR					123
#define Cyclic_interval_BASE_ADDR				124
#define Cyclic_interval_END_ADDR				125

/* LORA的部分参数设置 */
#define Lora_TRMode_ADDR                        126
#define Lora_TRMode_Set_ADDR                    127
#define LORA_SYNC_ADDR                          128
#define Lora_SYNC_Set_ADDR                      129

/* 正反转模式相关存储 */
#define AI_Relation_Way_BASE_ADDR				130//AI关联路数地址（1*6）
#define AI_Relation_Way_END_ADDR				135

#define Stop_AI_BASE_ADDR						136//静止状态AI（2*6）
#define Stop_AI_END_ADDR						147

#define Forward_AI_BASE_ADDR					148//正转AI（2*6）
#define Forward_AI_END_ADDR						159

#define Reversal_AI_BASE_ADDR					160//反转AI（2*6）
#define Reversal_AI_END_ADDR					171	

#define Forward_Time_BASE_ADDR					172//正转时间（3*6）
#define Forward_Time_END_ADDR					189

#define Reversal_Time_BASE_ADDR					190//反转时间（3*6）
#define Reversal_Time_END_ADDR					207

#define Threshold_multiple_BASE_ADDR			208//阈值倍数存储(1*6)
#define Threshold_multiple_END_ADDR				213

#define A009_Seted								214//AI关联以及阈值倍数被设置
#define A00A_Seted								215//（1*6）静止状态、正传、反转AI以及正传、反转时间被设置



/*使用芯片自带备份寄存器的宏定义地址*/
/*上一次开度值保存地址（0 - 100）*/
#define BKP_MOTOR_LAST_OPENING_ADDR             1
/*上一次开度值CRC8保存地址*/
#define BKP_MOTOR_LAST_OPENING_CRC_ADDR         2
/*本次开度值保存地址（0 - 100）*/
#define BKP_MOTOR_RECENT_OPENING_ADDR           3
/*本次开度值CRC8保存地址*/
#define BKP_MOTOR_RECENT_OPENING_CRC_ADDR       4
/*实时开度值保存地址*/
#define BKP_MOTOR_REALTIME_OPENING_ADDR         5 
/*实时开度值CRC8保存地址*/
#define BKP_MOTOR_REALTIME_OPENING_CRC_ADDR     6   

extern String comdata;

/*
 @brief     : 上拉该引脚，禁止EEPROM写操作
 @para      : 无
 @return    : 无
 */
inline void EEPROM_Write_Disable(void)
{
	digitalWrite(EP_WP_PIN, HIGH);
}

/*
 @brief     : 下拉该引脚，允许EEPROM写操作
 @para      : 无
 @return    : 无
 */
inline void EEPROM_Write_Enable(void)
{
	digitalWrite(EP_WP_PIN, LOW);
}

class EEPROM_Operations : protected AT24Cxx {
public:
	void EEPROM_GPIO_Config(void);
	void EEPROM_Reset(void);//将EP的值重置
};

class SN_Operations : public EEPROM_Operations {
public:
	bool Save_SN_Code(unsigned char *sn_code);
	bool Save_BKP_SN_Code(unsigned char *sn_code);
	bool Read_SN_Code(unsigned char *sn_code);
	bool Read_BKP_SN_Code(unsigned char *sn_code);
	// bool Verify_Save_SN_Code(void);
	// bool Verify_Save_BKP_SN_Code(void);
	bool Clear_SN_Save_Flag(void);
	bool Clear_BKP_SN_Save_Flag(void);
	bool Set_SN_Access_Network_Flag(void);
	bool Clear_SN_Access_Network_Flag(void);
	bool Verify_SN_Access_Network_Flag(void);
	bool Self_check(unsigned char *dat);

    bool Save_Area_Number(unsigned char area_num);
	unsigned char Read_Area_Number(void);
	bool Check_Area_Number(void);
	bool Verify_Area_Number_Flag(void);
	bool Clear_Area_Number(void);

    bool Save_Group_Number(unsigned char *group_num);
	bool Read_Group_Number(unsigned char *group_num);
	bool Check_Group_Number(void);
	bool Verify_Group_Number_Flag(void);
	bool Clear_Group_Number(void);

	void Read_Random_Seed(unsigned char *random_seed);

    //没用的
    unsigned char Read_RealTime_Opening_Value(void);//没用的
    unsigned char Read_Roll_High_Current_Limit_Value(void);//没用的
    unsigned char Read_Roll_Low_Voltage_Limit_Value(void);//没用的
    unsigned char Read_Roll_Report_Status_Interval_Value(void);
};

class LoRa_Config : public EEPROM_Operations {
public:
	bool Save_LoRa_Config_Flag(void);
	bool Verify_LoRa_Config_Flag(void);
	bool Clear_LoRa_Config_Flag(void);

	bool Save_LoRa_Com_Mode_Flag(void);
	bool Clear_LoRa_Com_Mode_Flag(void);
	bool Save_LoRa_Com_Mode(unsigned char mode);
	unsigned char Read_LoRa_Com_Mode(void);

	void Clear_LoRa_Addr_Flag(void);
	void Save_LoRa_Addr_Flag(void);
	bool Verify_LoRa_Addr_Flag(void);
	bool Read_LoRa_Addr(unsigned char *addr);
	bool Save_LoRa_Addr(unsigned char *addr);

	bool Verify_LoRa_TRMode_Flag(void);//验证Lora的TRmode是否被配置过
    bool Save_LoRa_TRMode(unsigned char TRMode);//保存TRMode
    unsigned char Read_LoRa_TRMode(void);//读取TRMode

    bool Verify_LoRa_SYNC_Flag(void);//验证Lora的SYNC是否被配置过
    bool Save_LoRa_SYNC(unsigned char SYNC);//保存SYNC
    unsigned char Read_LoRa_SYNC(void);//读取SYNC
};

class Soft_Hard_Vertion : public EEPROM_Operations {
public:
	void Save_hardware_version(unsigned char number_high, unsigned char number_low);
	void Save_Software_version(unsigned char number_high, unsigned char number_low);
	unsigned char Read_hardware_version(unsigned char number_addr);
	unsigned char Read_Software_version(unsigned char number_addr);
};

class ModbusController_InitState : public EEPROM_Operations{
private:
	/* data */
public:
	bool Save_InitState_Flag(void);//存储初始状态的标志位
	bool Read_InitState_Flag(void);//读取初始状态的标志位
	bool Clean_InitState_Flag(void);//清除初始状态的标志位

	bool Save_DO_InitState(unsigned char *data);//存储DO初始状态
	unsigned char* Read_DO_InitState(void);//读取DO初始状态
	bool Clean_DO_InitState();//清除DO初始状态

	bool Save_AO_InitState(unsigned char *data);//存储AO初始状态
	bool Read_AO_InitState(unsigned char *data);//读取AO初始状态
	bool Clean_AO_InitState();//清除AO初始状态

	bool Save_Timeout(unsigned char High, unsigned char Low);//存储timeout时间
	unsigned int Read_Timeout(void);//读取timeout时间
	bool Clean_Timeout(void);//清除timeout时间

	bool Save_E000Interval(unsigned char High, unsigned char Low);//存储E000Interval时间
	unsigned int Read_E000Interval(void);//读取E000Interval时间
	bool Clean_E000Interval(void);//清除E000Interval时间

	bool Save_StopInterval(unsigned char High, unsigned char Low);//存储StopInterval时间
	unsigned int Read_StopInterval(void);//读取StopInterval时间
	bool Clean_StopInterval(void);//清除StopInterval时间

	bool Save_WorkInterval(unsigned char High, unsigned char Low);//存储WorkInterval时间
	unsigned int Read_WorkInterval(void);//读取WorkInterval时间
	bool Clean_WorkInterval(void);//清除WorkInterval时间

	bool Save_CyclicInterval(unsigned char High, unsigned char Low);//保存循环间隔时间
	unsigned int Read_CyclicInterval();//读取循环间隔时间
	bool Clean_CyclicInterval(void);//清除循环间隔时间
};

class Positive_Negative_MODE : public EEPROM_Operations{
private:
	/* data */
public:
	bool Save_AI_Relation_Way(unsigned char* data);//存储AI关联路数
	unsigned char Read_AI_Relation_Way(unsigned char which_Way);//读取AI关联路数
	bool Clean_AI_Relation_Way(void);//清除AI关联路数

	bool Save_Stop_AI(unsigned short data,unsigned char which_Way);//存储静止状态AI
	unsigned short Read_Stop_AI(unsigned char which_Way);//读取静止状态AI
	bool Clean_Stop_AI(unsigned char which_Way);//清除静止状态AI

	bool Save_Forward_AI(unsigned short data,unsigned char which_Way);//存储正转AI
	unsigned short Read_Forward_AI(unsigned char which_Way);//读取正转AI
	bool Clean_Forward_AI(unsigned char which_Way);//清除正转AI

	bool Save_Reversal_AI(unsigned short data,unsigned char which_Way);//存储反转AI
	unsigned short Read_Reversal_AI(unsigned char which_Way);//读取反转AI
	bool Clean_Reversal_AI(unsigned char which_Way);//清除反转AI

	bool Save_Forward_Time(unsigned int data,unsigned char which_Way);//存储正转时间
	unsigned int Read_Forward_Time(unsigned char which_Way);//读取正转时间
	bool Clean_Forward_Time(unsigned char which_Way);//清除正转时间

	bool Save_Reversal_Time(unsigned int data,unsigned char which_Way);//存储反转时间
	unsigned int Read_Reversal_Time(unsigned char which_Way);//读取反转时间
	bool Clean_Reversal_Time(unsigned char which_Way);//清除反转时间

	bool Save_Threshold_multiple(unsigned char* data);//阈值倍数
	unsigned char Read_Threshold_multiple(unsigned char which_Way);//阈值倍数
	bool Clean_Threshold_multiple(void);//阈值倍数

	bool Save_A009_Seted(void);//保存AI关联以及阈值倍数被设置
	bool Read_A009_Seted(void);//读取AI关联以及阈值倍数被设置
	bool Clean_A009_Seted(void);//清除AI关联以及阈值倍数被设置

	bool Save_A00A_Seted(unsigned char which_Way);//保存静止状态、正传、反转AI以及正传、反转时间被设置
	bool Read_A00A_Seted(unsigned char which_Way);//读取静止状态、正传、反转AI以及正传、反转时间被设置
	bool Clean_A00A_Seted(unsigned char which_Way);//清除静止状态、正传、反转AI以及正传、反转时间被设置
};



/*Create EEPROM object*/
extern EEPROM_Operations EEPROM_Operation;
/*Create SN code operation object*/
extern SN_Operations SN;
/*Create LoRa Config object*/
extern LoRa_Config LoRa_Para_Config;
/*Create software and hardware object*/
extern Soft_Hard_Vertion Vertion;
/*Create ModbusController_InitState object*/
extern ModbusController_InitState InitState;
/*Create Positive_Negative_MODE object*/
extern Positive_Negative_MODE Pos_Nega_mode;

#endif
