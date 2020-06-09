#ifndef _COMMAND_ANALYSIS_H
#define _COMMAND_ANALYSIS_H

#include <Arduino.h>
// #include "boot_config.h"
// #include "boot_mem.h"
// #include "boot_protocol.h"

#define ON						0x00	//
#define OFF						0x01	//
#define Solenoid_mode			0x00	//定时DO模式（电磁阀专属）
#define Delay_mode				0x01	//延时DO模式（风机、拉幕专属）
#define Forward_Reverse_mode	0x02	//正反转模式（卷膜、拉幕专属）

enum Frame_ID {
	Modbus_Control, R_Modbus_Control,Output_default,Work_Para, Set_Group_Num, SN_Area_Channel, Work_Status,
	/*ResetRoll, Opening, Work_Limit,*/ Stop_Work,Irrigation_Control,Delay_Start_DO_Control,
	Positive_negative_Control,Query_SignalQuality_Version,Set_Reporting_Interval,Set_Lora_parameter,Non_existent,
	Set_threshold,Calculate_travel,Opening_Control
};

class Command_Analysis {
public:
	void Receive_LoRa_Cmd(void);

private:
	Frame_ID FrameID_Analysis(void);
	bool Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len);
	bool Verify_Device_Type_ID(void);
	void Verify_Mass_Commands(void);
	bool Verify_Area_Number(void);
	bool Verify_Work_Group(void);
	bool Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag, bool group_flag);

private:
	void Receive_Data_Analysis(void);
	void Query_Current_Work_Param(void);//服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等(A011)
	void Set_Group_Number(void);//设置本设备的工作组号，不需要验证本设备原有工作组号(A012)
	void Set_SN_Area_Channel(void);//设置本设备的SN码、区域号、设备路数等参数(A013)
	void Detailed_Work_Status(void);//查询本设备详细工作状态(A014)
	// void ResetRoll_Command(void);//重置卷膜测量行程()
	// void Opening_Command(void);//设置卷膜开度()
	// void Working_Limit_Command(void);//电机工作电压阈值、上报状态间隔值设置()
	void Stop_Work_Command(void);//强制停止当前设备的工作(A015)
	void General_controller_control_command(void);//服务器发送通用控制器Modbus控制指令(A000)
	void Set_RTC_command(void);//服务器发送设置RTC指令(A001)
	void Set_General_controller_output_init(void);//服务器设置通用控制器输出默认状态(A002)

	void Irrigation_Controllor_control_command(void);//服务器发送灌溉控制器控制指令(A003)
	void Delay_Start_DO_Control_command();//服务器发送延时DO指令（A004）
	void Positive_negative_Control_command();//服务器发送正反转控制指令(A005)
	void Query_SignalQuality_Version_command();//服务器查询信号质量与版本号(A006)
	void Set_Reporting_Interval_command();//服务器设置上报时间间隔(A007)
	void Set_Lora_parameter_command();//服务器设置LORA参数(A008)
	void Set_Forward_Reverse_mode_threshold();//服务器设置正反转模式阈值（A009）
	void Forward_Reverse_mode_Calculate_travel();//服务器发送正反转模式计算行程（A00A）
	void Forward_Reverse_mode_Opening_Control();//服务器发送正反转模式开度控制指令（A00B）

	void Forced_Stop(bool Need_Receipt);//强制停止
};



/*Create command analysis project*/
extern Command_Analysis LoRa_Command_Analysis;
extern unsigned char Device_Mode;//
// extern Mode Device_Mode; 

extern bool gAccessNetworkFlag;
// extern bool gStopWorkFlag;
extern bool gMassCommandFlag;
extern bool gIsHandleMsgFlag;

extern bool Enter_Work_State;//開始自動上報

/* 通用相关 */
extern unsigned int Worktime[16]; //16个开启时间
extern unsigned int Worktime_backups[16];//16个开启时间的备份
extern unsigned int WorkInterval[16];//16个间隔时间
extern unsigned int WorkInterval_backups[16];//16个间隔时间的备份
extern unsigned int retryCnt[16];//16个DO的循环次数
extern unsigned int DO_WayOpentime[16];//16路DO的开始时间
extern unsigned int DO_WayClosetime[16];//16路DO的关闭时间
extern unsigned int DO_WayIntervalBengin[16];//16路DO的间隔开始时间
extern unsigned int DO_WayIntervalEnd[16];//16路DO的间隔结束时间
extern bool DO_WayOpen[16];//16路DO打开的标志位
extern bool DO_WayInterval[16];//16路DO间隔标志位
extern bool DO_WayComplete[16];//16路DO完成的标志位
extern bool DO_Set[16];//DO设置的标志位
extern unsigned char Need_Num;//一轮循环需要开启的DO数量
extern unsigned char Complete_Num;//一轮循环完成开启的DO数量
extern unsigned int Cyclic_interval;//循环间隔时间
extern bool Waiting_Collection;//等待采集标志

/* 电磁阀模式相关 */
extern bool Irrigation_use;//正在灌溉的标志位

/* 延时模式相关 */
extern unsigned int Delay_mode_OpenSec;//延时模式的开启时间
extern unsigned char Delay_mode_Interval;//延时模式的间隔时间
extern unsigned char Delay_mode_DONum;//延时模式的一次开启数量
extern unsigned int Delay_mode_Worktime[16];//16个开启时间
extern bool Delay_mode_NeedWait;//需要进行延时标志位
extern unsigned char Delay_mode_OpenNum;//开启的数量
extern unsigned int Delay_mode_EndWait;//结束等待的时间

/* 正反转模式相关 */
extern unsigned int Forward_Reverse_mode_Worktime[12];//4个开启时间
extern unsigned char Forward_Reverse_mode_Interval;//正反转模式的间隔时间
extern bool Forward_Reverse_mode_NeedWait;//需要进行延时标志位
extern unsigned int Forward_Reverse_mode_EndWait;//结束等待的时间

/* 重置行程相关 */
extern unsigned char A00A_WayUsed;//
// extern bool Calculate_travel_Flag;//需要进行重置行程的标志
extern bool A00A_WayUsed_Array[8];//需要进行重置行程的路数

#endif
