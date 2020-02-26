#ifndef _COMMAND_ANALYSIS_H
#define _COMMAND_ANALYSIS_H

#include <Arduino.h>

#define ON						0x00	//
#define OFF						0x01	//

enum Frame_ID {
	Modbus_Control, R_Modbus_Control,Output_default,Work_Para, Set_Group_Num, SN_Area_Channel, Work_Status
	/* , ResetRoll, Opening, Work_Limit*/, Stop_Work , Irrigation_Control
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
	void Query_Current_Work_Param(void);//服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等
	void Set_Group_Number(void);//设置本设备的工作组号，不需要验证本设备原有工作组号
	void Set_SN_Area_Channel(void);//设置本设备的SN码、区域号、设备路数等参数
	void Detailed_Work_Status(void);//查询本设备详细工作状态
	// void ResetRoll_Command(void);//重置卷膜测量行程
	// void Opening_Command(void);//设置卷膜开度
	// void Working_Limit_Command(void);//电机工作电压阈值、上报状态间隔值设置
	void Stop_Work_Command(void);//强制停止当前设备的工作
	void General_controller_control_command(void);//服务器发送通用控制器Modbus控制指令
	void R_General_controller_control_command(void);//服务器发送通用控制器Modbus控制指令接收回执
	void Set_General_controller_output_init(void);//服务器设置通用控制器输出默认状态

	void Irrigation_Controllor_control_command(void);//服务器发送灌溉控制器控制指令(A025)
};



/*Create command analysis project*/
extern Command_Analysis LoRa_Command_Analysis;

extern bool gAccessNetworkFlag;
// extern bool gStopWorkFlag;
extern bool gMassCommandFlag;
extern bool gIsHandleMsgFlag;

//
extern bool Get_receipt;

extern unsigned int Worktime[16]; //16个开启时间
extern unsigned int Worktime_backups[16];//16个开启时间
extern unsigned int WorkInterval[16];//16个间隔时间
extern unsigned int WorkInterval_backups[16];//16个间隔时间的备份
extern unsigned int retryCnt[16];//循环次数（）
extern unsigned int DO_WayOpentime[16];//16路DO的开始时间
extern unsigned int DO_WayClosetime[16];//16路DO的关闭时间
extern unsigned int DO_WayIntervalBengin[16];//16路DO的间隔开始时间
extern unsigned int DO_WayIntervalEnd[16];//16路DO的间隔结束时间
extern bool DO_WayOpen[16];//16路DO打开的标志位
extern bool DO_WayInterval[16];//16路DO间隔标志位
extern bool DO_WayComplete[16];//16路DO完成的标志位
extern bool Irrigation_use;//正在灌溉的标志位
extern bool DO_Set[16];//DO设置的标志位
//extern unsigned int OpenSec;//开启的时间
//extern unsigned int DO_Interval;//单个的间隔时间
//extern unsigned int DO_Num;//一次性开启的DO数量
extern unsigned char Need_Num;//一轮循环需要开启的DO数量
extern unsigned char Complete_Num;//一轮循环完成开启的DO数量
extern unsigned int Cyclic_interval;//循环间隔时间
//extern unsigned char fornum;//一轮循环需要的循环次数，（例如开4路，每次开2路，fornum=2）
//extern unsigned char fornum_backups;//一轮循环需要的循环次数的备份，（例如开4路，每次开2路，fornum=2）
//extern bool Last_full;//最后一轮循环是否能开满，（例如开5路，每次开2路，最后一轮开不满）
//extern unsigned char Last_num;//最后一轮循环开不满时需要开启的个数

#endif
