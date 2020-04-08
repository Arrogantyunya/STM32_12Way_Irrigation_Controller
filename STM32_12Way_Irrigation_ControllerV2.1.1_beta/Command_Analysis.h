#ifndef _COMMAND_ANALYSIS_H
#define _COMMAND_ANALYSIS_H

#include <Arduino.h>

#define ON						0x00	//
#define OFF						0x01	//

enum Frame_ID {
	Modbus_Control, R_Modbus_Control,Output_default,Work_Para, Set_Group_Num, SN_Area_Channel, Work_Status
	, /* ResetRoll, Opening, Work_Limit, */ Stop_Work , Irrigation_Control,Non_existent
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
	void Query_Current_Work_Param(void);//?????????????????????????????????SN??��?????????�??
	void Set_Group_Number(void);//??????��????????????????????��??��??????
	void Set_SN_Area_Channel(void);//??????��??SN?????????����???????
	void Detailed_Work_Status(void);//??????��?????????
	// void ResetRoll_Command(void);//???????????��?
	// void Opening_Command(void);//??????????
	// void Working_Limit_Command(void);//????????????????????????????
	void Stop_Work_Command(void);//?????????��?????
	void General_controller_control_command(void);//??????????????????Modbus???????
	void R_General_controller_control_command(void);//??????????????????Modbus????????????
	void Set_General_controller_output_init(void);//??????????????????????????

	void Irrigation_Controllor_control_command(void);//????????????????????????(A025)
};



/*Create command analysis project*/
extern Command_Analysis LoRa_Command_Analysis;

extern bool gAccessNetworkFlag;
// extern bool gStopWorkFlag;
extern bool gMassCommandFlag;
extern bool gIsHandleMsgFlag;

//
extern bool Get_receipt;
extern bool Begin_AUTOReceipt;

extern unsigned int Worktime[16]; //16?????????
extern unsigned int Worktime_backups[16];//16?????????
extern unsigned int WorkInterval[16];//16????????
extern unsigned int WorkInterval_backups[16];//16????????????
extern unsigned int retryCnt[16];//???????????
extern unsigned int DO_WayOpentime[16];//16��DO???????
extern unsigned int DO_WayClosetime[16];//16��DO???????
extern unsigned int DO_WayIntervalBengin[16];//16��DO??????????
extern unsigned int DO_WayIntervalEnd[16];//16��DO???????????
extern bool DO_WayOpen[16];//16��DO?????��
extern bool DO_WayInterval[16];//16��DO??????��
extern bool DO_WayComplete[16];//16��DO??????��
extern bool Irrigation_use;//?????????��
extern bool DO_Set[16];//DO???????��
//extern unsigned int OpenSec;//?????????
//extern unsigned int DO_Interval;//???????????
//extern unsigned int DO_Num;//??????????DO????
extern unsigned char Need_Num;//???????????????DO????
extern unsigned char Complete_Num;//??????????????DO????
extern unsigned int Cyclic_interval;//?????????
//extern unsigned char fornum;//?????????????????????????�C4��????��?2��??fornum=2??
//extern unsigned char fornum_backups;//?????????????????????????????�C4��????��?2��??fornum=2??
//extern bool Last_full;//????????????????????????�C5��????��?2��???????????????
//extern unsigned char Last_num;//????????????????????????????

#endif
