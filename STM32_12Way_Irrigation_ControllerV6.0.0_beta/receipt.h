#ifndef _RECEIPT_H
#define _RECEIPT_H

#include <Arduino.h>
#include "film_config.h"
#include "film_mem.h"
#include "film.h"

#define DEVICE_TYPE_ID  0xC003

//#define SOFT_VERSION    0x14 //V2.0
//
//#define HARD_VERSION    0x53//V8.3

enum E014_ReceiptStatus
{
	DefaultValue = 0, Failed_save_E000interval, Modbuspacket_length_overflow, A000_Received_Success
}/*E014_ReceiptState*/;

enum ReceiptStatus {
	FactoryMode = 0, AskUploadParamsOk, AskUploadParamsErr, AssignGroupIdArrayOk, AssignGroupIdArrayErr, SetSnAndSlaverCountOk,
	SetSnAndSlaverCountErr, TrunOffOk, TrunOffErr, RestRollerOk, ResetRollerErr, OpenRollerOk, OpenRollerErr, LimitRollerOk,
	LimitRollerErr, SetLoRaModeOk, SetLoRaModeErr,SetLoRaTFREQOk,SetLoRaTFREQErr,SetLoRaRFREQOk,SetLoRaRFREQErr,
	SetIntervalOK,SetIntervalErr,SetRTCOK,SetRTCErr
};

enum E009_Status{
	Illegal_AI_relation = 0,Illegal_threshold_multiple,E009_Success
};

enum E00A_Status{
	A009_Seted_ERR = 0,Begin_Calculate_travel,complete_Calculate_travel
};

enum EOOB_Status{
	Auto_Report = 0 ,Opening_SetOK ,Opening_SetErr 
};

enum E00C_Status{
	Way0_Over_threshold = 0,Way1_Over_threshold,Way2_Over_threshold,Way3_Over_threshold,Way4_Over_threshold,
	Way5_Over_threshold,Way6_Over_threshold
};

// /*电机状态*/
// enum MotorStatus {
// 	MotorFactoryMode = 0, ROLL_OK, HIGH_POSITION_LIMIT_EXCEPTION, LOW_POSITION_LIMIT_EXCEPTION, LOW_POWER, MOTOR_EXCEPTION, MOTOR_CURRENT_EXCEPTION,
// 	ROLLING, CMD_EXCEPTION, NOT_INITIALIZED, STORE_EXCEPTION, RESET_ROLLING, RESET_ROLLOK, RS485_EXCEPTION, FORCE_STOP, MANUAL_ROLL_OK, MANUAL_KEY_EXCEPTION,
// 	OPEN_ROLL_ERROR
// };

// extern unsigned char gMotorStatus;

// /*
//  @brief   : 设置当前设备工作状态
//  @para    : 设备状态
//  @return  : 无
// */
// inline void Set_Motor_Status(unsigned char status) { gMotorStatus = status; }

// /*
//  @brief   : 读取当前设备工作状态
//  @para    : None
//  @return  : 设备状态
// */
// inline unsigned char Read_Motor_Status(void) { return (gMotorStatus); }

class Receipt {
public:
	void Report_General_Parameter(void);//E011上报本设备通用设置参数
	void Request_Set_Group_Number(void);//E012当本地工作组号丢失，向服务器申请本机的工作组号
	void Request_Device_SN_and_Channel(void);//E013当本地SN码丢失，向服务器申请本机的SN码
	void Working_Parameter_Receipt(bool use_random_wait, unsigned char times);//E014上报实时详细工作状态
	void General_Receipt(unsigned char status, unsigned char send_times);//E015发送通用回执信息给服务器
	void Control_command_Receipt(bool use_random_wait, unsigned char send_times);//E000发送通用控制器Modbus控制指令回执信息给服务器
	// void Irrigation_loop_Receipt(bool use_random_wait, unsigned char times, unsigned char randomId_1, unsigned char randomId_2);//E001发送灌溉循环状态回执信息给服务器
	void OnLine_Receipt(bool use_random_wait, unsigned char times);//E002发送上线状态回执信息给服务器
	void Irrigation_control_Receipt(unsigned char send_times, unsigned char* gReceiveCmd);//E003发送灌溉控制回执信息给服务器
	void Delay_Start_DO_Control_Receipt(unsigned char send_times, unsigned char* gReceiveCmd);//E004发送延时开启回执信息给服务器
	void Positive_negative_Control_Receipt(unsigned char send_times, unsigned char* gReceiveCmd);//E005发送正反转回执信息给服务器
	void New_Working_Parameter_Receipt(bool use_random_wait,unsigned char send_times);//E006新实时状态回执
	void SignalQuality_Version_Receipt(bool use_random_wait,unsigned char send_times);//E007信号质量与版本号回执
	void Set_threshold_Receipt(unsigned char send_times,unsigned char* gReceiveCmd,unsigned char E009status);//E009设置正反转模式阈值回执
	void Calculate_travel_Receipt(unsigned char send_times, unsigned char WayUsed, unsigned char E00Astatus);//E00A计算行程与正反转AI回执
	void Opening_Control_Receipt(unsigned char send_times, unsigned char E00Bstatus);//EOOB开度控制回执
	void Rolling_ch_Status_Receipt(unsigned char send_times,unsigned char ch, unsigned char E00Cstatus);//E00C卷膜路数状态回执
	
	void Output_init_Receipt(unsigned char status, unsigned char send_times);//E002发送设置初始状态回执信息给服务器
private:
	void Receipt_Random_Wait_Value(unsigned long int *random_value);//随机生成回执消息的回执发送微秒延时
	void Clear_Server_LoRa_Buffer(void);//清除服务器上一次接收的LoRa数据缓存
	void Print_Debug(unsigned char *base_addr, unsigned char len);//串口打印16进制回执信息
};

/* 卷膜流程中的状态信息投递任务，开发者根据传入的状态来提供具体事项，如将状态上报给服务器 */
void Film_Status_Msg_Delivery_Task(film_u8 ch, film_m_sta sta);

extern Receipt Message_Receipt;
extern unsigned char gStatus_E014;
extern volatile bool gStateReportFlag;
extern unsigned char gLoRaCSQ[4];  //接收LoRa发送和接收的信号强度


#endif
