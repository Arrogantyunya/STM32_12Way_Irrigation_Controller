/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/8/27
 *
 * 该文件的作用是接收服务器通过LoRa无线模块发送过来的指令数据，然后解析这些指令。指令包括通用指令
 * （绑定SN，设置区域号，设置工作组号，查询本机状态等），私有指令（卷膜机重置行程，设置开度卷膜
 * 设置卷膜工作阈值等）。接收的指令都要校验CRC8，有些指令要校验区域号或工作组号本机才会执行相应
 * 功能。
 * 头文件中提供了各个类的公共接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Command_Analysis.h"
#include <libmaple/iwdg.h>
#include "LoRa.h"
#include "User_CRC8.h"
#include "Memory.h"
#include "receipt.h"
#include "fun_periph.h"
#include "Set_coil.h"
#include "Private_Timer.h"
#include "Private_RTC.h"
//#include "ModbusSerial.h"


Command_Analysis LoRa_Command_Analysis;

unsigned char Device_Mode;//设备的工作模式

unsigned char gReceiveCmd[300];   //接收LoRa数据缓存
unsigned short gReceiveLength;     //接收LoRa数据长度
bool gAccessNetworkFlag = true;   //是否已经注册到服务器标志位
// bool gStopWorkFlag = false;       //是否要强制停止标志位
bool gMassCommandFlag = false;    //接收的消息是否是群发标志位

bool gIsHandleMsgFlag = true;     //是否接收到LoRa消息然后解析处理，还是只接收不解析处理（刚上电LoRa模块发的厂家信息）
bool Enter_Work_State = false;//開始自動上報
unsigned char randomId_1;	//
unsigned char randomId_2;

/* 通用相关 */
unsigned int Worktime[16];//16个开启时间
unsigned int Worktime_backups[16];//16个开启时间的备份
unsigned int WorkInterval[16];//16个间隔时间
unsigned int WorkInterval_backups[16];//16个间隔时间的备份
unsigned int retryCnt[16];//16个DO的循环次数
unsigned int DO_WayOpentime[16];//16路DO的开始时间
unsigned int DO_WayClosetime[16];//16路DO的关闭时间
unsigned int DO_WayIntervalBengin[16];//16路DO的间隔开始时间
unsigned int DO_WayIntervalEnd[16];//16路DO的间隔结束时间
bool DO_WayOpen[16];//16路DO打开的标志位
bool DO_WayInterval[16];//16路DO间隔标志位
bool DO_WayComplete[16];//16路DO完成的标志位
bool DO_Set[16] = {true};//DO设置的标志位
unsigned char Need_Num = 0;//一轮循环需要开启的DO数量
unsigned char Complete_Num = 0;//一轮循环完成开启的DO数量
unsigned int Cyclic_interval = 0;//循环间隔时间
bool Waiting_Collection = false;//等待采集标志

/* 电磁阀模式相关 */
bool Irrigation_use = false;//正在灌溉的标志位

/* 延时模式相关 */
unsigned int Delay_mode_OpenSec;//延时模式的开启时间
unsigned char Delay_mode_Interval;//延时模式的间隔时间
unsigned char Delay_mode_DONum;//延时模式的一次开启数量
unsigned int Delay_mode_Worktime[16];//16个开启时间
bool Delay_mode_NeedWait = false;//需要进行延时标志位
unsigned char Delay_mode_OpenNum;//开启的数量
unsigned int Delay_mode_EndWait;//结束等待的时间

/* 正反转模式相关 */
unsigned int Forward_Reverse_mode_Worktime[12];//6个开启时间
unsigned char Forward_Reverse_mode_Interval;//正反转模式的间隔时间
bool Forward_Reverse_mode_NeedWait =false;//需要进行延时标志位
unsigned int Forward_Reverse_mode_EndWait;//结束等待的时间

extern bool DOStatus_Change;//DO的状态改变标志位
extern bool One_Cycle_Complete;//一轮循环完成的标志位
extern bool Cyclic_intervalFlag;//循环时间间隔的标志位
extern bool Cyclic_timing;//正在进行循环时间间隔的标志位
extern bool DO_intervalFlag;//单个间隔时间的标志位
extern bool DO_interval_timing;//正在进行单个间隔时间的标志位
extern bool gRTCTime_arrive_Flag;//RTC时间到达的标志位
extern unsigned int KeyDI7_num;//按键DI7按下的次数

/* 重置行程相关 */
unsigned char A00A_WayUsed = 0x00;//
bool A00A_WayUsed_Array[8] = {0x00};//需要进行重置行程的路数
bool Calculate_travel_Flag = false;//需要进行重置行程的标志

/* 远程升级 */
unsigned char gRcvOtherFlag = 0;

/*
 @brief   : 从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
            每条指令以0xFE为帧头，0x0D 0x0A 0x0D 0x0A 0x0D 0x0A，6个字节为帧尾。最大接受指令长度为128字节，超过将清空接收的数据和长度。
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Receive_LoRa_Cmd(void)
{
  unsigned char EndNum = 0;  //帧尾数量计数值
  unsigned char FrameHeadDex = 0;
  bool NoFrameHeadFlag = true;
  unsigned long Rcv_OverTime;

  gReceiveLength = 0;
  iwdg_feed();
  Rcv_OverTime = millis();

  while ((LoRa_Serial.available() > 0) && (millis() < (Rcv_OverTime + 3000)))
  {
    gReceiveCmd[gReceiveLength++] = LoRa_Serial.read();
    Serial.print(gReceiveCmd[gReceiveLength - 1], HEX);
    Serial.print(" ");
    /* 如果缓存里没有数据，等待一会儿 */
    if (LoRa_Serial.available() <= 0) delay(4);
    /*防止越界*/
    if (gReceiveLength >= 300)  gReceiveLength = 0;

    //记录帧头所在接收的数据包里的位置（因为噪音的干扰，第一个字节可能不是帧头）
    if (NoFrameHeadFlag && gReceiveCmd[gReceiveLength - 1] == 0xFE)
    {
      FrameHeadDex = gReceiveLength - 1;
      NoFrameHeadFlag = false;
    }

    if (!NoFrameHeadFlag)
    {
      /*验证帧尾: 0D 0A 0D 0A 0D 0A*/
      if (gReceiveCmd[gReceiveLength - 1] == 0x0D && (EndNum % 2 == 0))  //如果检测到第一个帧尾
        EndNum += 1;
      else if (gReceiveCmd[gReceiveLength - 1] == 0x0D && (EndNum % 2 == 1))  //防止校验位是0x0D的情况下取不到帧尾
        EndNum = 1;
      else if (gReceiveCmd[gReceiveLength - 1] == 0x0A && (EndNum % 2 == 1))
        EndNum += 1;
      else
        EndNum = 0;
	  if (EndNum == 6) break;
    }
  }
  iwdg_feed();

  if (EndNum == 6)  //帧尾校验正确
  {
    EndNum = 0;
    //Serial.println("Get frame end... <Receive_LoRa_Cmd>");

    Serial.println("Parsing LoRa command... <Receive_LoRa_Cmd>");

    if (FrameHeadDex != 0)  //第一个字节不是0xFE，说明有噪音干扰，重新从0xFE开始组合出一帧
    {
      unsigned char HeadStart = FrameHeadDex;
      for (unsigned char i = 0; i < (gReceiveLength - HeadStart); i++)
      {
        gReceiveCmd[i] = gReceiveCmd[FrameHeadDex++];
      }
    }
    if (gIsHandleMsgFlag)
      Receive_Data_Analysis();
      
    iwdg_feed();
  }
}

/*
 @brief     : 根据验证接收的帧ID，决定返回对应的指令枚举
 @param     : 无
 @return    : frame id type(enum)
 */
Frame_ID Command_Analysis::FrameID_Analysis(void)
{
	unsigned int FrameID = ((gReceiveCmd[1] << 8) | gReceiveCmd[2]);
	switch (FrameID)
	{
	case 0xA000: return Modbus_Control;				break;//通用控制器modbus控制(A000)
	case 0xA001: return R_Modbus_Control;			break;//通用控制器modbus控制回执(A001)
	case 0xA002: return Output_default;				break;//设置输出默认状态及超时时间(A0002)
	case 0xA003: return Irrigation_Control; 		break;//服务器发送灌溉控制器控制指令(A003)
	case 0xA004: return Delay_Start_DO_Control;		break;//服务器发送延时启动DO控制指令(A004)
	case 0xA005: return Positive_negative_Control; 	break;//服务器发送正反转控制指令(A005)
	case 0xA006: return Query_SignalQuality_Version;break;//服务器查询信号质量与版本号(A006)
	case 0xA007: return Set_Reporting_Interval;		break;//服务器设置上报时间间隔(A007)
	case 0xA008: return Set_Lora_parameter;			break;//服务器设置LORA参数(A008)
	case 0xA009: return Set_threshold;				break;//服务器设置正反转模式阈值（A009）
	case 0xA00A: return Calculate_travel;			break;//服务器发送正反转模式计算行程（A00A）
	case 0xA00B: return Opening_Control;			break;//服务器发送正反转模式开度控制指令（A00B）
	case 0xA011: return Work_Para;					break;//基地服务器查询LoRa设备当前工作参数(A011)
	case 0xA012: return Set_Group_Num;				break;//基地服务器设置LoRa设备工作组编号(A012)
	case 0xA013: return SN_Area_Channel;			break;//基地服务器设置设备（主/子）SN及子设备总路数(A013)
	case 0xA014: return Work_Status;				break;//查询LoRa设备当前工作状态（A014）
	case 0xA015: return Stop_Work;					break;//强制停止当前设备的工作(A015)
	//case 0xA020: return ResetRoll;			break;
	//case 0xA021: return Opening;			break;
	//case 0xA022: return Work_Limit;			break;

	default:/* gRcvOtherFlag = 1; Serial.println("RCV BT CMD"); BT_Receive_Protocol_Msg(&gReceiveCmd[0], &gReceiveLength, &gRcvOtherFlag); */break;
	}
	return Non_existent;
}

/*
 @brief     : 验证接收到的LoRa数据里的CRC8校验码
 @param     : 1.验证数据的起始地址
			  2.验证数据的长度
 @return    : true or false.
 */
bool Command_Analysis::Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len)
{
	unsigned char ReceiveCRC8 = GetCrc8(&gReceiveCmd[verify_data_base_addr], verify_data_len);
	if (ReceiveCRC8 == gReceiveCmd[gReceiveLength - 7])
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的设备ID与本机是否相同
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Device_Type_ID(void)
{
	unsigned int DeviceTypeID = ((gReceiveCmd[4] << 8) | gReceiveCmd[5]);
	if (DeviceTypeID == 0x5555)
		return true;

	if (DeviceTypeID == DEVICE_TYPE_ID)
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的指令是否是群发指令
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Verify_Mass_Commands(void)
{
	gReceiveCmd[6] == 0x55 ? gMassCommandFlag = true : gMassCommandFlag = false;
}

/*
 @brief   : 验证接收的区域号与本地是否相同
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Area_Number(void)
{
	if (gReceiveCmd[7] == 0x55) return true;  //0x55:群控指令，忽略区域号

	unsigned char LocalAreaNumber = SN.Read_Area_Number();
	if (gReceiveCmd[7] == LocalAreaNumber || LocalAreaNumber == 0)
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的工作组号是否在本机组控列表内。
			如果接收的组号是0x55，表明此指令忽略组控，发送给区域内所有的设备
			如果本设备还未申请注册过服务器，不用校验组号。
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Work_Group(void)
{
	if (gReceiveCmd[8] == 0x55) return true;  //0x55:群控指令，忽略工作组号

	unsigned char LocalGroupNumber[5], ReceiveGroupSingleNumber = gReceiveCmd[8];
	unsigned char UndefinedGroupNum = 0;
	SN.Read_Group_Number(&LocalGroupNumber[0]);

	for (unsigned char i = 0; i < 5; i++)
	{
		if (ReceiveGroupSingleNumber == LocalGroupNumber[i]) return true;

		/*全为0，说明是未初始化的组号，算校验通过*/
		if (LocalGroupNumber[i] == 0x00)
		{
			UndefinedGroupNum++;
			if (UndefinedGroupNum == 5)
				return true;
		}
	}
	return false;
}

/*
 @brief   : 验证接收的指令CRC8校验、设备类型码、区域号、工作组是否合法。
			可以通过形参决定是否屏蔽验证区域号和工作组。
 @param   : 1.验证的数据起始地址
			2.验证的数据长度
			3.是否要验证区域号标志位
			4.是否要验证工作组号标志位
 @return  : true or false.
 */
bool Command_Analysis::Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag = true, bool group_flag = true)
{
	if ((Verify_CRC8(verify_data_base_addr, verify_data_len) == true) || ((gReceiveCmd[gReceiveCmd[3]+11-7]) == 0xD6))// (gReceiveCmd[gReceiveLength - 7] == 0xD6)
	{
		if (Verify_Device_Type_ID() == true)
		{
			Verify_Mass_Commands();
			if (Verify_Area_Number() == true || area_flag == false)
			{
				if (Verify_Work_Group() == true || group_flag == false)
					return true;
				else
				{
					Debug_Serial.println("Not this device group number... <Verify_Frame_Validity>");
					Debug_Serial.println("不是这个设备组号... <Verify_Frame_Validity>");
				}
			}
			else
			{
				Debug_Serial.println("Not this device area number... <Verify_Frame_Validity>");
				Debug_Serial.println("不是这个设备区域号... <Verify_Frame_Validity>");
			}
		}
		else
		{
			Debug_Serial.println("Device type ID ERROR! <Verify_Frame_Validity>");
			Debug_Serial.println("设备类型ID错误! <Verify_Frame_Validity>");
		}
	}
	else
	{
		Debug_Serial.println("CRC8 ERROR! <Verify_Frame_Validity>");
	}
	return false;
}

/*
 @brief   : 根据帧ID分析判断执行哪一个接收到的通用指令或私有指令
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Receive_Data_Analysis(void)
{
	switch (FrameID_Analysis())//根据验证接收的帧ID，决定返回对应的指令枚举
	{
	/*通用指令*/
	case Work_Para: Query_Current_Work_Param();		break;//服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等(A011)
	case Set_Group_Num: Set_Group_Number();			break;//设置本设备的工作组号，不需要验证本设备原有工作组号(A012)
	case SN_Area_Channel: Set_SN_Area_Channel();	break;//基地服务器设置设备（主/子）SN及子设备总路数(A013)
	case Work_Status: Detailed_Work_Status();		break;//查询本设备详细工作状态(A014)
	case Stop_Work: Stop_Work_Command();			break;//强制停止当前设备的工作(A015)
	case Non_existent: Debug_Serial.println("不存在于本设备帧ID!!!");break;//异常处理
	///*卷膜机私有指令*/
	//case ResetRoll: ResetRoll_Command();			break;//重置卷膜测量行程(A020)
	//case Opening: Opening_Command();				break;//设置卷膜开度(A021)
	//case Work_Limit: Working_Limit_Command();		break;//电机工作电压阈值、上报状态间隔值设置(A022)

	/*通用控制器私有指令*/
	case Modbus_Control: General_controller_control_command();		break;//服务器发送通用控制器Modbus控制指令(A000)
	case R_Modbus_Control: Set_RTC_command();  break;//服务器发送通用控制器Modbus控制指令接收回执(A001)
	case Output_default: Set_General_controller_output_init();		break;//服务器设置通用控制器输出默状态(A002)

	case Irrigation_Control: Irrigation_Controllor_control_command(); break;//服务器发送灌溉控制器控制指令(A003)
	case Delay_Start_DO_Control: Delay_Start_DO_Control_command(); break;//服务器发送延时启动DO控制指令(A004)
	case Positive_negative_Control: Positive_negative_Control_command(); break;//服务器发送正反转控制指令(A005)
	case Query_SignalQuality_Version: Query_SignalQuality_Version_command(); break;//服务器查询信号质量与版本号(A006)
	case Set_Reporting_Interval: Set_Reporting_Interval_command(); break;//服务器设置上报时间间隔(A007)
	case Set_Lora_parameter: Set_Lora_parameter_command(); break;//服务器设置LORA参数(A008)
	case Set_threshold: Set_Forward_Reverse_mode_threshold(); break;//服务器设置正反转模式阈值（A009）
	case Calculate_travel: Forward_Reverse_mode_Calculate_travel();break;//服务器发送正反转模式计算行程（A00A）
	case Opening_Control: Forward_Reverse_mode_Opening_Control();break;//服务器发送正反转模式开度控制指令（A00B）
	}
}

/*
 @brief   : 服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等。（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Query_Current_Work_Param(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 | 申号标志 |  查询角色 | 采集时间间隔      |  时间   |  预留位     |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number | intent   |  channel | collect interval  |  RTC   |   allocate  |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte       1 byte       1 byte      1 byte      2 byte           7 byte      8 byte     1 byte      6 byte

	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

	if (Verify_Frame_Validity(4, 23, true, false) == true)
	{
		Debug_Serial.println("A011 <Query_Current_Work_Param>");
		Debug_Serial.flush();
		// delay(200);
		if (gReceiveCmd[8] == 0x01) //配置参数标志（LoRa大棚传感器是0x00配置采集时间）
		{
			/* 预留位第一字节用来设置LoRa的通信模式 */
			if (LoRa_Para_Config.Save_LoRa_Com_Mode(gReceiveCmd[19]))
			{
				Message_Receipt.General_Receipt(SetLoRaModeOk, 1);
				LoRa_MHL9LF.Parameter_Init(true);
				//Message_Receipt.Working_Parameter_Receipt(true, 2);
			}
			else
			{
				Message_Receipt.General_Receipt(SetLoRaModeErr, 2);
				Debug_Serial.println("Set LoRa Mode Err设置LORA模式错误! <Query_Current_Work_Param>");
			}
		}
		else  //回执状态标志
		{
			Message_Receipt.Report_General_Parameter();
		}
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 设置本设备的工作组号，不需要验证本设备原有工作组号（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Set_Group_Number(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 | 工作组号   | 设备路数 |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag   |  Area number |  workgroup |  channel |   CRC8 |  |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte        5 byte       1 byte    1 byte      6 byte

	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

	if (Verify_Frame_Validity(4, 10, true, false) == true)
	{
		Debug_Serial.println("A013 <Set_Group_Number>");
		Debug_Serial.flush();
		if (SN.Save_Group_Number(&gReceiveCmd[8]) == true)
		{
			Debug_Serial.println("Save group number success保存组号成功... <Set_Group_Number>");
			Message_Receipt.General_Receipt(AssignGroupIdArrayOk, 2);
		}
		else
		{
			Debug_Serial.println("Save group number failed保存组号失败 !!! <Set_Group_Number>");
			/*  */
			Message_Receipt.General_Receipt(AssignGroupIdArrayErr, 1);
		}
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 设置本设备的SN码、区域号、设备路数等参数（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Set_SN_Area_Channel(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   | 所在执行区域号 |  设备路数      |  子设备总路数           |  SN码       | 校验码   |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag   |   Area number | Device channel |  subordinate channel   | SN code     |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte          1 byte           1 byte                9 byte       1 byte      6 byte

	if (Verify_Frame_Validity(4, 15, false, false) == true)
	{
		Debug_Serial.println("A013 <Set_SN_Area_Channel>");
		Debug_Serial.flush();
		if (SN.Save_SN_Code(&gReceiveCmd[10]) == true && SN.Save_BKP_SN_Code(&gReceiveCmd[10]) == true)
		{
			Debug_Serial.println("Set SN code success... <Set_SN_Area_Channel>");
			if (SN.Save_Area_Number(gReceiveCmd[7]) == true)
			{
				Debug_Serial.println("Save area number success保存区域ID成功... <Set_SN_Area_Channel>");
				Message_Receipt.General_Receipt(SetSnAndSlaverCountOk, 1);
				SN.Set_SN_Access_Network_Flag();
			}
			else
			{
				Debug_Serial.println("Save area number ERROR保存区域ID失败 !!! <Set_SN_Area_Channel>");
				/*  */
				Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
			}
		}
		else
		{
			Debug_Serial.println("Save SN code ERROR保存SN出错 !!! <Set_SN_Area_Channel>");
			/*  */
			Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
		}
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 查询本设备详细工作状态（服务器 ---> 本设备）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Detailed_Work_Status(void)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9      	10  	11-15        
// 数据域     	FrameHead	FrameId	DataLen	DeviceTypeId	IsBroadcast	ZoneId	GroupId	channel	CRC8	FrameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	1      	1   	6            
// 示例数据    	FE       	A014   	06     	C003        	55         	01    	01     	00     	D6  	0D0A0D 0A0D0A

//FE A014 06 C003 55 01 01 00 D6 0D0A0D0A0D0A
	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true) 
	{
		Debug_Serial.println("A014 <Detailed_Work_Status>");
		Debug_Serial.flush();

		Debug_Serial.println("查询本设备详细工作状态 <Detailed_Work_Status>");

		// Message_Receipt.Working_Parameter_Receipt(false, 1, gReceiveCmd[12], gReceiveCmd[13]);
		Message_Receipt.New_Working_Parameter_Receipt(false,2);
	}

	memset(gReceiveCmd, 0x00, gReceiveLength);
}

 /*
  @brief   : 强制停止当前设备的工作（服务器 ---> 本设备）
  @param   : 无
  @return  : 无
  */
 void Command_Analysis::Stop_Work_Command(void)
 {
 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 |  设备路数 |  校验码  |     帧尾 
 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number |   channel |   CRC8 |  |  Frame end
 	//  1 byte       2 byte      1 byte          2 byte        1 byte        1 byte         1 byte    1 byte      6 byte

	// FE A015 05 C003 00 55 01 D6 0D0A0D0A0D0A
 	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

 	if (Verify_Frame_Validity(4, 5, true, false) == true)
 	{
		Debug_Serial.println("A015 <Stop_Work_Command>");
		Debug_Serial.println("强制停止!!!");
		Debug_Serial.flush();

		Forced_Stop(true);
 	}
 	memset(gReceiveCmd, 0x00, gReceiveLength);
 }

// /*
//  @brief     : 重置卷膜测量行程
//  @param     : 无
//  @return    : 无
//  */
// void Command_Analysis::ResetRoll_Command(void)
// {
// 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 |所在执行区域号 |  工作组号   | 设备路数 |  校验码  |     帧尾 
// 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag  |  Area number |   workgroup | channel |   CRC8 |  |  Frame end
// 	//  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte         1 byte      1 byte    1 byte      6 byte

// 	if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

// 	if (Verify_Frame_Validity(4, 6, true, true) == true)
// 	{
// 		/*如果电机手动卷膜按键电路异常，禁止自动卷膜，等待更换设备*/
// 		if (gManualKeyExceptionFlag)
// 		{
// 			Serial.println("Manual roll key exception手动卷膜键异常 !!! <ResetRoll_Command>");
// 			return;
// 		}
// 		/*如果当前正在卷膜，不进行二次卷膜*/
// 		else if (gResetRollWorkingFlag == true || gOpeningWorkingFlag == true || gForceRollWorkingFlag == true)
// 		{
// 			Serial.println("The motor is resetting the distance卷膜机正在重置行程... <ResetRoll_Command>");
// 			return;
// 		}
// 		/*如果当前正在手动卷膜。拒绝执行自动卷膜*/
// 		else if (gManualUpDetectFlag || gManualDownDetectFlag)
// 		{
// 			if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
// 			if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
// 			if (gManualUpDetectFlag || gManualDownDetectFlag)
// 			{
// 				Serial.println("Detect manual rolling检测到手动卷膜... <ResetRoll_Command>");
// 				/*
// 				  *待处理事情
// 				*/
// 				return;
// 			}
// 		}
// 		else
// 		{
// 			MANUAL_ROLL_OFF;/*使能/失能手动卷膜*/
// 			detachInterrupt(DEC_MANUAL_DOWN_PIN);
// 			detachInterrupt(DEC_MANUAL_UP_PIN);
// 			Message_Receipt.General_Receipt(RestRollerOk, 1);
// 			Motor_Operation.Reset_Motor_Route();
// 			attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 			attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 			MANUAL_ROLL_ON;/*使能/失能手动卷膜*/
// 			iwdg_feed();
// 		}
// 	}
// 	memset(gReceiveCmd, 0x00, gReceiveLength);
// }

// /*
//  @brief     : 设置卷膜开度
//  @param     : 无
//  @return    : 无
//  */
// void Command_Analysis::Opening_Command(void)
// {
// 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 |  工作组号   | 设备路数 |  开度    | 校验码  |     帧尾 
// 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag  |  Area number |   workgroup | channel |  oepning | CRC8 |  |  Frame end
// 	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte    1 byte    1 byte      6 byte

// 	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

// 	if (Verify_Frame_Validity(4, 7, true, true) == true)  //如果校验通过（CRC校验、区域号校验、组号校验）
// 	{
// 		Serial.println("这是卷膜函数《测试》");
// 		/*如果电机手动卷膜按键电路异常，禁止自动卷膜*/
// 		if (gManualKeyExceptionFlag)
// 		{
// 			Set_Motor_Status(MANUAL_KEY_EXCEPTION);
// 			Message_Receipt.Working_Parameter_Receipt(false, 2);
// 			Serial.println("Manual roll key exception !!! <Opening_Command>");
// 			Serial.println("手动滚动键异常!!! <Opening_Command>");
// 			return;
// 		}
// 		/*如果当前正在手动卷膜。拒绝执行自动卷膜*/
// 		else if (gManualUpDetectFlag || gManualDownDetectFlag)
// 		{
// 			if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
// 			if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
// 			if (gManualUpDetectFlag || gManualDownDetectFlag)
// 			{
// 				Serial.println("Detect manual rolling... <Opening_Command>");
// 				Serial.println("检测手动卷膜... <Opening_Command>");
// 				/*
// 				  *待处理事情
// 				*/
// 				return;
// 			}
// 		}
// 		/*如果当前正在卷膜，不进行二次卷膜*/
// 		else if (gOpeningWorkingFlag == true || gResetRollWorkingFlag == true || gForceRollWorkingFlag == true)
// 		{
// 			Serial.println("Currently the motor is opening, and others opening cannot to do !!! <Opening_Command>");
// 			Serial.println("目前电机正在卷膜，不进行二次卷膜 !!! <Opening_Command>");
// 			return;
// 		}

// 		/*失能手动卷膜， 失能检测手动卷膜按键中断*/
// 		detachInterrupt(DEC_MANUAL_DOWN_PIN);
// 		detachInterrupt(DEC_MANUAL_UP_PIN);
// 		MANUAL_ROLL_OFF;

// 		Message_Receipt.General_Receipt(OpenRollerOk, 1); //通用回执，告诉服务器接收到了开度卷膜命令
// 		Serial.println("发送回执信息成功《测试》");

// 		volatile unsigned char opening_value = gReceiveCmd[10]; //获取从服务器接收的开度值

// 		/*
// 		  *如果是开度卷膜，且要求全关或全开。本次当前开度正好已经是全关或全开了
// 		  *那么就不需要再次进入到卷膜函数里。
// 		  *应用在于：假如冬天不能开棚，本来也确实关着。服务器发送一个关棚开度，
// 		  *程序会先判断有没有重置行程，假如有些因素导致还要重置行程，那么就会打开棚了。
// 		  *当然，不适用于强制卷膜
// 		  *还有，假如发来的开度是0或100，说明需要开棚和关棚，一般用于天冷了或天热了，这个时候就
// 		  *不去判断是否发来的和保存的是否一至了，而是发来的只要是关棚或开棚，同时需要重置行程，
// 		  *就会用强制开棚和强制关棚来操作，最大限度不去重置行程。
// 		  *而其他的开度值，就会去判断上一次的开度和本次的是否相等，如果相等，就不动作
// 		  *如果不相等，同时又需要重置行程，那么只能必须先重置行程才能开到某个开度了。
// 		 */
// 		if (opening_value >= 0 && opening_value <= 100)
// 		{
// 			unsigned char RealTimeOpenTemp = Roll_Operation.Read_RealTime_Opening_Value();//读取实时卷膜开度值

// 			if (!Roll_Operation.Read_Route_Save_Flag())
// 			{
// 				if (opening_value == 0 || opening_value == 100)
// 				{
// 					if (opening_value == 0)
// 						opening_value = 0xF0;
// 					else
// 						opening_value = 0xF1;

// 					Serial.println("Prepare Force Open or Close. Be careful... <Opening_Command>");
// 					Serial.println("准备强行全开或全关。小心... <Opening_Command>");
// 					Motor_Operation.Force_Open_or_Close(opening_value);
// 					memset(gReceiveCmd, 0x00, gReceiveLength);
// 					/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
// 					attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 					attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 					MANUAL_ROLL_ON;
// 					return;
// 				}
// 			}

// 			if (RealTimeOpenTemp == opening_value)
// 			{
// 				Serial.println("Film has been rolled to the current opening, do not repeat the film... <Opening_Command>");
// 				Serial.println("已滚动到当前开度值，不要重复卷膜…... <Opening_Command>");
// 				Set_Motor_Status(ROLL_OK);
// 				Message_Receipt.Working_Parameter_Receipt(true, 2);

// 				attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 				attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 				MANUAL_ROLL_ON;
// 				return;
// 			}
// 		}

// 		//正常开度值范围是0到100，如果是F0，表明是强制关棚，如果是F1，表明是强制开棚
// 		if (opening_value == 0xF0 || opening_value == 0xF1)
// 		{
// 			Serial.println("Prepare Force Open or Close. Be careful... <Opening_Command>");
// 			Serial.println("准备全开全关，请小心... <Opening_Command>");
// 			Motor_Operation.Force_Open_or_Close(opening_value);
// 			memset(gReceiveCmd, 0x00, gReceiveLength);
// 			/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
// 			attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 			attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 			MANUAL_ROLL_ON;
// 			return;
// 		}

// 		if (Roll_Operation.Read_Route_Save_Flag())  //如果已经重置行程过
// 		{
// 			if (Roll_Operation.Save_Current_Opening_Value(opening_value))  //保存当前开度值
// 			{
// 				Serial.println("Begin to coiling... <Opening_Command>");
// 				Serial.println("开始卷膜... <Opening_Command>");
// 				Motor_Operation.Motor_Coiling();  //开始卷膜
// 				iwdg_feed();
// 			}
// 			else  //保存开度值异常
// 			{
// 				Serial.println("Save current opening value ERROR !!! <Opening_Command>");
// 				Serial.println("保存当前开度值错误!!! <Opening_Command>");
// 				Set_Motor_Status(STORE_EXCEPTION);
// 				Message_Receipt.Working_Parameter_Receipt(false, 2);
// 			}
// 		}
// 		else  //或者没有重置行程
// 		{
// 			Serial.println("The film has not measured the distance, first measure, then roll the film... <Opening_Command>");
// 			Serial.println("电机未测量距离，先测量距离，再卷膜... <Opening_Command>");
// 			if (Motor_Operation.Reset_Motor_Route() == true) //先重置行程，再开度卷膜
// 			{
// 				Serial.println("Roll OK, motor begin coiling... <Opening_Command>");
// 				Serial.println("重置行程完成,电机开始转动... <Opening_Command>");

// 				if (Roll_Operation.Save_Current_Opening_Value(opening_value))  //保存当前开度值
// 				{
// 					gAdjustOpeningFlag = false;
// 					Motor_Operation.Motor_Coiling();  //开始卷膜
// 				}
// 				else  //保存开度值操作异常
// 				{
// 					Serial.println("Save current opening value ERROR !!! <Opening_Command>");
// 					Serial.println("保存当前开度值错误 !!! <Opening_Command>");
// 					Set_Motor_Status(STORE_EXCEPTION);
// 					Message_Receipt.Working_Parameter_Receipt(false, 2);
// 				}
// 			}
// 			else  //重置行程失败
// 				Serial.println("Reset motor route failed !!! <Opening_Command>");
// 			iwdg_feed();
// 		}
// 		/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
// 		attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
// 		attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
// 		MANUAL_ROLL_ON;
// 	}
// 	memset(gReceiveCmd, 0x00, gReceiveLength);
// }

// /*
//  @brief     : 电机工作电压阈值、上报状态间隔值设置（网关 ---> 本机）
//  @param     : 无
//  @return    : 无
//  */
// void Command_Analysis::Working_Limit_Command(void)
// {
// 	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  | 所在执行区域号 |  工作组号   | 设备路数 | 低电压阈值       |   高电压阈值      | 状态上报间隔     |校验码 | 帧尾 
// 	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |   Area number |   workgroup | channel | LowVolThreshold | HighVolThreshold |  ReprotInterval | CRC8 |  Frame end
// 	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte     2 byte             2 byte              1 byte        1 byte  6 byte

// 	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

// 	if (Verify_Frame_Validity(4, 11, true, true) == true)
// 	{
// 		if (Roll_Operation.Save_Roll_Work_Voltage_and_Report_Interval(&gReceiveCmd[10]) == true)
// 			Message_Receipt.General_Receipt(LimitRollerOk, 1);
// 		else
// 		{
// 			Serial.println("Save working threshold ERROR !");
// 			Serial.println("保存工作阈值错误!");
// 			Message_Receipt.General_Receipt(LimitRollerErr, 1);
// 			Set_Motor_Status(STORE_EXCEPTION);
// 			Message_Receipt.Working_Parameter_Receipt(false, 2);
// 		}
// 	}
// 	memset(gReceiveCmd, 0x00, gReceiveLength);
//}

 /*
  @brief     : 服务器发送通用控制器Modbus控制指令（网关 ---> 本机）
  @param     : 无
  @return    : 无
  */
void Command_Analysis::General_controller_control_command(void)
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8      	9-n         	    	             
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	zoneId	groupId	modbusPacket	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	n           	1   	2            
// 示例数据  	FE       	A000   	5+n    	C003        	00         	01    	01     	XXXXXXXX    	D6  	0D0A0D 0A0D0A



	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)
	{
		Debug_Serial.println("A000 <General_controller_control_command>");
		Debug_Serial.flush();
		iwdg_feed();


		Debug_Serial.println("服务器发送通用控制器Modbus控制指令 <General_controller_control_command>");

		unsigned char modbusPacket[20] = { 0x00 };unsigned int modbusPacket_Length = gReceiveCmd[3] - 5;
		// unsigned char R_modbusPacket[20] = { 0x00 };unsigned int R_modbusPacket_Length = 0;
		if (modbusPacket_Length > 20)
		{
			//此处应该有异常处理！！！
			Debug_Serial.println("modbusPacket_Length超过数组预设值!!! <General_controller_control_command>");
			gStatus_E014 = Modbuspacket_length_overflow;//modbusPacket_Length超过数组预设值
		}
		else
		{
			for (size_t i = 0; i < modbusPacket_Length; i++)
			{
				modbusPacket[i] = gReceiveCmd[9 + i];
				//Serial.println(modbusPacket[i], HEX);
			}
			gStatus_E014 = A000_Received_Success;//A000指令接收成功
		}

		Modbus_Coil.Modbus_Realization(modbusPacket, modbusPacket_Length);//设置输出线圈状态，modbus实现

		Message_Receipt.Control_command_Receipt(false,1);
		DOStatus_Change = true;//接收到指令后上报实时状态
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

 /*
  @brief     : 服务器发送设置RTC指令（网关 ---> 本机）
  @param     : 无
  @return    : 无
  */
void Command_Analysis::Set_RTC_command(void)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9-15          	16  	17-22        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	zoneId	groupId	rtcTime       	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	7             	1   	6            
// 示例数据    	FE       	A001   	0C     	C003        	00         	01    	01     	20200101103520	00  	0D0A0D 0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A001 <Set_RTC_command>");
		Debug_Serial.flush();

		Debug_Serial.println("服务器发送设置RTC指令 <Set_RTC_command>");

		unsigned char RTC[7];//
		for (size_t i = 0; i < 7; i++)
		{
			RTC[i] = gReceiveCmd[9 + i];
		}
		Private_RTC.Update_RTC(&RTC[0]);

		Message_Receipt.General_Receipt(SetRTCOK,2);

		// /* 预留位第一字节用来设置LoRa的通信模式 */
		// if(LoRa_Para_Config.Save_LoRa_Com_Mode(gReceiveCmd[20]))
		// {
		// 	Message_Receipt.General_Receipt(SetLoRaModeOk, 1);
		// 	LoRa_MHL9LF.Parameter_Init(true);
		// 	// Message_Receipt.Working_Parameter_Receipt(true, 2);
		// }
		// else 
		// {
		// 	Message_Receipt.General_Receipt(SetLoRaModeErr, 2);
		// 	Serial.println("Set LoRa Mode Err! <Query_Current_Work_Param>");
		// }

		// /* 前4位为发送频点，后4位为接收频点，若是全为0则表示不更改 */
		// if (!(gReceiveCmd[21] == 0x00 && gReceiveCmd[22] == 0x00 && gReceiveCmd[23] == 0x00 && gReceiveCmd[24] == 0x00))
		// {
		// 	String TFREQ = String(gReceiveCmd[21],HEX)+String(gReceiveCmd[22],HEX)+String(gReceiveCmd[23],HEX)+String(gReceiveCmd[24],HEX);
		// 	Serial.println(String("TFREQ = ") + TFREQ);
		// 	// if (LoRa_MHL9LF.Param_Check(AT_TFREQ_, "1C578DE0", false))
		// 	// {
		// 	// 	// Message_Receipt.Working_Parameter_Receipt(true, 2);
		// 	// }
		// 	// else
		// 	// {
		// 	// 	Message_Receipt.General_Receipt(SetLoRaTFREQErr, 2);
		// 	// 	Serial.println("Set LoRa TFREQ Err! <Query_Current_Work_Param>");
		// 	// }
		// }
		// /* 前4位为发送频点，后4位为接收频点，若是全为0则表示不更改 */
		// if (!(gReceiveCmd[24] == 0x00 && gReceiveCmd[25] == 0x00 && gReceiveCmd[26] == 0x00 && gReceiveCmd[27] == 0x00))
		// {
		// 	String RFREQ = String(gReceiveCmd[24],HEX)+String(gReceiveCmd[25],HEX)+String(gReceiveCmd[26],HEX)+String(gReceiveCmd[27],HEX);
		// 	Serial.println(String("RFREQ = ") + RFREQ);
		// 	// if (LoRa_MHL9LF.Param_Check(AT_RFREQ_, "1C578DE0", false))
		// 	// {
		// 	// 	// Message_Receipt.Working_Parameter_Receipt(true, 2);
		// 	// }
		// 	// else
		// 	// {
		// 	// 	Message_Receipt.General_Receipt(SetLoRaTFREQErr, 2);
		// 	// 	Serial.println("Set LoRa TFREQ Err! <Query_Current_Work_Param>");
		// 	// }
		// }

		return;
		
	}
}


/*
 @brief     : 服务器设置通用控制器输出默认状态（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Set_General_controller_output_init(void)
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8-15  	16-31 	 32 	33-38        
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	IsBroadcast	ZoneId	DOInit	AOInit	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	8     	16    	 1  	6            
// 示例数据  	FE       	A002   	0x1C   	C003        	55         	01    	      	      	 00 	0D0A0D 0A0D0A


	/*	|Status_E002:	|0x00	|0x01			|0x02			|0x03		|0x04		|
		|				|正常	|保存超时时间失败|保存初始化值失败	|485无回执	|485回执溢出	|*/

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A002 <Set_General_controller_output_init>");
		Debug_Serial.flush();

		unsigned char Status_E002 = 0x00;//状态

		if (!InitState.Save_Timeout(gReceiveCmd[34], gReceiveCmd[35]))
		{
			Debug_Serial.println("保存timeout失败!!!<Set_General_controller_output_init>");
			Status_E002 = 0x01;
		}
		Debug_Serial.println("保存timeout成功<Set_General_controller_output_init>");

		//Serial.println("开始保存通用控制器输出默认状态<Set_General_controller_output_default>");
		//Serial.println(String(gReceiveCmd[8])+String(gReceiveCmd[9]));

		unsigned char DO_Init[8] = { 0x22 };unsigned char AO_Init[16] = { 0x33 };

		//将DO初始化的值赋给DO_Init[8]
		for (size_t i = 0; i < 8; i++)
		{
			DO_Init[i] = gReceiveCmd[i + 8];
			//Serial.println(String("DO_Init[") + i + "]=" + DO_Init[i]);
		}

		//将AO初始化的值赋给AO_Init[8]
		for (size_t i = 0; i < 16; i++)
		{
			AO_Init[i] = gReceiveCmd[i + 16];
			//Serial.println(String("AO_Init[") + i + "]=" + AO_Init[i]);
		}
		
		if (InitState.Save_DO_InitState(DO_Init) && InitState.Save_AO_InitState(AO_Init))//读取初始状态的标志位
		{
			Debug_Serial.println("保存初始化值成功<Set_General_controller_output_init>");
			InitState.Save_InitState_Flag();//存储初始状态的标志位
			Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
		}
		else
		{
			Debug_Serial.println("保存初始化值失败<Set_General_controller_output_init>");
			Status_E002 = 0x02;
			InitState.Clean_InitState_Flag();//清除初始状态的标志位
			Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
		}

		Message_Receipt.Output_init_Receipt(Status_E002, 1);
	}
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器发送灌溉控制器控制指令(A003)（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Irrigation_Controllor_control_command(void)
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8-39   	40-69   	70-71  	72-73 	74-105  	106 	107-112      
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	IsBroadcast	ZoneId	openSec	interval	timeOut	DOUsed	retryCnt	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	32     	30      	2      	2     	32      	 1  	6            
// 示例数据  	FE       	A003   	0x66   	C003        	55         	01    	       	        	       	00 00 	        	 00 	0D0A0D 0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	// FE A0 3 66 C0 3 0 5 12 34 0 5A 0 0 0 5A 0 0 0 5A 0 0 0 5A 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 55 0 0 1 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 EA D A D A D A Get frame end... <Receive_LoRa_Cmd>

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A003 <Irrigation_Controllor_control_command>");
		Debug_Serial.flush();

		Enter_Work_State = true;
		
		Forced_Stop(false);

		Device_Mode = Solenoid_mode;//定时DO模式（电磁阀）

		if (InitState.Save_CyclicInterval(gReceiveCmd[70], gReceiveCmd[71]))
		{
			Debug_Serial.println("设置并保存参数成功 <Irrigation_Controllor_control_command>");
		}
		else
		{
			Debug_Serial.println("设置并保存参数失败!!! <Irrigation_Controllor_control_command>");
		}
		Cyclic_interval = InitState.Read_CyclicInterval();

		//DO_Num = gReceiveCmd[16];
		//retryCnt = gReceiveCmd[76] * 0x100 + gReceiveCmd[77];//得到循环次数
		/*if (DO_Num <= 0 || DO_Num > 8)
		{
			Serial.println("ERROR!!!一次性开启的DO数量错误");
			DO_Num = 1;
		}*/
		//Serial.println(String("开启时间openSec = ") + OpenSec);
		//Serial.println(String("单个间隔时间DO_Interval = ") + DO_Interval);
		Debug_Serial.println(String("循环间隔时间Cyclic_interval = ") + Cyclic_interval);
		//Serial.println(String("一次开启的DO数量DO_Num = ") + DO_Num);
		//Serial.println(String("循环次数retryCnt = ") + retryCnt);
		iwdg_feed();
		byte bitn = 0;
		word Outputs = 12; word UseOutputs = 12;
		word x;
		byte Bitread = 0;
		for (size_t i = 0; i < 12; i++)
		{
			x = (Outputs - UseOutputs) / 8;
			Bitread = ((gReceiveCmd[72 + x] >> bitn) & 0x01);
			//Serial.println(String("Bitread = ") + Bitread);
			if (Bitread)
			{
				Worktime[i] = gReceiveCmd[(2 * i) + 8] * 0x100 + gReceiveCmd[(2 * i) + 9];
				WorkInterval[i] = gReceiveCmd[(2 * i) + 40] * 0x100 + gReceiveCmd[(2 * i) + 41];
				retryCnt[i] = gReceiveCmd[(2 * i) + 74] * 0x100 + gReceiveCmd[(2 * i) + 75];
			}

			bitn++;
			if (bitn == 8) bitn = 0;

			UseOutputs--;
		}

		//Serial.println(String("需要开启的个数Need_Num = ") + Need_Num);


		/*if (Need_Num % DO_Num == 0)
		{
			fornum = Need_Num / DO_Num;
			Last_full = true;
			Serial.println("Last_full = true");
		}
		else
		{
			fornum = (Need_Num / DO_Num)+1;
			Last_full = false;
			Last_num = Need_Num % DO_Num;
			Serial.println("Last_full = false");
			Serial.println(String("Last_num = ") + Last_num);
		}
		fornum_backups = fornum;
		Serial.println(String("一轮循环需要的循环次数fornum = ") + fornum);*/

		// for (size_t i = 0; i < 12; i++)
		// {
		// 	Serial.println(String("Worktime[") + i + "]= " + Worktime[i]);
		// 	Serial.println(String("WorkInterval[") + i + "]= " + WorkInterval[i]);
		// 	Serial.println(String("retryCnt[") + i + "]= " + retryCnt[i]);
		// 	Serial.println("---");
		// }
		Debug_Serial.flush();

		/*这里上报E003定时IO指令接收回执*/
		Message_Receipt.Irrigation_control_Receipt(3, gReceiveCmd);

		Start_Timer4();		
	}
	else
	{
		Debug_Serial.println("不用理会的A003指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}


/*
 @brief     : 服务器发送延时DO指令（A004）（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Delay_Start_DO_Control_command()
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8-9    	10      	11   	12-13 	14  	15~20        
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	openSec	interval	DONum	DOUsed	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	2      	1       	1    	2     	1   	6            
// 示例数据  	FE       	A004   	0A     	C003        	00         	01    	003C   	30      	2    	FF00  	00  	0D0A0D 0A0D0A

	
	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A004 0A C003 00 55 003C 05 02 FF00 D6 0D0A0D0A0D0A  

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A004 <Delay_Start_DO_Control_command>");
		Debug_Serial.flush();

		Enter_Work_State = true;

		Forced_Stop(false);

		Device_Mode = Delay_mode;//延时DO模式

		Delay_mode_OpenSec = gReceiveCmd[8]*0x100 + gReceiveCmd[9];
		Debug_Serial.println(String("Delay_mode_OpenSec = ") + Delay_mode_OpenSec + "s");

		Delay_mode_Interval = gReceiveCmd[10];
		Debug_Serial.println(String("Delay_mode_Interval = ") + Delay_mode_Interval + "s");

		Delay_mode_DONum = gReceiveCmd[11];
		Debug_Serial.println(String("Delay_mode_DONum = ") + Delay_mode_DONum);

		byte bitn = 0; byte Bitread = 0;
		word Outputs = 12; word UseOutputs = 12; word x;
		for (size_t i = 0; i < 12; i++)
		{
			x = (Outputs - UseOutputs) / 8;
			Bitread = ((gReceiveCmd[12 + x] >> bitn) & 0x01);
			if (Bitread)
			{
				//表示第几个需要开启
				Delay_mode_Worktime[i] = Delay_mode_OpenSec;
				Debug_Serial.println(String("Delay_mode_Worktime[") + i + "]= " + Delay_mode_Worktime[i]);
			}
			bitn++;
			if (bitn == 8) bitn = 0;

			UseOutputs--;
		}


		/*这里上报E004延时DO指令接收回执*/
		Message_Receipt.Delay_Start_DO_Control_Receipt(3, gReceiveCmd);

		Start_Timer4();
	}
	else
	{
		Debug_Serial.println("不用理会的A004指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器发送正反转控制指令(A005)（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Positive_negative_Control_command()
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8-15    	16      	17       	18  	19~24        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	WorkTime	interval	DOUsed   	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	12      	1       	2        	1   	6            
// 示例数据    	FE       	A005   	0x13   	C003        	00         	01    	000A000A	03      	5504/AA00	00  	0D0A0D 0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A005 13 C003 00 55 000A 000A 000A 000A 000A 000A 05 5504 D6 0D0A0D0A0D0A 

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A005 <Positive_negative_Control_command>");
		Debug_Serial.flush();

		Enter_Work_State = true;
		
		Forced_Stop(false);
		
		Device_Mode = Forward_Reverse_mode;//正反转模式

		byte bitn = 0; byte Bitread = 0;
		for (size_t i = 0; i < 12; i++)
		{
			if (i>7)
			{
				Bitread = ((gReceiveCmd[22] >> bitn) & 0x01);
			}
			else
			{
				Bitread = ((gReceiveCmd[21] >> bitn) & 0x01);
			}

			if (Bitread)
			{
				//表示第几个需要开启
				Forward_Reverse_mode_Worktime[i] = gReceiveCmd[8+2*(i/2)]*0x100 + gReceiveCmd[9+2*(i/2)];
				Debug_Serial.println(String("Forward_Reverse_mode_Worktime[") + i + "]= " + Forward_Reverse_mode_Worktime[i]);
			}
			bitn++;
			if (bitn == 8) bitn = 0;
		}
		
		Forward_Reverse_mode_Interval = gReceiveCmd[20];
		Debug_Serial.println(String("Forward_Reverse_mode_Interval = ") + Forward_Reverse_mode_Interval + "s");

		/*这里上报E005正反转指令接收回执*/
		Message_Receipt.Positive_negative_Control_Receipt(3, gReceiveCmd);

		Start_Timer4();
	}
	else
	{
		Debug_Serial.println("不用理会的A005指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器查询信号质量与版本号指令(A006)（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Query_SignalQuality_Version_command()
{
//   字节索引    	0        	1-2    	3      	4-5         	6          	7     	8   	9~14         
//   数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	CRC8	frameEnd     
//   长度（byte）	1        	2      	1      	2           	1          	1     	1   	6            
//   示例数据    	FE       	A006   	04     	C003        	00         	01    	00  	0D0A0D 0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A006 04 C003 00 55 D6 0D0A0D0A0D0A  

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A006 <Query_Signal_Quality>");
		Debug_Serial.flush();

		/*这里上报E007信号质量与版本号回执*/
		Message_Receipt.SignalQuality_Version_Receipt(false, 3);
	}
	else
	{
		Debug_Serial.println("不用理会的A006指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器设置上报时间间隔指令(A007)（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Set_Reporting_Interval_command()
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8-9         	10-11       	12  	13~18        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	WorkInterval	StopInterval	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	2           	2           	1   	6            
// 示例数据    	FE       	A007   	08     	C003        	00         	01    	0005        	003C        	00  	0D0A0D 0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A007 08 C003 00 55 0005 003C D6 0D0A0D0A0D0A  

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A007 <Set_Reporting_Interval_command>");
		Debug_Serial.flush();

		bool Save_Success = true;//用来标志是否存储成功

		/*这里存工作的时间间隔*/
		unsigned int WorkInterval = gReceiveCmd[8] * 0x100 + gReceiveCmd[9];
		if (WorkInterval < 5 && WorkInterval > 1800)
		{
			Debug_Serial.println("WorkInterval值值超阈值!!!强制置为5");
			WorkInterval = 5;
			gReceiveCmd[10] = 0x00;gReceiveCmd[11] = 0x05;
		}
		Debug_Serial.println(String("WorkInterval = ") + WorkInterval);
		if (!InitState.Save_WorkInterval(gReceiveCmd[8], gReceiveCmd[9]))
		{
			Debug_Serial.println("保存WorkInterval失败!!! <Set_Reporting_Interval_command>");
			Save_Success = false;
		}

		/*这里存非工作的时间间隔*/
		unsigned int StopInterval = gReceiveCmd[10] * 0x100 + gReceiveCmd[11];
		if (StopInterval < 15 && StopInterval > 7200)
		{
			Debug_Serial.println("StopInterval值超阈值!!!强制置为30");
			StopInterval = 30;
			gReceiveCmd[10] = 0x00;gReceiveCmd[11] = 0x1E;
		}
		Debug_Serial.println(String("StopInterval = ") + StopInterval);
		if (!InitState.Save_StopInterval(gReceiveCmd[10], gReceiveCmd[11]))
		{
			Debug_Serial.println("保存StopInterval失败!!! <Set_Reporting_Interval_command>");
			Save_Success = false;
		}

		/*这里上报服务器上报时间间隔指令接收回执*/
		if(Save_Success)
		{
			Message_Receipt.General_Receipt(SetIntervalOK, 2);
		}
		else
		{
			Message_Receipt.General_Receipt(SetIntervalErr, 2);
		}
	}
	else
	{
		Debug_Serial.println("不用理会的A007指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器设置LORA参数(A008)（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Set_Lora_parameter_command()
{
// 字节索引    	0        	1-2    	3       	4-5         	6          	7     	8       	9   	10-13   	14-17   	18  	19  	20-27   	28  	29~34        
// 数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	LoraMode	SYNC	TFREQ   	RFREQ   	TSF 	RSF 	Allocate	CRC8	frameEnd     
// 长度（byte）	1        	2      	1       	2           	1          	1     	1       	1   	4       	4       	1   	1   	8       	1   	6            
// 示例数据    	FE       	A008   	0x18(24)	C003        	00         	01    	F1      	12  	1C578DE0	1C03A180	09  	09  	0000000 	D6  	0D0A0D 0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
//61 0C3E4818 000000 FE A008 18 C003 00 55 F1 12 19DE5080 19CF0E40 09 09 0000000000000000 D6 0D0A0D0A0D0A  

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A008 <Set_Lora_parameter_command>");
		Debug_Serial.flush();

		/* 设置LoRa的通信模式 */
		if(LoRa_Para_Config.Save_LoRa_Com_Mode(gReceiveCmd[8]))
		{
			// Message_Receipt.General_Receipt(SetLoRaModeOk, 2);
			LoRa_MHL9LF.Parameter_Init(true);
			// Message_Receipt.Working_Parameter_Receipt(2);
		}
		else 
		{
			// Message_Receipt.General_Receipt(SetLoRaModeErr, 2);
			Serial.println("Set LoRa Mode Err! <Set_Lora_parameter_command>");
		}

		/* 设置SYNC */


		/* 设置TFREQ */
		// 487.1	1D088E60‬
		// unsigned char TFREQ_Array[4] = {0x19,0xCF,0x0E,0x40};
		// String Str_TFREQ = "";
		// for (size_t i = 0; i < sizeof(TFREQ_Array); i++)
		// {
		// 	if (TFREQ_Array[i] <= 0x0F)
		// 	{
		// 		Str_TFREQ += String(0,HEX);
		// 	}
		// 	Str_TFREQ += String(TFREQ_Array[i],HEX);
		// }
		// Debug_Serial.println("Str_TFREQ = " + Str_TFREQ);
		// const char* C_TFREQ = Str_TFREQ.c_str();

		// LoRa_MHL9LF.Param_Check(AT_TFREQ_, C_TFREQ, false);//433.000.000
		// Debug_Serial.println("设置TFREQ成功");

		/* 设置RFREQ */
		// 470.5	1C0B42A0‬
		// unsigned char RFREQ_Array[4] = {0x19,0xDE,0x50,0x80};
		// String Str_RFREQ = "";
		// for (size_t i = 0; i < sizeof(RFREQ_Array); i++)
		// {
		// 	if (RFREQ_Array[i] <= 0x0F)
		// 	{
		// 		Str_RFREQ += String(0,HEX);
		// 	}
		// 	Str_RFREQ += String(RFREQ_Array[i],HEX);
		// }
		// Debug_Serial.println("Str_RFREQ = " + Str_RFREQ);

		// const char* C_RFREQ = Str_RFREQ.c_str();
		// LoRa_MHL9LF.Param_Check(AT_RFREQ_, C_RFREQ, false);//434.000.000
		// Debug_Serial.println("设置RFREQ成功");

		/* 设置TSF */


		/* 设置RSF */


		/* Allocate */

	}
	else
	{
		Debug_Serial.println("不用理会的A008指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器设置正反转模式阈值（A009）（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Set_Forward_Reverse_mode_threshold()
{
//   数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	AI_Relation_Way	Threshold_multiple	IS_Reverse	CRC8	frameEnd     
//   字节索引    	0        	1-2    	3       	4-5         	6          	7     	8-11           	12-19             	20        	21  	22-27        
//   长度（byte）	1        	2      	1       	2           	1          	1     	4              	8                 	1         	1   	6            
//   示例数据    	FE       	A009   	0x11(17)	C003        	00         	01    	16324578       	121212121212      	00        	00  	0D0A0D0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A009 11 C003 00 55 16324578 1212121212121212 00 D6 0D0A0D0A0D0A  
	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A009 <Set_Forward_Reverse_mode_threshold>");
		Debug_Serial.flush();

		bool Illegal_AI_relation_Flag = false;//AI关联非法标志位
		bool Illegal_threshold_multiple_Flag = false;//阈值倍数非法标志位

		String Str_AI_Relation_Way = String(gReceiveCmd[8],HEX) + String(gReceiveCmd[9],HEX) + String(gReceiveCmd[10],HEX) + String(gReceiveCmd[11],HEX);
		Debug_Serial.println("Str_AI_Relation_Way = " + Str_AI_Relation_Way);

		unsigned char AI_Relation_Way_Array[8] = {0x00};
		// Debug_Serial.println(String("Str_AI_Relation_Way.length = ") + Str_AI_Relation_Way.length());
		Debug_Serial.print(">>>");
		for(int i=0; i < Str_AI_Relation_Way.length(); i++)
		{
			unsigned char x = Str_AI_Relation_Way.charAt(i);
			if(x >= '0' && x <= '8')
			{
				AI_Relation_Way_Array[i] = x - '0';
				// Debug_Serial.print(Str_AI_Relation_Way.charAt(i));
				Debug_Serial.print(AI_Relation_Way_Array[i]);
				Debug_Serial.print(" ");
			}
			else
			{
				Illegal_AI_relation_Flag = true;break;
			}
		}
		Debug_Serial.println("<<< AI关联");

		Debug_Serial.print(">>>");
		unsigned char Threshold_multiple_Array[8] = {0x00};
		for (size_t i = 0; i < 8; i++)
		{
			unsigned char x = gReceiveCmd[12+i];
			if(x >= 0 && x <= 100)
			{
				Threshold_multiple_Array[i] = x;
				Debug_Serial.print(Threshold_multiple_Array[i]);
				Debug_Serial.print(" ");
			}
			else
			{
				Illegal_threshold_multiple_Flag = true;break;
			}
		}
		Debug_Serial.println("<<< 阈值倍数");

		Debug_Serial.print(">>>");
		unsigned char IS_Reverse_Array[8] = {0x00};
		for (size_t i = 0; i < 8; i++)
		{
			if (gReceiveCmd[20] >> i)
			{
				IS_Reverse_Array[i] = 0x01;
			}
			Debug_Serial.print(IS_Reverse_Array[i]);
		}
		Debug_Serial.println("<<< 反向路数");

		if(Illegal_AI_relation_Flag)
		{
			Debug_Serial.println("AI关联非法!!! <Set_Forward_Reverse_mode_threshold>");
			Message_Receipt.Set_threshold_Receipt(3, gReceiveCmd, Illegal_AI_relation);
		}
		else if(Illegal_threshold_multiple_Flag)
		{
			Debug_Serial.println("阈值倍数非法!!! <Set_Forward_Reverse_mode_threshold>");
			Message_Receipt.Set_threshold_Receipt(3, gReceiveCmd, Illegal_threshold_multiple);
		}
		else
		{
			Pos_Nega_mode.Save_AI_Relation_Way(AI_Relation_Way_Array);//保存AI的关联
			Pos_Nega_mode.Save_WayIS_Reverse(IS_Reverse_Array);//保存是否反向的信息
			Pos_Nega_mode.Save_A009_Seted();//保存AI关联以及阈值倍数被设置

			/* 这里调用卷膜库的写入阈值函数 */


			Debug_Serial.println("AI关联以及阈值倍数设置成功... <Set_Forward_Reverse_mode_threshold>");

			Message_Receipt.Set_threshold_Receipt(3, gReceiveCmd, E009_Success);//
		}
	}
	else
	{
		Debug_Serial.println("不用理会的A009指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器发送正反转模式计算行程（A00A）（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Forward_Reverse_mode_Calculate_travel()
{
//   字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9   	10~15       
//   数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	WayUsed	CRC8	frameEnd    
//   长度（byte）	1        	2      	1      	2           	1          	1     	1      	1   	6           
//   示例数据    	FE       	A00A   	0x05(5)	C003        	00         	01    	0xFC   	00  	0D0A0D0A0D0A


	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A00A 05 C003 00 55 FC D6 0D0A0D0A0D0A  

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A00A <Forward_Reverse_mode_Calculate_travel>");
		Debug_Serial.flush();

		Device_Mode = Forward_Reverse_mode;//正反转模式

		A00A_WayUsed = gReceiveCmd[8];

		if (!Pos_Nega_mode.Read_A009_Seted())
		{
			Debug_Serial.println("ERROR!!!未进行AI关联以及设置阈值倍数 <Forward_Reverse_mode_Calculate_travel>");
			Message_Receipt.Calculate_travel_Receipt(3, A00A_WayUsed, A009_Seted_ERR);
		}
		else
		{
			for (unsigned char i = 0; i < 8; i++)
			{
				A00A_WayUsed_Array[i] = false;
			}
			
			// memset(A00A_WayUsed_Array,0,6*sizeof(bool));
			
			byte bitn = 7;byte BitRead = 0;
			for (unsigned char i = 0; i < 8; i++)
			{
				BitRead = ((A00A_WayUsed >> bitn) & 0x01);
				if(BitRead)
				{
					//表示第几路需要重置
					A00A_WayUsed_Array[i] = true;
					Debug_Serial.println(String("第") + i + "路需要进行重置");
				}
				bitn--;
			}

			// Forced_Stop(false);//强制停止
			
			// Calculate_travel_Flag = true;//计算行程标志位

			/* 这里加入卷膜库的重置行程参数 */


			Debug_Serial.println("正在开始计算行程... <Forward_Reverse_mode_Calculate_travel>");
			Message_Receipt.Calculate_travel_Receipt(3, A00A_WayUsed, Begin_Calculate_travel);

			/* 测试 */
			delay(2000);
			Message_Receipt.Calculate_travel_Receipt(3, A00A_WayUsed, complete_Calculate_travel);

			// Start_Timer4();//
		}
	}
	else
	{
		Debug_Serial.println("不用理会的A00A指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 服务器发送正反转模式开度控制指令（A00B）（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Forward_Reverse_mode_Opening_Control()
{
//   字节索引    	0        	1-2    	3       	4-5         	6          	  7   	8-15          	16      	17  	18~23       
//   数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	TargetOpenRate	interval	CRC8	frameEnd    
//   长度（byte）	1        	2      	1       	2           	1          	  1   	8             	1       	1   	6           
//   示例数据    	FE       	A00B   	0x0D（13）	C003        	00         	  01  	646464646464  	02      	00  	0D0A0D0A0D0A

	if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令
	//FE A00B 0D C003 00 55 6464646464646464 02 D6 0D0A0D0A0D0A  

	if (Verify_Frame_Validity(4, gReceiveCmd[3], true, false) == true)//第4个参数选择了false，不校验工作组号
	{
		Debug_Serial.println("A00B <Forward_Reverse_mode_Opening_Control>");
		Debug_Serial.flush();

		for (size_t i = 0; i < 8; i++)
		{
			if (gReceiveCmd[8+i] == 0xff)
			{
				Debug_Serial.println(String("第") + i + "路维持当前开度");
			}
			else if (gReceiveCmd[8+i] >= 0x00 && gReceiveCmd[8+i] <= 0x64)
			{
				Debug_Serial.println(String("第") + i + "路设置目标开度为" + gReceiveCmd[8+i]);
			}
			else if (gReceiveCmd[8+i] == 0xF0)
			{
				Debug_Serial.println(String("第") + i + "路强制全关");
			}
			else if (gReceiveCmd[8+i] == 0xF1)
			{
				Debug_Serial.println(String("第") + i + "路强制全开");
			}
			else
			{
				Debug_Serial.println(String("第") + i + "路设置的开度不存在，维持当前开度");
			}
		}
		
		unsigned char A00B_Interval = gReceiveCmd[16];
		Debug_Serial.println(String("每路开启的间隔时间为：") + A00B_Interval + "s");

		Message_Receipt.
	}
	else
	{
		Debug_Serial.println("不用理会的A00B指令!!!!");
	}
	
	memset(gReceiveCmd, 0x00, gReceiveLength);
}

//强制停止
void Command_Analysis::Forced_Stop(bool Need_Receipt)
{
	iwdg_feed();
	Stop_Timer4();//先停止计时
	rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？

	Cyclic_intervalFlag = false;//循环时间间隔的标志位
	gRTCTime_arrive_Flag = false;//RTC时间到达的标志位
	gTime_arrive_Flag = false; //定时器时间到达的标志位
	Cyclic_timing = false;//正在进行循环时间间隔的标志位
	Irrigation_use = false;//正在灌溉的标志位
	//OpenSec = 0;//开启的时间
	//DO_Interval = 0;//单个的间隔时间
	//DO_Num = 0;//一次性开启的DO数量
	//retryCnt = 0;//循环次数（）
	Cyclic_interval = 0;//循环间隔时间
	//fornum = 0;//一轮循环需要的循环次数，（例如开4路，每次开2路，fornum=2）
	//fornum_backups = 0;//一轮循环需要的循环次数的备份，（例如开4路，每次开2路，fornum=2）
	//Last_full = false;//最后一轮循环是否能开满，（例如开5路，每次开2路，最后一轮开不满）
	//Last_num = 0;//最后一轮循环开不满时需要开启的个数
	DO_intervalFlag = false;//单个间隔时间的标志位
	DO_interval_timing = false;//正在进行单个间隔时间的标志位
	DOStatus_Change = false;//DO的状态改变标志位
	One_Cycle_Complete = false;//一轮循环完成的标志位

	Need_Num = 0;	//需要开启继电器的个数
	Complete_Num = 0;//完成开启的个数

	KeyDI7_num = 0;//将按键按下次数清零，就可以让E014一直上报
	Waiting_Collection = true;//下次回复状态需要等待采集

	/* 延时模式相关 */
	Delay_mode_OpenSec = 0;//
	Delay_mode_Interval = 0;//
	Delay_mode_DONum = 0;//
	Delay_mode_NeedWait = false;//
	Delay_mode_OpenNum = 0;//
	Delay_mode_EndWait = 0;//

	/* 正反转模式相关 */
	Forward_Reverse_mode_NeedWait = false;//需要进行延时标志位
	Forward_Reverse_mode_EndWait = 0;//结束等待的时间

#if PLC_V1
	for (size_t i = 0; i < 12; i++)
	{
		/* 延时模式相关 */
		Delay_mode_Worktime[i] = 0;

		Worktime[i] = 0;//16个开启时间
		WorkInterval[i] = 0;//16个间隔时间
		Worktime_backups[i] = 0;//16个开启时间的备份
		WorkInterval_backups[i] = 0;//16个间隔时间的备份
		retryCnt[i] = 0;//循环次数（）
		DO_WayOpentime[i] = 0;//16路DO的开始时间
		DO_WayClosetime[i] = 0;//16路DO的关闭时间
		DO_WayIntervalBengin[i] = 0;//16路DO的间隔开始时间
		DO_WayIntervalEnd[i] = 0;//16路DO的间隔结束时间
		DO_WayOpen[i] = false; //16路DO打开的标志位
		DO_WayInterval[i] = false;//16路DO间隔标志位
		DO_WayComplete[i] = false;//16路DO完成的标志位
		DO_Set[i] = false;//DO设置的标志位
		Set_DO_relay(i, OFF);

		/* 正反转模式相关 */
		Forward_Reverse_mode_Worktime[i] = 0;//
	}

	/*这里上报强制停止指令接收回执*/
	if (Need_Receipt)
	{
		Message_Receipt.General_Receipt(TrunOffOk, 3);
	}

#elif PLC_V2
	Serial.println("");

#endif
}
 