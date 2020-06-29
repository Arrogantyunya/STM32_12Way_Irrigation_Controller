#include "receipt.h"
#include "User_CRC8.h"
#include <libmaple/iwdg.h>
#include "LoRa.h"
#include "Set_coil.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "fun_periph.h"
#include "public.h"
#include "Set_coil.h"
#include "Private_Timer.h"

Receipt Message_Receipt;

// #define CLEAR_BUFFER_FLAG   0 //是否清除服务器数据

#define SEND_DATA_DELAY     200 //发送完一帧数据后延时时间（ms）

// unsigned char gMotorStatus = MotorFactoryMode;  //每次开机默认状态未初始化
//unsigned char x = 0x00;
unsigned char gStatus_Working_Parameter_Receipt = 0x00;//E014实时工作状态回执的状态
unsigned char gStatus_E014 = DefaultValue;	//E014通用回执的状态
unsigned char gLoRaCSQ[4] = { 0 };  //接收LoRa发送和接收的信号强度
extern unsigned char G_modbusPacket[20];
extern int G_modbusPacket_Length;
extern void Solenoid_mode_DO_ON(void);	//灌溉分时开启V3
extern void Key_cycle_irrigationV3(void);//按键启动循环灌溉v3

volatile bool gStateReportFlag = false;

 /*
  @brief   : 清除服务器上一次接收的LoRa数据缓存
  @param   : 无
  @return  : 无
 */
 void Receipt::Clear_Server_LoRa_Buffer(void)
 {
 	/*发送一帧帧尾，让服务器认为接收到了完成数据，清空缓存*/
 	unsigned char Buffer[6] = { 0x0D, 0x0A, 0x0D, 0x0A, 0x0D, 0x0A };
 	LoRa_Serial.write(Buffer, 6);
 	delay(SEND_DATA_DELAY);
 }

/*
 @brief   : 随机生成回执消息的回执发送微秒延时
 @param   : 随机延时（ms）
 @return  : 无
 */
void Receipt::Receipt_Random_Wait_Value(unsigned long int *random_value)
{
	unsigned char RandomSeed;
	SN.Read_Random_Seed(&RandomSeed); //读取随机数种子
	/*RandomSeed * 1ms, 1.5S + RandomSeed * 0.1ms, 200ms*/
	*random_value = random(RandomSeed * 1000, 1500000) + random(RandomSeed * 100, 200000);
}

/*
 @brief   : 上报本设备通用设置参数,包括区域号、SN码、子设备路数、工作组号、采集间隔等（本机 ---> 服务器）
			该回执帧也用作设备第一次申请注册服务器。
 @param   : 无
 @return  : 无
 */
void Receipt::Report_General_Parameter(void)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8-12   	13-21   	22     	23-24   	25-31         	32-39   	40  	41-45        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	GroupID	SN      	channel	Interval	RTC           	Allocata	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	5      	9       	1      	2       	7             	8       	1   	6            
// 示例数据    	FE       	E001   	0x24   	C003        	00         	01    	XXXXXX 	C003XXXX	01     	0000    	20200501121010	        	00  	0D0A0D 0A0D0A


	unsigned char ReportFrame[64] = { 0 };
	unsigned char FrameLength = 0;
	unsigned char DataTemp[10];
	unsigned long int RandomSendInterval = 0;

	Receipt_Random_Wait_Value(&RandomSendInterval);
	delayMicroseconds(RandomSendInterval);
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReportFrame[FrameLength++] = 0xFE; //帧头
	ReportFrame[FrameLength++] = 0xE0; //帧ID
	ReportFrame[FrameLength++] = 0x11;
	ReportFrame[FrameLength++] = 0x24; //帧有效数据长度

	/*设备类型*/
	ReportFrame[FrameLength++] = highByte(DEVICE_TYPE_ID);
	ReportFrame[FrameLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReportFrame[FrameLength++] = 0x55 : ReportFrame[FrameLength++] = 0x00;
	/*区域号*/
	ReportFrame[FrameLength++] = SN.Read_Area_Number();

	/*组号*/
	SN.Read_Group_Number(&DataTemp[0]);
	for (unsigned char i = 0; i < 5; i++)
		ReportFrame[FrameLength++] = DataTemp[i];

	/*SN码*/
	SN.Read_SN_Code(&DataTemp[0]);
	for (unsigned char i = 0; i < 9; i++)
		ReportFrame[FrameLength++] = DataTemp[i];

	/*路数*/
	ReportFrame[FrameLength++] = 0x01; //卷膜机默认只有一路

	/*采集状态间隔*/
	ReportFrame[FrameLength++] = 0x00;
	ReportFrame[FrameLength++] = 0x00;

	/*RTC*/
	for (unsigned char i = 0; i < 7; i++)
		ReportFrame[FrameLength++] = 0x00;

	/*预留的8个字节*/
	for (unsigned char i = 0; i < 8; i++)
		ReportFrame[FrameLength++] = 0x00;

	/*CRC8*/
	ReportFrame[FrameLength++] = GetCrc8(&ReportFrame[4], ReportFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReportFrame[FrameLength++] = 0x0D : ReportFrame[FrameLength++] = 0x0A;

	Info_Println("Report general parameter...");
	/*打印要发送的数据帧*/
	Print_Debug(&ReportFrame[0], FrameLength);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&ReportFrame[0], FrameLength);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 当本地工作组号丢失，向服务器申请本机的工作组号（本设备 ---> 服务器）
 @param   : 无
 @return  : 无
 */
void Receipt::Request_Set_Group_Number(void)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9   	10~15        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	channel	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	1   	6            
// 示例数据    	FE       	E012   	0x05   	C003        	00         	01    	01     	00  	0D0A0D 0A0D0A


	unsigned char RequestFrame[20] = { 0 };
	unsigned char FrameLength = 0;
	// unsigned char RandomSeed;
	unsigned long int RandomSendInterval = 0;

	Receipt_Random_Wait_Value(&RandomSendInterval);
	delayMicroseconds(RandomSendInterval);
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	RequestFrame[FrameLength++] = 0xFE; //帧头
	RequestFrame[FrameLength++] = 0xE0; //帧ID
	RequestFrame[FrameLength++] = 0x12;
	RequestFrame[FrameLength++] = 0x05; //帧有效数据长度
	
	/*设备ID*/
	RequestFrame[FrameLength++] = highByte(DEVICE_TYPE_ID);
	RequestFrame[FrameLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? RequestFrame[FrameLength++] = 0x55 : RequestFrame[FrameLength++] = 0x00;
	
	/*区域号*/
	RequestFrame[FrameLength++] = SN.Read_Area_Number();
	
	/*设备路数*/
	RequestFrame[FrameLength++] = 0x01;
	
	/*CRC8*/
	RequestFrame[FrameLength++] = GetCrc8(&RequestFrame[4], RequestFrame[3]);
	
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? RequestFrame[FrameLength++] = 0x0D : RequestFrame[FrameLength++] = 0x0A;

	Info_Println("Requeset SN code to access network...");
	/*打印要发送的数据帧*/
	Print_Debug(&RequestFrame[0], FrameLength);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&RequestFrame[0], FrameLength);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 当本地SN码丢失，向服务器申请本机的SN码（本设备 ---> 服务器）
 @param   : 无
 @return  : 无
 */
void Receipt::Request_Device_SN_and_Channel(void)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9   	10~15        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	channel	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	1   	6            
// 示例数据    	FE       	E013   	0x05   	C003        	00         	01    	       	00  	0D0A0D 0A0D0A


	unsigned char RequestFrame[20] = { 0 };
	unsigned char FrameLength = 0;

	unsigned long int RandomSendInterval = 0;

	Receipt_Random_Wait_Value(&RandomSendInterval);
	delayMicroseconds(RandomSendInterval);
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	RequestFrame[FrameLength++] = 0xFE; //帧头
	RequestFrame[FrameLength++] = 0xE0; //帧ID
	RequestFrame[FrameLength++] = 0x13;
	RequestFrame[FrameLength++] = 0x05; //帧有效数据长度

	/*设备ID*/
	RequestFrame[FrameLength++] = highByte(DEVICE_TYPE_ID);
	RequestFrame[FrameLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? RequestFrame[FrameLength++] = 0x55 : RequestFrame[FrameLength++] = 0x00;
	
	/*区域号*/
	RequestFrame[FrameLength++] = SN.Read_Area_Number();
	
	/*设备路数*/
	RequestFrame[FrameLength++] = 0x01;
	
	/*CRC8*/
	RequestFrame[FrameLength++] = GetCrc8(&RequestFrame[4], RequestFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? RequestFrame[FrameLength++] = 0x0D : RequestFrame[FrameLength++] = 0x0A;

	Info_Println("Requeset SN code to access network...");
	/*打印要发送的数据帧*/
	Print_Debug(&RequestFrame[0], FrameLength);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&RequestFrame[0], FrameLength);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 上报实时详细工作状态（本机 ---> 服务器）
 @param   : 上报次数
 @return  : 无
 */
void Receipt::Working_Parameter_Receipt(bool use_random_wait, unsigned char times)
{
// 字节索引    	0        	1-2    	3      	4-5         	6     	7-8  	9-10 	11  	12  	13-20	21-28	29-44	45-60	61-62	63      	64-65	66  	67-72        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	status	swVer	hwVer	SNR 	RSSI	DI   	DO   	AI   	AO   	VOL  	LoraMode	符号位  	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1     	2    	2    	1   	1   	8    	8    	16   	16   	2    	1       	2    	1   	6            
// 示例数据    	FE       	E014   	0x3E   	C003        	00    	0200 	0200 	    	    	     	     	     	     	     	        	     	00  	0D0A0D 0A0D0A

	iwdg_feed();
	Info_Println("上报实时详细工作状态 <Working_Parameter_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[72] = { 0x00 };
	unsigned char ReceiptLength = 0;

	unsigned long int RandomSendInterval = 0;

	iwdg_feed();
	if (use_random_wait)
	{
		//随机等待一段时间后再发送，避免大量设备发送堵塞。
		Receipt_Random_Wait_Value(&RandomSendInterval);
		delayMicroseconds(RandomSendInterval);
	}
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x14; //
	ReceiptFrame[ReceiptLength++] = 0x3E; //数据长度62

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*status*/
	Info_Println(String("gStatus_E014 = ") + gStatus_E014);
	Debug_Serial.flush();
	ReceiptFrame[ReceiptLength++] = gStatus_E014;

	/*swVer*/
	ReceiptFrame[ReceiptLength++] = Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR);
	ReceiptFrame[ReceiptLength++] = Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1);

	/*hwVer*/
	ReceiptFrame[ReceiptLength++] = Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR);
	ReceiptFrame[ReceiptLength++] = Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1);
	
	/*SNR 和 RSSI*/
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[0];  //信号发送强度
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[1];  //信号接收强度
	//ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(gLoRaCSQ[0]);  //信号发送强度
	//ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(gLoRaCSQ[1]);  //信号接收强度

#if PLC_V1
	/*DI	8*/
	ReceiptFrame[ReceiptLength++] = Modbus_Coil.Get_DI_1to8();
	for (size_t i = 0; i < 7; i++)
	{
		ReceiptFrame[ReceiptLength++] = 0x00;
	}

	/*DO	8*/
	ReceiptFrame[ReceiptLength++] = Modbus_Coil.Get_DO_1to8();
	ReceiptFrame[ReceiptLength++] = Modbus_Coil.Get_DO_9to16();
	for (size_t i = 0; i < 6; i++)
	{
		ReceiptFrame[ReceiptLength++] = 0x00;
	}

	/*AI	16*/
	unsigned char* p = Modbus_Coil.Get_AI_1to8();
	for (size_t i = 0; i < 16; i++)
	{
		ReceiptFrame[ReceiptLength++] = *(p + i);
	}
	/*AO	16*/
	for (size_t i = 0; i < 16; i++)
	{
		ReceiptFrame[ReceiptLength++] = 0x00;
	}

	/*VOL*/
	ReceiptFrame[ReceiptLength++] = 0x09;
	ReceiptFrame[ReceiptLength++] = 0x60;//0x0960=2400mv

	/*LoraMode*/
	ReceiptFrame[ReceiptLength++] = LoRa_Para_Config.Read_LoRa_Com_Mode();

	/* 信号的符号位 */
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[2];  //接收信号强度符号
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[3];  //接收信号强度符号

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}

#elif PLC_V2

#endif
}

/*
 @brief   : 发送通用回执信息给服务器。（本设备 ---> 服务器）
			在大多数情况下，当接受到服务器的指令后，需要发送本条通用回执
 @param   : 1.回执状态
			2.回执次数
 @return  : 无
 */
void Receipt::General_Receipt(unsigned char status, unsigned char send_times)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9     	10-17   	18  	19~24        
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	channel	Status	Allocate	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	1     	8       	1   	6            
// 示例数据    	FE       	E015   	0x0E   	C003        	00         	01    	       	      	00000   	00  	0D0A0D 0A0D0A


	unsigned char ReceiptFrame[25] = { 0 };
	unsigned char ReceiptLength = 0;
	// unsigned char RandomSeed;
	unsigned long int RandomSendInterval = 0;

	Receipt_Random_Wait_Value(&RandomSendInterval);
	delayMicroseconds(RandomSendInterval);
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x15;
	ReceiptFrame[ReceiptLength++] = 0x0E; //帧有效数据长度
	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/*路数*/
	ReceiptFrame[ReceiptLength++] = 0x01;

	/*回执状态*/
	ReceiptFrame[ReceiptLength++] = status;

	/*预留的8个字节*/
	for (unsigned char i = 0; i < 8; i++)
		ReceiptFrame[ReceiptLength++] = 0x00;

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("Send General Receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(ReceiptFrame, ReceiptLength);
		delay(SEND_DATA_DELAY);
	}
}

/*
 @brief   : 发送通用控制器Modbus控制指令回执信息给服务器。（本设备 ---> 服务器）
 @param   :	1.使用随机延时
			2.回执次数，send_times
 @return  : 无
 */
void Receipt::Control_command_Receipt(bool use_random_wait, unsigned char send_times)
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8      	9-n         	    	             
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	zoneId	groupId	modbusPacket	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1      	n           	1   	6            
// 示例数据  	FE       	E000   	5+N    	C003        	00         	01    	01     	XXXXXXXX    	00  	0D0A0D 0A0D0A


	unsigned char ReceiptFrame[50] = { 0 };
	unsigned char ReceiptLength = 0;
	// unsigned char RandomSeed;
	unsigned long int RandomSendInterval = 0;

	iwdg_feed();
	if (use_random_wait)
	{
		//随机等待一段时间后再发送，避免大量设备发送堵塞。
		Receipt_Random_Wait_Value(&RandomSendInterval);
		delayMicroseconds(RandomSendInterval);
	}
	iwdg_feed();

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x00;
	ReceiptFrame[ReceiptLength++] = G_modbusPacket_Length + 5; //帧有效数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	//gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;
	ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/*组ID*/
	ReceiptFrame[ReceiptLength++] = 0x01;

	/*设备回执的字段*/
	for (unsigned char i = 0; i < G_modbusPacket_Length; i++)
	{
		ReceiptFrame[ReceiptLength++] = G_modbusPacket[i];
	}

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	G_modbusPacket_Length = 0;//每次用完都得清零

	Info_Println("Send Control command Receipt...<Control_command_Receipt>");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(ReceiptFrame, ReceiptLength);
		delay(SEND_DATA_DELAY);
	}
}

/*
 @brief   : 发送设置初始状态回执信息给服务器。（本设备 ---> 服务器）
 @param   : 1.回执状态，status
			2.回执次数，send_times
			3.随机ID第1位，randomId_1
			4.随机ID第2位，randomId_2
			5.485设备回执数组的指针，R_Modbus_Instructions
			6.485设备回执数组的长度，R_Modbus_Length
 @return  : 无
 */
void Receipt::Output_init_Receipt(unsigned char status, unsigned char send_times)
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8     	9  		10-15        
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	IsBroadcast	ZoneId	status	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	1     	1   	6            
// 示例数据  	FE       	E008   	0x01   	C003        	55         	01    	00    	D6  	0D0A0D 0A0D0A

	unsigned char ReceiptFrame[50] = { 0 };
	unsigned char ReceiptLength = 0;

	iwdg_feed();

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x08;
	ReceiptFrame[ReceiptLength++] = 0x01; //帧有效数据长度

	/*回执状态*/
	ReceiptFrame[ReceiptLength++] = status;

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("Send Output init Receipt...<Output_init_Receipt>");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(ReceiptFrame, ReceiptLength);
		delay(SEND_DATA_DELAY);
	}
}

/*
 @brief   : 发送灌溉控制回执信息给服务器。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
			2.接收到的A003数组
 @return  : 无
 */
void Receipt::Irrigation_control_Receipt(unsigned char send_times, unsigned char* gReceiveCmd)
{
	  /*| 字节索引	| 0			| 1 - 2		| 3			| 4 - 5			| 6				| 7			| 8-9		| 10 - 41	| 42 - 71	| 72-73		| 74-75	| 76-107	| 108	| 109 - 114     |
		| 数据域	| FrameHead | FrameId	| DataLen	| DeviceTypeId	| IsBroadcast	| zoneId	| randomId	| openSec	| interval	| timeout	| DOUsed| retryCnt	| CRC8	| FrameEnd      |
		| 长度		| 1			| 2			| 1			| 2				| 1				| 1			| 2			| 32		| 30		| 2			| 2		| 32		| 1		| 6				|
		| 示例数据	| FE		| E003		| 0x68(104)	| C003			| 00			| 01		| 1234		| 0005		| 0003		| 001e		| FF00	| 0005		| D6	| 0D0A0D0A0D0A	|*/
	unsigned char ReceiptFrame[115] = { 0 };
	unsigned char ReceiptLength = 0;
	// unsigned char RandomSeed;
	// unsigned long int RandomSendInterval = 0;

	//Receipt_Random_Wait_Value(&RandomSendInterval);
	//delayMicroseconds(RandomSendInterval);
	iwdg_feed();

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x03;
	ReceiptFrame[ReceiptLength++] = 0x04; //帧有效数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;
	
	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	// /*openSec*/
	// for (size_t i = 0; i < 32; i++)
	// {
	// 	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	// }

	// /*interval*/
	// for (size_t i = 0; i < 30; i++)
	// {
	// 	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	// }

	// /*timeout*/
	// ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	// ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];

	// /*DOUsed*/
	// ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	// ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];

	// /*retryCnt*/
	// for (size_t i = 0; i < 32; i++)
	// {
	// 	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	// }

	///*回执状态*/
	//ReceiptFrame[ReceiptLength++] = status;
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("Send Irrigation control Receipt...<Irrigation_control_Receipt>");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(ReceiptFrame, ReceiptLength);
		delay(SEND_DATA_DELAY);
	}
}

/*
 @brief   : 发送延时开启回执信息给服务器。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
			2.接收到的A004数组
 @return  : 无
 */
void Receipt::Delay_Start_DO_Control_Receipt(unsigned char send_times, unsigned char* gReceiveCmd)
{
// 字节索引  	0        	1-2    	3      	4-5         	6          	7     	8-9    	10      	11   	12-13 	14  	15~20        
// 数据域   	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	openSec	interval	DONum	DOUsed	CRC8	frameEnd     
// 长度（byte）	1        	2      	1      	2           	1          	1     	2      	1       	1    	2     	1   	6            
// 示例数据  	FE       	E004   	0A     	C003        	00         	01    	003C   	30      	2    	FF00  	D6  	0D0A0D 0A0D0A

	unsigned char ReceiptFrame[50] = { 0 };
	unsigned char ReceiptLength = 0;
	unsigned char x = 8;
	// unsigned char RandomSeed;
	// unsigned long int RandomSendInterval = 0;

	//Receipt_Random_Wait_Value(&RandomSendInterval);
	//delayMicroseconds(RandomSendInterval);
	iwdg_feed();

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x04;
	ReceiptFrame[ReceiptLength++] = 0x0A; //帧有效数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;
	
	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/*openSec*/
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];

	/*interval*/
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];

	/* DONum */
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];

	/*DOUsed*/
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("Send Irrigation control Receipt...<Irrigation_control_Receipt>");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(ReceiptFrame, ReceiptLength);
		delay(SEND_DATA_DELAY);
	}
}

/*
 @brief   : 发送正反转回执信息给服务器。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
			2.接收到的A005数组
 @return  : 无
 */
void Receipt::Positive_negative_Control_Receipt(unsigned char send_times, unsigned char* gReceiveCmd)
{
// 字节索引    	0        	1-2    	3       	4-5         	6          	7     	8-19    	20      	21-22    	23  	24~29        
// 数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	WorkTime	interval	DOUsed   	CRC8	frameEnd     
// 长度（byte）	1        	2      	1       	2           	1          	1     	12      	1       	2        	1   	6            
// 示例数据    	FE       	E005   	0x13（19）	C003        	00         	01    	0A0A0A0A	03      	5540/AA00	00  	0D0A0D 0A0D0A

	unsigned char ReceiptFrame[50] = { 0 };
	unsigned char ReceiptLength = 0;
	unsigned char x = 8;

	iwdg_feed();

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x05;
	ReceiptFrame[ReceiptLength++] = 0x13; //帧有效数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;
	
	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/*WorkTime*/
	for (size_t i = 0; i < 12; i++)
	{
		ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	}
	
	/*interval*/
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];

	/*DOUsed*/
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("Send Irrigation control Receipt...<Irrigation_control_Receipt>");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(ReceiptFrame, ReceiptLength);
		delay(SEND_DATA_DELAY);
	}
}

/*
 @brief   : E006新实时状态回执。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
 @return  : 无
 */
void  Receipt::New_Working_Parameter_Receipt(bool use_random_wait,unsigned char send_times)
{
// 字节索引  		0        	1-2   	3       	4-5         	6-9     	10-13   	14-17 	18  	19-24            
// 数据域   		frameHead	frameId	dataLen 	DeviceTypeId	DI      	DO      	AI    	CRC8	frameEnd         
// 长度（byte）		1        	2      	1       	2           	4       	4       	16    	1   	6                
// 示例数据  		FE       	E006   	0x1A（26）	C003        	FF000000	FF000000	XXXXXX	00  	0D0A0D0A0D0A

	iwdg_feed();
	Info_Println("上报新实时详细工作状态 <New_Working_Parameter_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;
	unsigned long int RandomSendInterval = 0;

	iwdg_feed();
	if (use_random_wait)
	{
		//随机等待一段时间后再发送，避免大量设备发送堵塞。
		Receipt_Random_Wait_Value(&RandomSendInterval);
		delayMicroseconds(RandomSendInterval);
	}
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x06; //
	ReceiptFrame[ReceiptLength++] = 0x1A; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

#if PLC_V1
	/*DI	4*/
	ReceiptFrame[ReceiptLength++] = Modbus_Coil.Get_DI_1to8();
	for (size_t i = 0; i < 3; i++)
	{
		ReceiptFrame[ReceiptLength++] = 0x00;
	}

	/*DO	4*/
	ReceiptFrame[ReceiptLength++] = Modbus_Coil.Get_DO_1to8();
	ReceiptFrame[ReceiptLength++] = Modbus_Coil.Get_DO_9to16();
	for (size_t i = 0; i < 2; i++)
	{
		ReceiptFrame[ReceiptLength++] = 0x00;
	}

	/*AI	16*/
	unsigned char* p = Modbus_Coil.Get_AI_1to8();
	for (size_t i = 0; i < 16; i++)
	{
		ReceiptFrame[ReceiptLength++] = *(p + i);
	}

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}

#elif PLC_V2

#endif
}

/*
 @brief   : E007信号质量与版本号回执。（本设备 ---> 服务器）
 @param   :	1.是否启用随机等待
 			2.回执次数，send_times
 @return  : 无
 */
void Receipt::SignalQuality_Version_Receipt(bool use_random_wait,unsigned char send_times)//E007信号质量与版本号回执
{
// 字节索引    	0        	1-2    	3       	4-5         	6          	7     	8-9     	11-12     	13-14       	15-16       	17      	18  	19~24        
// 数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	SNR     	RSSI      	SoftwareVer 	HardwareVer 	LoraMode	CRC8	frameEnd     
// 长度（byte）	1        	2      	1       	2           	1          	1     	2       	2         	2           	2           	1       	1   	6            
// 示例数据    	FE       	E007   	0x0D（13）	C003        	00         	01    	05E0（+5）	80F0（-128）	0202（V2.0.2）	0220（V2.2.0）	        	D6  	0D0A0D 0A0D0A


	iwdg_feed();
	Info_Println("上报信号强度以及版本号 <SignalQuality_Version_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;
	unsigned long int RandomSendInterval = 0;

	iwdg_feed();
	if (use_random_wait)
	{
		//随机等待一段时间后再发送，避免大量设备发送堵塞。
		Receipt_Random_Wait_Value(&RandomSendInterval);
		delayMicroseconds(RandomSendInterval);
	}
	iwdg_feed();

	/*读取LoRa模块的 SNR and RSSI*/
	LoRa_MHL9LF.LoRa_AT(gLoRaCSQ, true, AT_CSQ_, 0);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x07; //
	ReceiptFrame[ReceiptLength++] = 0x0D; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/*SNR and RSSI*/
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[0];  //SNR信噪比
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[2];  //符号位
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[1];  //RSSI信号质量
	ReceiptFrame[ReceiptLength++] = gLoRaCSQ[3];  //符号位

	/*swVer*/
	ReceiptFrame[ReceiptLength++] = Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR);
	ReceiptFrame[ReceiptLength++] = Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1);

	/*hwVer*/
	ReceiptFrame[ReceiptLength++] = Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR);
	ReceiptFrame[ReceiptLength++] = Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1);

	/* LoraMode */
	ReceiptFrame[ReceiptLength++] = LoRa_Para_Config.Read_LoRa_Com_Mode();

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
}


/*
 @brief   : E009设置正反转模式阈值回执。（本设备 ---> 服务器）
 @param   :	1.是否启用随机等待
 			2.回执次数，send_times
 @return  : 无
 */
void Receipt::Set_threshold_Receipt(unsigned char send_times,unsigned char* gReceiveCmd,unsigned char E009status)
{
//数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	AI_Relation_Way	Threshold_multiple	Is_Reverse	E009_Status	CRC8	frameEnd     
//字节索引    	0        	1-2    	3       	4-5         	6          	7     	8-11           	12-19             	20        	21         	22  	23-28        
//长度（byte）	1        	2      	1       	2           	1          	1     	4              	8                 	1         	1          	1   	6            
//示例数据    	FE       	E009   	0x12(18)	C003        	00         	01    	16324578       	1212121212121212  	00        	01         	D6  	0D0A0D0A0D0A


	iwdg_feed();
	Info_Println("正反转模式阈值回执 <Set_threshold_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;
	unsigned char x = 8;

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x09; //
	ReceiptFrame[ReceiptLength++] = 0x12; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/*AI_Relation_Way*/
	for (size_t i = 0; i < 4; i++)
	{
		ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	}

	/*Threshold_multiple*/
	for (size_t i = 0; i < 8; i++)
	{
		ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	}

	/*Is_Reverse*/
	ReceiptFrame[ReceiptLength++] = gReceiveCmd[x++];
	
	/* E009status */
	ReceiptFrame[ReceiptLength++] = E009status;

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
}

/*
 @brief   : E00A计算行程与正反转AI回执。（本设备 ---> 服务器）
 @param   :	1.是否启用随机等待
 			2.回执次数，send_times
 @return  : 无
 */
void Receipt::Calculate_travel_Receipt(unsigned char send_times,unsigned char WayUsed,unsigned char E00Astatus)
{
//   字节索引    	0        	1-2    	3      	4-5         	6          	7     	8      	9          	10  	11-16       
//   数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	WayUsed	E00A_Status	CRC8	frameEnd    
//   长度（byte）	1        	2      	1      	2           	1          	1     	1      	1          	1   	6           
//   示例数据    	FE       	E00A   	0x06(6)	C003        	00         	01    	FC     	02         	00  	0D0A0D0A0D0A

	iwdg_feed();
	Info_Println("正反转模式计算行程回执 <Calculate_travel_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x0A; //
	ReceiptFrame[ReceiptLength++] = 0x06; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/* WayUsed */
	ReceiptFrame[ReceiptLength++] = WayUsed;

	/* E00Astatus */
	ReceiptFrame[ReceiptLength++] = E00Astatus;
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
}

/*
 @brief   : E00B卷膜开度设置回执。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
			2.状态
 @return  : 无
 */
void Receipt::Opening_Control_Receipt(unsigned char send_times, unsigned char E00Bstatus, unsigned char *TargetOpenRate)//EOOB开度控制回执
{
//   字节索引    	0        	1-2    	3       	4-5         	6          	7     	8     	9-16           	17-32	33  	34~39       
//   数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	Status	current_opening	AI   	CRC8	frameEnd    
//   长度（byte）	1        	2      	1       	2           	1          	1     	1     	8              	16   	1   	6           
//   示例数据    	FE       	E00B   	0x1D（29）	C003        	00         	01    	01    	646464646464   	     	00  	0D0A0D0A0D0A

	iwdg_feed();
	if (E00Bstatus == Auto_Report)
	{
		Info_Println("卷膜开度自动上报 <Opening_Control_Receipt>");
		
	}
	else
	{
		Info_Println("卷膜开度设置回执 <Opening_Control_Receipt>");
	}
	
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x0B; //
	ReceiptFrame[ReceiptLength++] = 0x1D; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/* E00Bstatus */
	ReceiptFrame[ReceiptLength++] = E00Bstatus;

	/* current_opening */
	if (E00Bstatus == Auto_Report)
	{
		/* 这里放入查询开度的代码 */
		for (unsigned char i = 0; i < 8; i++)
		{
			/* 开度查询函数，提供实时开度值 */
			// film_u8 Film_Read_RT_Opening_CH(film_u8 ch);
			if (i < MOTOR_CHANNEL)
				ReceiptFrame[ReceiptLength++] = Film_Read_RT_Opening_CH(i+1);
			else
				ReceiptFrame[ReceiptLength++] = 0xFE;//表示不存在的路数
		}
	}
	else
	{
		/* 这里是上报目标开度 */
		for (unsigned char i = 0; i < 8; i++)
		{
			ReceiptFrame[ReceiptLength++] = TargetOpenRate[i];
		}
	}
	
	/* AI */
	unsigned short AI_data = 0;
	for (unsigned char i = 0; i < 8; i++)
	{
		AI_data = Modbus_Coil.Get_which_AI(Pos_Nega_mode.Read_AI_Relation_Way(i));
		// Serial.println(String("AI_data = ") + AI_data);
		ReceiptFrame[ReceiptLength++] = (AI_data >> 8) & 0xFF;
		ReceiptFrame[ReceiptLength++] = AI_data & 0xFF;
	}
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
}

/*
 @brief   : E00C卷膜路数状态回执。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
 			2.路数
			3.状态
 @return  : 无
 */
void Receipt::Rolling_ch_Status_Receipt(unsigned char send_times,unsigned char ch, unsigned char E00Cstatus)
{
//   字节索引    	0        	1-2    	3      	4-5         	6          	7     	8   	9     	10  	11-16       
//   数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	ch  	status	CRC8	frameEnd    
//   长度（byte）	1        	2      	1      	2           	1          	1     	1   	1     	1   	6           
//   示例数据    	FE       	E00C   	0x06   	C003        	00         	01    	01  	      	00  	0D0A0D0A0D0A


	iwdg_feed();
	Info_Println("卷膜路数状态回执 <Rolling_ch_Status_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x0C; //
	ReceiptFrame[ReceiptLength++] = 0x06; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/* Channel */
	ReceiptFrame[ReceiptLength++] = ch;

	/* E00Cstatus */
	ReceiptFrame[ReceiptLength++] = E00Cstatus;
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
	Info_Println("");
}


/*
 @brief   : E00D卷膜电流倍数设置回执。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
 			2.路数
			3.状态
 @return  : 无
 */
void Receipt::TimesSet_ClearEleCurrent_Receipt(unsigned char send_times, unsigned char E00Dstatus)
{
// 字节索引    	0        	1-2    	3      	4-5         	6          	7     	8     	9   	10-15       
// 数据域     	frameHead	frameId	dataLen	DeviceTypeId	isBroadcast	ZoneId	Status	CRC8	frameEnd    
// 长度（byte）	1        	2      	1      	2           	1          	1     	1     	1   	6           
// 示例数据    	FE       	E00D   	0x05   	C003        	00         	01    	00    	00  	0D0A0D0A0D0A

	iwdg_feed();
	Info_Println("卷膜电流倍数设置回执 <TimesSet_ClearEleCurrent_Receipt>");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x0D; //
	ReceiptFrame[ReceiptLength++] = 0x05; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/* E00Dstatus */
	ReceiptFrame[ReceiptLength++] = E00Dstatus;
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
	Info_Println("");
}


/*
 @brief   : E00E卷膜开度状态全回执。（本设备 ---> 服务器）
 @param   :	1.回执次数，send_times
 @return  : 无
 */
void Receipt::All_Opening_Status_Receipt(unsigned char send_times)
{
//   字节索引    	0        	1-2    	3       	4-5         	6          	7     	8-15           	16-23      	23-38	39  	40-45       
//   数据域     	frameHead	frameId	dataLen 	DeviceTypeId	isBroadcast	ZoneId	current_opening	MotorStatus	AI   	CRC8	frameEnd    
//   长度（byte）	1        	2      	1       	2           	1          	1     	8              	8          	16   	1   	6           
//   示例数据    	FE       	E00D   	0x24（36）	C003        	00         	01    	000000         	00000      	     	00  	0D0A0D0A0D0A

	iwdg_feed();
	Info_Println("卷膜开度状态全回执");
	Debug_Serial.flush();
	unsigned char ReceiptFrame[50] = { 0x00 };
	unsigned char ReceiptLength = 0;

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x0E; //
	ReceiptFrame[ReceiptLength++] = 0x24; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/*是否是群发*/
	gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;

	/*区域号*/
	ReceiptFrame[ReceiptLength++] = SN.Read_Area_Number();

	/* 开度*/
	for (unsigned char i = 0; i < 8; i++)
	{
		/* 开度查询函数，提供实时开度值 */
		// film_u8 Film_Read_RT_Opening_CH(film_u8 ch);
		if (i < MOTOR_CHANNEL)
			ReceiptFrame[ReceiptLength++] = Film_Read_RT_Opening_CH(i+1);
		else
			ReceiptFrame[ReceiptLength++] = 0xFE;//表示不存在的路数
	}
	
	/* 状态 */
	for (unsigned char i = 0; i < 8; i++)
	{
		/* 状态查询函数，提供实时状态 */
		// film_m_sta Film_Read_Motor_Status_CH(film_u8 ch);
		if (i < MOTOR_CHANNEL)
			ReceiptFrame[ReceiptLength++] = Film_Read_Motor_Status_CH(i+1);
		else
			ReceiptFrame[ReceiptLength++] = 0xFE;//表示不存在的路数
	}

	/* AI */
	unsigned short AI_data = 0;
	for (unsigned char i = 0; i < 8; i++)
	{
		AI_data = Modbus_Coil.Get_which_AI(Pos_Nega_mode.Read_AI_Relation_Way(i));
		// Serial.println(String("AI_data = ") + AI_data);
		ReceiptFrame[ReceiptLength++] = (AI_data >> 8) & 0xFF;
		ReceiptFrame[ReceiptLength++] = AI_data & 0xFF;
	}
	
	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);

	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < send_times; i++)
	{
		iwdg_feed();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
	}
	Info_Println("");
}


/*
 @brief   : 发送灌溉循环状态回执信息给服务器。（本设备 ---> 服务器）
 @param   :	1.是否需要随机时间延时发送
			2.回执次数，send_times
			3.随机ID第1位，randomId_1
			4.随机ID第2位，randomId_2
 @return  : 无
 */
// void Receipt::Irrigation_loop_Receipt(bool use_random_wait, unsigned char times, unsigned char randomId_1, unsigned char randomId_2)
// {
// 	  /*| 字节索引		| 0			| 1 - 2		| 3			| 4 - 5			| 6 - 7		| 8 - 39			| 40	| 41 - 46		|
// 		| 数据域		| frameHead | frameId	| dataLen	| DeviceTypeId	| randomId	| Surplus retryCnt	| CRC8	| frameEnd		|
// 		| 长度（byte）	| 1			| 2			| 1			| 2				| 2			| 32				| 1		| 6				|
// 		| 示例数据		| FE		| E001		| 0x24(36)	| C003			| 1234		| 00640000000		| D6	| 0D0A0D0A0D0A	|*/
// 	Serial.println("上报灌溉循环状态 <Working_Parameter_Receipt>");
// 	unsigned char ReceiptFrame[47] = { 0 };
// 	unsigned char ReceiptLength = 0;
// 	// unsigned char RandomSeed;
// 	unsigned long int RandomSendInterval = 0;

// 	iwdg_feed();
// 	if (use_random_wait)
// 	{
// 		//随机等待一段时间后再发送，避免大量设备发送堵塞。
// 		Receipt_Random_Wait_Value(&RandomSendInterval);
// 		delayMicroseconds(RandomSendInterval);
// 	}
// 	iwdg_feed();

// #if CLEAR_BUFFER_FLAG
// 	Clear_Server_LoRa_Buffer();
// #endif

// 	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
// 	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
// 	ReceiptFrame[ReceiptLength++] = 0x01; //
// 	ReceiptFrame[ReceiptLength++] = 0x24; //数据长度

// 	/*设备类型*/
// 	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
// 	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

// 	/*randomId*/
// 	ReceiptFrame[ReceiptLength++] = randomId_1;
// 	ReceiptFrame[ReceiptLength++] = randomId_2;

// 	/*Surplus retryCnt*/
// 	for (size_t i = 0; i < 16; i++)
// 	{
// 		ReceiptFrame[ReceiptLength++] = (retryCnt[i] >> 8) & 0XFF;
// 		ReceiptFrame[ReceiptLength++] = retryCnt[i] & 0XFF;
// 	}

// 	/*CRC8*/
// 	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
// 	/*帧尾*/
// 	for (unsigned char i = 0; i < 6; i++)
// 		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

// 	Serial.println("LoRa parameter receipt...");
// 	Print_Debug(&ReceiptFrame[0], ReceiptLength);

// 	for (unsigned char i = 0; i < times; i++)
// 	{
// 		iwdg_feed();
// 		//Some_Peripheral.Stop_LED();
// 		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
// 		delayMicroseconds(SEND_DATA_DELAY * 1000);
// 		//Some_Peripheral.Start_LED();
// 	}
// }

/*
 @brief   : 发送上线状态回执信息给服务器。（本设备 ---> 服务器）
 @param   :	1.是否需要随机时间延时发送
			2.回执次数，send_times
			3.随机ID第1位，randomId_1
			4.随机ID第2位，randomId_2
 @return  : 无
 */
void Receipt::OnLine_Receipt(bool use_random_wait, unsigned char times)
{
//   字节索引    	0        	1-2    	3      	4-5         	6-13    	14  	15-20        
//   数据域     	frameHead	frameId	dataLen	DeviceTypeId	Allocate	CRC8	frameEnd     
//   长度（byte）	1        	2      	1      	2           	8       	1   	6            
//   示例数据    	FE       	E002   	0A     	C003        	0000    	D6  	0D0A0D 0A0D0A

	Info_Println("上线状态报告 <Working_Parameter_Receipt>");
	unsigned char ReceiptFrame[23] = { 0 };
	unsigned char ReceiptLength = 0;
	// unsigned char RandomSeed;
	unsigned long int RandomSendInterval = 0;

	iwdg_feed();
	if (use_random_wait)
	{
		//随机等待一段时间后再发送，避免大量设备发送堵塞。
		Receipt_Random_Wait_Value(&RandomSendInterval);
		delayMicroseconds(RandomSendInterval);
	}
	iwdg_feed();

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
	ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
	ReceiptFrame[ReceiptLength++] = 0x02; //
	ReceiptFrame[ReceiptLength++] = 0x0A; //数据长度

	/*设备类型*/
	ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
	ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);

	/* allocate */
	for (size_t i = 0; i < 8; i++)
	{
		ReceiptFrame[ReceiptLength++] = 0x00;
	}
	

	/*CRC8*/
	ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], ReceiptFrame[3]);
	/*帧尾*/
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

	Info_Println("LoRa parameter receipt...");
	Print_Debug(&ReceiptFrame[0], ReceiptLength);

	for (unsigned char i = 0; i < times; i++)
	{
		iwdg_feed();
		//Some_Peripheral.Stop_LED();
		LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
		delayMicroseconds(SEND_DATA_DELAY * 1000);
		//Some_Peripheral.Start_LED();
	}	
}

/*
 @brief   : 串口打印16进制回执信息
 @param   : 1.数据起始地址
			2.数据长度s
 @return  : 无
 */
void Receipt::Print_Debug(unsigned char *base_addr, unsigned char len)
{
	for (unsigned char i = 0; i < len; i++)
	{
		Debug_Serial.print(base_addr[i], HEX);
		Debug_Serial.print(" ");
	}
	Info_Println();
}

/* 卷膜流程中的状态信息投递任务，开发者根据传入的状态来提供具体事项，如将状态上报给服务器 */
void Film_Status_Msg_Delivery_Task(film_u8 ch, film_m_sta sta)
{
	Debug_Serial.println("卷膜流程中的状态信息投递任务>>>");
	Debug_Serial.print(String("第") + ch + "路的状态为 ");
	switch (sta)
	{
		case Film_M_IDIE: Debug_Serial.println("Film_M_IDIE,电机空闲"); break;
		case Film_M_Opening: Debug_Serial.println("Film_M_Opening,正在卷膜");break;
		case Film_M_F_Opening: Debug_Serial.println("Film_M_F_Opening,正在强制卷膜");break;
		case Film_M_Reseting: Debug_Serial.println("Film_M_Reseting,正在重置行程");break;
		case Film_M_Open_OK: Debug_Serial.println("Film_M_Open_OK,卷膜完成");break;
		case Film_M_F_Open_OK: Debug_Serial.println("Film_M_F_Open_OK,强制卷膜完成");break;
		case FILM_M_First_Reset_OK: Debug_Serial.println("FILM_M_First_Reset_OK,到达重置行程起始位置");break;
		case Film_M_Reset_OK: Debug_Serial.println("Film_M_Reset_OK,重置行程完成");break;
		case Film_M_F_Stop_OK: Debug_Serial.println("Film_M_F_Stop_OK,强制停止成功");break;
		case Film_M_Wait_Chk: Debug_Serial.println("Film_M_Wait_Chk,到达限位，等待确认");break;
		case Film_M_OverEleCur: Debug_Serial.println("Film_M_OverEleCur,电机过流");break;
		case Film_M_MEM_Exp: Debug_Serial.println("Film_M_MEM_Exp,电机储存信息异常");break;
		case Film_M_Up_Limit_Exp: Debug_Serial.println("Film_M_Up_Limit_Exp,上限位异常");break;
		case Film_M_Down_Limit_Exp: Debug_Serial.println("Film_M_Down_Limit_Exp,下限位异常");break;
		case Film_M_Run_Exp: Debug_Serial.println("Film_M_Run_Exp,电机异常（检测到电压过低）");break;
		case Film_M_Unkonwn_Open: Debug_Serial.println("Film_M_Unkonwn_Open,电机开度未知");break;
		default: Debug_Serial.println("Unknow_Status,未知状态!!!"); break;
	}

	// Message_Receipt.Rolling_ch_Status_Receipt(1, ch, sta);
	// Message_Receipt.Opening_Control_Receipt(1,Auto_Report);
	Message_Receipt.All_Opening_Status_Receipt(1);
}

// /* 电机状态 */
// typedef enum{
//   Film_M_IDIE,  //电机空闲
//   Film_M_Opening, //正在卷膜
//   Film_M_F_Opening, //正在强制卷膜
//   Film_M_Reseting, //正在重置行程
//   Film_M_Open_OK, //卷膜完成
//   Film_M_F_Open_OK, //强制卷膜完成
//   FILM_M_First_Reset_OK,  //到达重置行程起始位置
//   Film_M_Reset_OK, //重置行程完成
//   Film_M_F_Stop_OK, //强制停止成功
//   Film_M_Wait_Chk, //到达限位，等待确认
//   Film_M_OverEleCur, //电机过流
//   Film_M_MEM_Exp, //电机储存信息异常
//   Film_M_Up_Limit_Exp,  //上限位异常
//   Film_M_Down_Limit_Exp, //下限位异常
//   Film_M_Run_Exp, //电机异常（检测到电压过低）
//   Film_M_Unkonwn_Open //未知开度（开度卷膜途中断电，且实时开度在全关或者全开上，为了防止服务器以为还正常关棚或着开棚，让服务器目前不确定是否在全开或者全关）
// }film_m_sta;

