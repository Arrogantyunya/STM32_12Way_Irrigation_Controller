// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_12Way_Irrigation_ControllerV6.0.0_beta.ino
    Created:	2020/6/8 16:54:47
    Author:     刘家辉
*/

/*
 * 主函数文件。
 * setup部分的功能有：复用调试引脚、添加线圈寄存器、开启独立看门狗、初始化定时器、设备串口波特率、初始化各类总线
 * 工程模式、注册服务器申请、校正RTC等。
 * loop部分的功能有：LoRa监听服务器指令、DO状态改变上报、定时自检参数、自动启动灌溉循环等。
 *
 * 如有任何疑问，请发送邮件到： liujiahiu@qq.com
 */

#include <arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/iwdg.h>
#include <RTClock.h>
#include <wirish_debug.h>

#include "Modbus.h"
#include "ModbusSerial.h"
#include "Set_coil.h"
#include "i2c.h"
#include "Command_Analysis.h"
#include "receipt.h"
#include "AT24CXX.h"
#include "Memory.h"
#include "LoRa.h"
#include "public.h"
#include "Security.h"
#include "fun_periph.h"
#include "Private_Timer.h"
#include "Private_RTC.h"
#include "User_Serial.h"
// #include "boot_config.h"
// #include "boot_mem.h"
// #include "boot_protocol.h"
#include "film_config.h"
#include "film_mem.h"
#include "film.h"

/* 测试宏 */
#define DEBUG_PRINT 	false 	//打印调试信息
#define Reset_RTC 		false   //重置rtc时间
#define Get_RTC			false	//获取rtc时间
#define Reset_Binding	false	//重新绑定
/* 使能宏 */
#define SOFT_HARD_VERSION	true 	//使能写入软件和硬件版本
#define USE_RTC				true 	//使用RTC
#define USE_TIMER			false 	//使用定时器
#define USE_COM				true	//使用串口发送数据
#define USE_KEY				false	//使用按键发送数据
#define USE_GATEWAY			true 	//loRa网关模式
#define DI_CHANGE_REPORT	true	//DI状态改变上报
#define DO_CHANGE_REPORT	true	//DO状态改变上报
#define DO_OPEN_REPORT		true	//DO某路开启就启动高频率状态上报
#define EEPROM_RESET		false 	//重置EEPROM的所有值【测试使用】
/* 替换宏 */
#define Software_version_high 	0x06 	//软件版本的高位
#define Software_version_low 	0x00 	//软件版本的低位
#define Hardware_version_high 	0x02 	//硬件版本的高位
#define Hardware_version_low 	0x00	//硬件版本的低位
#define Init_Area				0x01	//初始区域ID
#define Waiting_Collection_Time 2000	//等待采集的时间（ms）

unsigned char SoftVer[2] = {4,10};


//全局变量
String comdata = "";//串口接收的字符串
unsigned char gSN_Code[9] = { 0x00 }; //设备出厂默认SN码全为0
unsigned char gRTC_Code[7] = { 20,20,00,00,00,00,00 };//设备出厂默认RTC
bool DIStatus_Change = false;//DI的状态改变标志位
bool DOStatus_Change = false;//DO的状态改变标志位
bool One_Cycle_Complete = false;//一轮循环完成的标志位
bool Cyclic_intervalFlag = false;//循环时间间隔的标志位
bool Cyclic_timing = false;//正在进行循环时间间隔的标志位
bool DO_intervalFlag = false;//单个间隔时间的标志位
bool DO_interval_timing = false;//正在进行单个间隔时间的标志位
unsigned int KeyDI7_num = 0;//按键DI7按下的次数
unsigned char DI_NumLast = 0;unsigned char DI_NumNow = 0;//8路DI的上次值与当前值
unsigned int DO_NumLast = 0;unsigned int DO_NumNow = 0;//8路DO的上次值与当前值
bool RS485_DEBUG = false;
unsigned char RS485_Debug[20] = {0x00};unsigned char RS485_Debug_Length = 0;


//函数声明
void Request_Access_Network(void);		//检测是否已经注册到服务器成功
void Project_Debug(void);				//工程模式
void Key_Reset_LoRa_Parameter(void);	//按键重置LORA参数
void Key_cycle_irrigationV3(void);		//按键启动循环灌溉v3
void Regular_status_report(void);		//定时状态上报
void Change_status_report(void);		//状态改变上报
void Serial_Port_Configuration(void);	//串口设置

void Solenoid_mode_DO_ON(void);				//电磁阀DO开启
bool Solenoid_mode_DO_OFF(unsigned char i);	//电磁阀DO关闭

void Delay_mode_DO_ON(void);			//延时DO开启
bool Delay_mode_DO_OFF(unsigned char i);//延时DO定时关闭

void Forward_Reverse_DO_ON(void);				//正反转DO开启
bool Forward_Reverse_DO_OFF(unsigned char i);	//正反转DO关闭


// the setup function runs once when you press reset or power the board
void setup()
{
	Some_Peripheral.Peripheral_GPIO_Pinmode();//进行引脚的pinmode配置

	Modbus_Coil.Modbus_Config();//添加线圈寄存器

	/*Serial Wire debug only (JTAG-DP disabled, SW-DP enabled)*/
	// C:\Program Files (x86)\Arduino\hardware\stm32\STM32F1\system\libmaple\stm32f1\include\series
	afio_cfg_debug_ports(AFIO_DEBUG_NONE);//设置为JTAG和SW禁用

	/*配置IWDG*/
	iwdg_init(IWDG_PRE_256, 2000); //6.5ms * 1000 = 6500ms.

	Debug_Serial.begin(115200); //USART1, 当使用USART下载程序：USART--->USART1
	LoRa_MHL9LF.BaudRate(9600);//#define LoRa_Serial     Serial1
	// LoRa_Serial.setTimeout(55);
	RS485_Serial.begin(9600);//485的串口

	bkp_init();	//备份寄存器初始化使能
	EEPROM_Operation.EEPROM_GPIO_Config();		//设置EEPROM读写引脚
#if	EEPROM_RESET
	EEPROM_Operation.EEPROM_Reset();//重置EEPROM的所有值【测试使用】
#endif
	Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
	iwdg_feed();

#if USE_LORA_Control
	LoRa_MHL9LF.LoRa_GPIO_Config();
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
	/*
	 *上电后LoRa模块会发送厂家信息过来
	 *这个时候配置的第一个参数在校验回车换行等参数
	 *的时候会受到影响。必须先接收厂家信息并清空缓存
	 */
	delay(2000);
	gIsHandleMsgFlag = false;
	LoRa_Command_Analysis.Receive_LoRa_Cmd(); //从网关接收LoRa数据
	gIsHandleMsgFlag = true;
	iwdg_feed();

	//Initialize LoRa parameter.
	#if USE_GATEWAY
		LoRa_Para_Config.Save_LoRa_Com_Mode(0xF1);//这里是写入模式为网关模式
	#else
		LoRa_Para_Config.Save_LoRa_Com_Mode(0xF0);//这里是写入模式为节点模式
	#endif

	if (!LoRa_Para_Config.Verify_LoRa_TRMode_Flag())
	{
		LoRa_Para_Config.Save_LoRa_TRMode(0x01);//保存为默认状态，也就是模式1
	}
	Lora_TRMode = LoRa_Para_Config.Read_LoRa_TRMode();
	Debug_Serial.println(String("Lora_TRMode = ") + String(Lora_TRMode,HEX));

	if (!LoRa_Para_Config.Verify_LoRa_SYNC_Flag())
	{
		LoRa_Para_Config.Save_LoRa_SYNC(0x34);//同步字默认设置为0x34
	}
	LORA_SYNC = LoRa_Para_Config.Read_LoRa_SYNC();
	Debug_Serial.println(String("LORA_SYNC = ") + String(LORA_SYNC));
	
	// LoRa_MHL9LF.Parameter_Init(false);//LORA参数设置
	LoRa_Para_Config.Save_LoRa_Config_Flag();//保存LORA参数配置完成标志位
#endif

	// BT_Mount_Dev_And_Init(LoRa_Send_Data_For_BT);
	// BT_Save_S_Version(&SoftVer[0]);

#if SOFT_HARD_VERSION
	Debug_Serial.println("");

	//软件版本存储程序
	if (Software_version_high == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR) &&
		Software_version_low == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1))
	{
		Debug_Serial.println(String("Software_version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	else
	{
		Vertion.Save_Software_version(Software_version_high, Software_version_low);
		Debug_Serial.println(String("Successfully store the software version, the current software version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	//硬件版本存储程序
	if (Hardware_version_high == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR) &&
		Hardware_version_low == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1))
	{
		Debug_Serial.println(String("Hardware_version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
	else
	{
		Vertion.Save_hardware_version(Hardware_version_high, Hardware_version_low);
		Debug_Serial.println(String("Successfully store the hardware version, the current hardware version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
#endif

#if Reset_RTC
	Private_RTC.Update_RTC(&gRTC_Code[0]);
#endif // Reset_RTC

#if Get_RTC
	/*得到随机值*/
	unsigned char random_1 = random(0, 255);
	unsigned char random_2 = random(0, 255);

	/*这里上报完成一个轮次循环*/
	Message_Receipt.Irrigation_loop_Receipt(false, 1, random_1, random_2);
#endif // Get_RTC

	Project_Debug(); //工程模式

	// SN.Clear_SN_Access_Network_Flag(); //清除注册到服务器标志位

	/*Request access network(request gateway to save the device's SN code and channel)*/
	Request_Access_Network(); //检查是否注册到服务器

	while (SN.Self_check(gSN_Code) == false)
	{
		// LED_SELF_CHECK_ERROR;
		Debug_Serial.println("");
		Debug_Serial.println("Verify SN code failed, try to Retrieving SN code...");
		//Serial.println("验证SN代码失败，尝试找回SN代码…");
		Message_Receipt.Request_Device_SN_and_Channel(); //当本地SN码丢失，向服务器申请本机的SN码
		LoRa_Command_Analysis.Receive_LoRa_Cmd();		 //接收LORA参数
		MyDelayMs(3000);
		iwdg_feed();
	}
	Debug_Serial.println("SN self_check success...");//SN自检成功
	Debug_Serial.print("My SN is:");
	for (size_t i = 0; i < 9; i++)
	{
		Debug_Serial.print(gSN_Code[i],HEX);
		Debug_Serial.print(" ");
	}
	Debug_Serial.println();

	iwdg_feed();

	Realtime_Status_Reporting_Timer_Init(); //使用定时器2初始化自动上报状态周期(改为了定时器3)
	Start_Status_Report_Timing();
	// Stop_Status_Report_Timing();
	Debug_Serial.println("Timed status reporting mechanism initialization completed...");//定时上报机制初始化完成
	Debug_Serial.println("");

	Timer3_Init();//初始化定时器
	Start_Timer3(); //使用定时器3初始化自检参数功能自检周期
	Debug_Serial.println("Timer_3 initialization completed...");//定时自检机制初始化完成
	Debug_Serial.println("");

	Irrigation_Timer_Init();//使用定时器4初始化灌溉计时
	Stop_Timer4();//不用的时候关闭
	Debug_Serial.println("Irrigation timing mechanism initialization completed...");//灌溉计时机制初始化完成
	Debug_Serial.println("");

	Message_Receipt.OnLine_Receipt(true, 2);//设备上线状态报告
	Debug_Serial.println("Online status report");
	Debug_Serial.println("");

	//得到当前DI、DO的状态
	DI_NumLast = Modbus_Coil.Get_DI_1to8();	DI_NumNow = DI_NumLast;
	DO_NumLast = Modbus_Coil.Get_DO_1to8();	DO_NumNow = DO_NumLast;

	void Film_Param_Init(void);//卷膜初始化
	Debug_Serial.println("");

	Debug_Serial.println("All configuration items are initialized. Welcome to use!!!  ~(*^__^*)~ ");//所有的设置项初始化完成，欢迎使用
	Debug_Serial.println("");

	InitState.Save_WorkInterval(0x00,0x05);
	InitState.Save_StopInterval(0x00,0x0A);
}

/*
 @brief     : loop
 @param     : 无
 @return    : 无
 */
void loop()
{
	unsigned char *_empty = NULL;

	iwdg_feed();

	LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据

	Modbus_Coil.Modbus_Realization(_empty, 0);//设置输出线圈状态，modbus实现

	if (Device_Mode == Solenoid_mode)//电磁阀模式
	{
		Solenoid_mode_DO_ON();//灌溉分时打开
	}
	else if(Device_Mode == Delay_mode)//延时开启模式（）
	{
		Delay_mode_DO_ON();//延时DO开启
	}
	else if (Device_Mode == Forward_Reverse_mode)//正反转模式
	{
		Forward_Reverse_DO_ON();//正反转DO开启
	}
	else
	{
		//空模式Null_Mode
	}
	
	Check_Store_Param_And_LoRa(); //检查存储参数以及LORA参数

	// Regular_status_report();//定时状态上报

	Change_status_report();//状态改变上报

	Key_cycle_irrigationV3();//按键启动循环灌溉

	Serial_Port_Configuration();//串口设置

	Project_Debug();

	Film_Switch_Task();

	// BT_Cycle_Query_SW_Version();//
}

/*
 @brief   : 检测是否已经注册到服务器成功，如果没有注册，则配置相关参数为默认参数，然后注册到服务器。
			没有注册成功，红灯1每隔500ms闪烁。
			Checks whether registration with the server was successful, and if not,
			configures the relevant parameters as default parameters and registers with the server.
			Failing registration, red light flashes every 500ms.
 @para    : None
 @return  : None
 */
void Request_Access_Network(void)
{
	if (SN.Verify_SN_Access_Network_Flag() == false)
	{
		gAccessNetworkFlag = false;

		if (SN.Save_SN_Code(gSN_Code) && SN.Save_BKP_SN_Code(gSN_Code))
			Debug_Serial.println("Write Inital SN success... <Request_Access_Network>");

		if (SN.Clear_Area_Number() && SN.Clear_Group_Number())
		{
			Debug_Serial.println("Already Clear area number and group number... <Request_Access_Network>");
		}

		unsigned char Default_WorkGroup[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
		if (SN.Save_Group_Number(Default_WorkGroup))
			Debug_Serial.println("Save Inital group number success... <Request_Access_Network>");
		if (SN.Save_Area_Number(Init_Area))
			Debug_Serial.println("Save Inital area number success... <Request_Access_Network>");

		Debug_Serial.println("");
		Debug_Serial.println("Not registered to server, please send \"S\"	<Request_Access_Network>");
		// LED_NO_REGISTER;
	}
	while (SN.Verify_SN_Access_Network_Flag() == false)
	{
		iwdg_feed();
		while (Debug_Serial.available() > 0)
		{
			comdata += char(Debug_Serial.read());  //每次读一个char字符，并相加
			delay(2);
		}

		if (comdata.length() > 0)
		{
			comdata.toUpperCase();
			Debug_Serial.println(comdata);
			if (comdata == String("S"))
			{
				comdata = "";
				Debug_Serial.println("Start sending registration data to server <Request_Access_Network>");
				Message_Receipt.Report_General_Parameter();
			}

			iwdg_feed();
		}
		comdata = "";
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
	}
	gAccessNetworkFlag = true;
}

/*
 @brief   : 工程模式。用于单机测试485通信
			通过按键1，可以测试重置行程。
 @para    : 无
 @return  : 无
 */
void Project_Debug(void)
{
	iwdg_feed();
	if (RS485_DEBUG)
	{
		while (RS485_Serial.available() > 0)
		{
			RS485_Debug[RS485_Debug_Length++] = RS485_Serial.read();
			delay(2);
			if (RS485_Debug_Length >= 20)
			{
				iwdg_feed();
				RS485_Debug_Length = 0;
				Debug_Serial.println("数据超出可以接收的范围 <Project_Debug>");
				// delay(1000);
				memset(RS485_Debug, 0x00, sizeof(RS485_Debug));
			}
			Debug_Serial.print(RS485_Debug[RS485_Debug_Length - 1], HEX);
			Debug_Serial.print("_");
		}	
	}
}

/*
 @brief   : 按键重置LORA参数
 @para    : 无
 @return  : 无
 */
void Key_Reset_LoRa_Parameter(void)
{
	
}

/*
 @brief   : 定时状态上报
 @para    : 无
 @return  : 无
 */
void Regular_status_report(void)
{
	if (gStateReportFlag)
	{
		gStateReportFlag = false;
		Stop_Status_Report_Timing();

		unsigned long Now = millis();
		if (Waiting_Collection)
		{
			while (millis() - Now < 2000)
			{
				LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据
				// if (Device_Mode == Solenoid_mode)//电磁阀模式
				// {
				// 	Solenoid_mode_DO_ON();//灌溉分时打开
				// }
				// else if(Device_Mode == Delay_mode)//延时开启模式（）
				// {
				// 	Delay_mode_DO_ON();//延时DO开启
				// }
				// else if (Device_Mode == Forward_Reverse_mode)//正反转模式
				// {
				// 	Forward_Reverse_DO_ON();//正反转DO开启
				// }
				// else
				// {
					
				// }
				// Serial.println("Waiting_Collection......");
				// delay(500);
			}
			Waiting_Collection = false;
		}

		/*得到随机值*/
		// unsigned char random_1 = random(0, 255);
		// unsigned char random_2 = random(0, 255);
		// Serial.println(String("random_1 = ") + String(random_1, HEX));
		// Serial.println(String("random_2 = ") + String(random_2, HEX));

		/*这里上报实时状态*/
		// Message_Receipt.Working_Parameter_Receipt(true, 1, random_1, random_2);
		Message_Receipt.New_Working_Parameter_Receipt(true,1);

		Start_Status_Report_Timing();//开始上报状态周期计时
	}
}

/*
 @brief   : 状态改变上报
 @para    : 无
 @return  : 无
 */
void Change_status_report(void)
{
	DI_NumNow = Modbus_Coil.Get_DI_1to8();//得到当前DI的状态
	DO_NumNow = Modbus_Coil.Get_DO_1to8() + Modbus_Coil.Get_DO_9to16();//得到当前DO的状态

#if DI_CHANGE_REPORT
	if (DI_NumLast != DI_NumNow)//8路DI的值有所改变
	{
		DI_NumLast = DI_NumNow;
		DIStatus_Change = true;
		Debug_Serial.println("DI 状态已改变");
	}
#endif

#if DO_CHANGE_REPORT
	if (DO_NumLast != DO_NumNow)//8路DO的值有所改变
	{
		DO_NumLast = DO_NumNow;
		DOStatus_Change = true;
		Debug_Serial.println("DO 状态已改变");
	}
#endif

#if DO_OPEN_REPORT
	if (DO_NumNow != 0)//如果DO有任意一路开启，那么开启高频率自动上报，否则关闭
	{
		Enter_Work_State = true;
	}
	else
	{
		Enter_Work_State = false;
	}
#endif
	
	if (DOStatus_Change || DIStatus_Change || One_Cycle_Complete)
	{
		unsigned long Now = millis();
		if(Waiting_Collection)
		{
			while (millis() - Now < 2000)
			{
				LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据
				// if (Device_Mode == Solenoid_mode)//电磁阀模式
				// {
				// 	Solenoid_mode_DO_ON();//灌溉分时打开
				// }
				// else if(Device_Mode == Delay_mode)//延时开启模式（）
				// {
				// 	Delay_mode_DO_ON();//延时DO开启
				// }
				// else if (Device_Mode == Forward_Reverse_mode)//正反转模式
				// {
				// 	Forward_Reverse_DO_ON();//正反转DO开启
				// }
				// else
				// {
					
				// }
				// Serial.println("Waiting_Collection......");
				// delay(500);
			}
			Waiting_Collection = false;
		}

		Debug_Serial.println("开始状态改变上报 Start reporting of status change <Change_status_report>");
		DIStatus_Change = false;
		DOStatus_Change = false;
		One_Cycle_Complete = false;

		/*这里上报实时状态*/
		// Message_Receipt.Working_Parameter_Receipt(false, 1, random_1, random_2);
		Message_Receipt.New_Working_Parameter_Receipt(true,1);

		// Serial.println("Scheduled status reporting completed... <Change_status_report>");
	}

	// if (One_Cycle_Complete)
	// {
	// 	Serial.println("开始循环完成上报 Start cycle to complete reporting <Change_status_report>");
	// 	One_Cycle_Complete = false;

	// 	/*这里上报完成一个轮次循环*/
	// 	// Message_Receipt.Irrigation_loop_Receipt(false, 1, random_1, random_2);
	// 	// Message_Receipt.Working_Parameter_Receipt(false, 1, random_1, random_2);
	// 	Message_Receipt.New_Working_Parameter_Receipt(true,1);

	// 	Serial.println("Cycle & status report completed... <Change_status_report>");
	// }
}


/*
 @brief   : 串口设置
 @para    : 无
 @return  : 无
 */
void Serial_Port_Configuration(void)
{
	while (Debug_Serial.available() > 0)
	{
		comdata += char(Debug_Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Debug_Serial.println(comdata);

		if (comdata.startsWith("SET:"))
		{
			comdata.remove(0, 4);
			//Serial.println(comdata);
			Cyclic_interval = comdata.toInt();
			//Serial.println(String("Cyclic_interval = ") + Cyclic_interval);
			unsigned char Cyclic_interval_1 = (Cyclic_interval >> 8) & 0XFF;
			unsigned char Cyclic_interval_2 = Cyclic_interval & 0XFF;
			if (InitState.Save_CyclicInterval(Cyclic_interval_1, Cyclic_interval_2))
			{
				Debug_Serial.println("设置并保存参数成功 <Serial_Port_Configuration>");
			}
			else
			{
				Debug_Serial.println("设置并保存参数失败!!! <Serial_Port_Configuration>");
			}
			Cyclic_interval = InitState.Read_CyclicInterval();
			Debug_Serial.println(String("Cyclic_interval = ") + Cyclic_interval);
		}
		else if (comdata == String("485DEBUG"))
		{
			RS485_DEBUG = !RS485_DEBUG;
			if (RS485_DEBUG)
			{
				Debug_Serial.println("485 DeBug模式已启动...");
			}
			else
			{
				Debug_Serial.println("485 DeBug模式已关闭...");
			}	
		}
		else if (comdata == String("GET_CSQ"))
		{
			//在这里通过AT指令查询信号质量

			Debug_Serial.println("开始查询信号质量...");
			LoRa_MHL9LF.LoRa_AT(gLoRaCSQ, true, AT_CSQ_, 0);
		}
		else if (comdata.startsWith("SET_LORA_TRMODE="))//SET_LORA_TRMODE=1 
		{
			Debug_Serial.println("设置LoRa的TRMode");
			String Str_x = comdata.substring(comdata.indexOf("=")+1,comdata.length());//可以配合indexOf使用
			unsigned char x = (unsigned char) Str_x.toInt();//使用long来接收,所以强转为unsigned char
			LoRa_Para_Config.Save_LoRa_TRMode(x);
			Lora_TRMode = LoRa_Para_Config.Read_LoRa_TRMode();
			Debug_Serial.println(String("Lora_TRMode = ") + String(Lora_TRMode,HEX));
		}
		else if (comdata.startsWith("SET_LORA_SYNC="))//SET_LORA_SYNC=12
		{
			Debug_Serial.println("设置LoRa的SYNC");
			String Str_x = comdata.substring(comdata.indexOf("=")+1,comdata.length());//可以配合indexOf使用
			unsigned char x = (unsigned char) Str_x.toInt();//使用long来接收,所以强转为unsigned char
			LoRa_Para_Config.Save_LoRa_SYNC(x);
			LORA_SYNC = LoRa_Para_Config.Read_LoRa_SYNC();
			Debug_Serial.println(String("LORA_SYNC = ") + String(LORA_SYNC,HEX));
		}
		else if(comdata == String("LORA_INIT"))
		{
			LoRa_MHL9LF.Parameter_Init(false);//LORA参数设置
		}
		else
		{
			Debug_Serial.println("输入错误 <Serial_Port_Configuration>");
		}
		comdata = "";
	}
}


/*
 @brief   : 灌溉分时开启V3
 @para    : 无
 @return  : 无
 */
void Solenoid_mode_DO_ON()
{
	iwdg_feed();
	if (Complete_Num!=12)//等待本轮循环结束
	{
		for (unsigned char i = 0; i < 12; i++)
		{
			iwdg_feed();
			if (!DO_Set[i])//该路DO未被设置
			{
				//Serial.println(String("第") + i + "路未被设置");
				DO_Set[i] = true;//DO状态已被设置

				if (Worktime[i] > 0 && retryCnt[i] > 0)
				{
					Set_DO_relay(i, ON);
					DO_WayOpen[i] = true;//该路DO已打开
					// DOStatus_Change = true;//状态改变标志位已改变
					DO_WayOpentime[i] = Time_Count;//记录该路DO的打开时间
					DO_WayClosetime[i] = DO_WayOpentime[i] + Worktime[i];//计算出该路DO的关闭时间
					Debug_Serial.println(String("第") + i + "路已设置并开启 <Solenoid_mode_DO_ON>");
					Debug_Serial.println(String("DO_WayClosetime[") + i + "] = "+ DO_WayClosetime[i]);
					Debug_Serial.flush();
					
					if (WorkInterval[i] != 0)//该路有间隔时间
					{
						return;
					}
				}
				else
				{
					//表示不用开启该路
					Debug_Serial.println(String("第") + i + "路无需开启 <Solenoid_mode_DO_ON>");
					DO_WayComplete[i] = true;//该路已完成
					Complete_Num++;//完成了一路，不用开启代表直接完成
					Debug_Serial.println(String("完成个数 = ") + Complete_Num);
					Debug_Serial.flush();
					if (Complete_Num == 12)
					{
						Cyclic_intervalFlag = true;//需要循环计时
						Debug_Serial.println("已完成本轮循环 Cycle completed <Solenoid_mode_DO_ON>");
						One_Cycle_Complete = true;//一轮循环已完成
					}
				}
			}
			else
			{
				switch (i)
				{
				case 0:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 1:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 2:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 3:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 4:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 5:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 6:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 7:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 8:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 9:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 10:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				case 11:if (Solenoid_mode_DO_OFF(i)) {return;}	break;
				default:Debug_Serial.println("default");		break;
				}
			}
		}
	}
	else//完成了一轮循环
	{
		Stop_Timer4();

		if (Cyclic_intervalFlag)
		{
			Cyclic_intervalFlag = false;
			gRTCTime_arrive_Flag = false;
			Private_RTC.Set_Alarm();//设置RTC闹钟
			Debug_Serial.println("开始循环间隔计时 Start cycle interval");
			Enter_Work_State = false;
			Cyclic_timing = true;
		}

		if (Cyclic_timing)//正在进行循环时间间隔的标志位
		{
			if (gRTCTime_arrive_Flag)
			{
				Enter_Work_State = true;
				Debug_Serial.println("循环时间间隔结束 End of cycle interval");
				Debug_Serial.println(">>>>>>>>>>>>");
				Cyclic_timing = false;
				gRTCTime_arrive_Flag = false;
				for (size_t i = 0; i < 12; i++)
				{
					DO_Set[i] = false;//该路DO未设置
					DO_WayComplete[i] = false;//该路DO未完成
					DO_WayOpen[i] = false;//该路DO已关闭
					if (retryCnt[i] > 0)//代表还有循环
					{
						Complete_Num = 0;
					}
				}

				if (Complete_Num == 0)
				{
					Debug_Serial.println("下一轮循环开始 Start of next cycle");
					Debug_Serial.println(">>>>>>>>>>>>");
					Start_Timer4();
				}
				else
				{
					Enter_Work_State = false;
					Debug_Serial.println("完成所有的灌溉循环 Complete all irrigation cycles");
					Debug_Serial.println("(｡◕ˇ∀ˇ◕）");
				}
			}
		}
	}
}

/* 电磁阀DO关闭 */
bool Solenoid_mode_DO_OFF(unsigned char i)
{
	if (!DO_WayComplete[i])//该路没完成
	{
		if (DO_WayOpen[i])//该路DO已打开
		{
			if (Time_Count >= DO_WayClosetime[i])
			{
				Set_DO_relay(i, OFF);
				DO_WayOpen[i] = false;//该路DO已关闭
				// DOStatus_Change = true;//状态改变标志位已改变
				Debug_Serial.println(String("第") + i + "路时间已到达并关闭");
				Debug_Serial.flush();
				if (WorkInterval[i] != 0)//间隔时间不为0
				{
					DO_WayInterval[i] = true;//需要进行间隔
					//DO_WayComplete[i] = true;//该路已完成
					DO_WayIntervalBengin[i] = Time_Count;//记录DO的间隔开始时间
					DO_WayIntervalEnd[i] = DO_WayIntervalBengin[i] + WorkInterval[i];//记录DO的间隔结束时间
					Debug_Serial.println(String("DO_WayIntervalEnd[") + i + "] = " + DO_WayIntervalEnd[i]);
					Debug_Serial.flush();
				}
				else//间隔时间为0
				{
					retryCnt[i]--;//循环的次数-1
					Complete_Num++;//完成了一路
					Debug_Serial.println(String("完成个数 = ") + Complete_Num);
					Debug_Serial.flush();
					if (Complete_Num == 12)
					{
						Debug_Serial.println("已完成本轮循环 Cycle completed");
						Cyclic_intervalFlag = true;//需要循环计时
						One_Cycle_Complete = true;//一轮循环已完成
					}
					DO_WayInterval[i] = false;//不需要进行间隔
					DO_WayComplete[i] = true;//16路DO完成的标志位
					return false;
				}
			}
		}
		if (DO_WayInterval[i])//该路DO需要间隔
		{
			if (Time_Count >= DO_WayIntervalEnd[i])
			{
				DO_WayComplete[i] = true;//该路已完成
				retryCnt[i]--;//循环的次数-1
				Complete_Num++;//完成了一路
				Debug_Serial.println(String("完成个数 = ") + Complete_Num);
				Debug_Serial.flush();
				if (Complete_Num == 12)
				{
					Debug_Serial.println("已完成本轮循环 Cycle completed");
					Cyclic_intervalFlag = true;//需要循环计时
					One_Cycle_Complete = true;//一轮循环已完成
				}
				DO_WayComplete[i] = true;//该路已完成
				Debug_Serial.println(String("第") + i + "路间隔时间到达");
				Debug_Serial.flush();
				return false;
			}
			return true;
		}
		if (WorkInterval[i] != 0)
		{
			return true;
		}
	}
	return false;
}


/*
 @brief   : 按键启动循环灌溉v3
 @para    : 无
 @return  : 无
 */
void Key_cycle_irrigationV3(void)
{
#if USE_COM
	while (Debug_Serial.available() > 0)
	{
		comdata += char(Debug_Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Debug_Serial.println(comdata);
		if (comdata == String("B"))
		{
			comdata = "";
			KeyDI7_num++;
			Debug_Serial.println("Start circulating irrigation button press <Key_cycle_irrigationV2>");
			Debug_Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
			Debug_Serial.println("");
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


			for (size_t i = 0; i < 12; i++)
			{
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
			}
			if (KeyDI7_num % 2)//代表需要启动
			{
				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）

				for (size_t i = 0; i < 12; i++)
				{
					Worktime[i] = 20;//开启时间为20s
					WorkInterval[i] = 60;//间隔时间为60s
					retryCnt[i] = 0xFFFF;//65535次循环
				}
				Start_Timer4();
			}
			else//代表关闭
			{
				Cyclic_interval = 0;//循环间隔为0小时
				for (size_t i = 0; i < 12; i++)
				{
					Worktime[i] = 0;//开启时间为0s
					WorkInterval[i] = 0;//间隔时间为0s
					retryCnt[i] = 0x00;//0次循环
				}
			}
		}
		else
		{
			//comdata = "";
		}
		iwdg_feed();
	}
#elif USE_KEY
	if (digitalRead(DI7) == HIGH)
	{
		iwdg_feed();
		delay(100);
		if (digitalRead(DI7) == HIGH)
		{
			KeyDI7_num++;
			Serial.println("Start circulating irrigation button press <Key_cycle_irrigationV2>");
			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
			Serial.println("");
			// Stop_Timer4();//先停止计时
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

			for (size_t i = 0; i < 12; i++)
			{
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
				Set_Irrigation_relay(i, OFF);
			}

			/*等待按键释放*/
			while (digitalRead(DI7) == HIGH)
			{

			}

			if (KeyDI7_num % 2)//代表需要启动
			{
				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）

				for (size_t i = 0; i < 12; i++)
				{
					Worktime[i] = 20;//开启时间为20s
					WorkInterval[i] = 60;//间隔时间为60s
					retryCnt[i] = 0xFFFF;//65535次循环
				}
				Start_Timer4();
			}
			else//代表关闭
			{
				Cyclic_interval = 0;//循环间隔为0小时
				for (size_t i = 0; i < 12; i++)
				{
					Worktime[i] = 0;//开启时间为0s
					WorkInterval[i] = 0;//间隔时间为0s
					retryCnt[i] = 0x00;//0次循环
				}
			}
		}
	}
#else

#endif
}

/*
 @brief   : 延时DO开启
 @para    : 无
 @return  : 无
 */
void Delay_mode_DO_ON(void)
{
	if (Complete_Num!=12)//等待本轮循环结束
	{
		for (size_t i = 0; i < 12; i++)
		{
			if (Delay_mode_NeedWait)//次数开始等待延时
			{
				if (Time_Count >= Delay_mode_EndWait)
				{
					Delay_mode_NeedWait  = false;
				}
			}
			if (!DO_Set[i] && !Delay_mode_NeedWait)
			{
				DO_Set[i] = true;//DO状态已被设置

				if (Delay_mode_Worktime[i] > 0)
				{
					Set_DO_relay(i, ON);
					DO_WayOpen[i] = true;//该路DO已打开
					DO_WayOpentime[i] = Time_Count;//记录该路DO的打开时间
					DO_WayClosetime[i] = DO_WayOpentime[i] + Delay_mode_Worktime[i];//计算出该路DO的关闭时间
					Debug_Serial.println(String("第") + i + "路已设置并开启<Delay_mode_DO_ON>");
					Debug_Serial.println(String("DO_WayClosetime[") + i + "] = "+ DO_WayClosetime[i]);
					Debug_Serial.flush();
					Delay_mode_OpenNum++;
					if (Delay_mode_OpenNum == Delay_mode_DONum)
					{
						//等待延时直到下一路开启
						Debug_Serial.println("等待延时直到下一路开启");
						Delay_mode_NeedWait = true;
						Delay_mode_OpenNum = 0;
						Delay_mode_EndWait = Time_Count + Delay_mode_Interval;
						return;
					}
				}
				else
				{
					//表示不用开启该路
					Debug_Serial.println(String("第") + i + "路无需开启<Delay_mode_DO_ON>");
					DO_WayComplete[i] = true;//该路已完成
					Complete_Num++;//完成了一路，不用开启代表直接完成
					Debug_Serial.println(String("完成个数 = ") + Complete_Num);
					Debug_Serial.flush();
					if (Complete_Num == 12)
					{
						Debug_Serial.println("已完成本轮循环 Cycle completed<Delay_mode_DO_ON>");
						One_Cycle_Complete = true;//一轮循环已完成
					}
				}
			}
			else
			{
				switch (i)
				{
					case 0:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 1:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 2:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 3:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 4:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 5:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 6:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 7:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 8:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 9:if (Delay_mode_DO_OFF(i)) {return;}	break;
					case 10:if (Delay_mode_DO_OFF(i)) {return;}break;
					case 11:if (Delay_mode_DO_OFF(i)) {return;}break;
					default:Debug_Serial.println("default");	break;
				}
			}
		}
	}
	else//完成了一轮循环
	{
		Stop_Timer4();
		Debug_Serial.println("完成延时循环 Complete Delay Cycles");
		Debug_Serial.println("(｡◕ˇ∀ˇ◕）");
		Enter_Work_State = false;
		Complete_Num = 0;

		Delay_mode_OpenSec = 0;//
		Delay_mode_Interval = 0;
		Delay_mode_DONum = 0;
		for (size_t i = 0; i < 16; i++)
		{
			Delay_mode_Worktime[i] = 0;
		}
		Delay_mode_NeedWait = false;
		Delay_mode_OpenNum = 0;
		Delay_mode_EndWait = 0;
	}
}


/*
 @brief   : 延时DO定时关闭
 @para    : 无
 @return  : 无
 */
bool Delay_mode_DO_OFF(unsigned char i)
{
	if (!DO_WayComplete[i])//该路没完成
	{
		if (DO_WayOpen[i])
		{
			if (Time_Count >= DO_WayClosetime[i])
			{
				Set_DO_relay(i, OFF);
				DO_WayOpen[i] = false;//该路DO已关闭
				Complete_Num++;//完成了一路
				Debug_Serial.println(String("第") + i + "路时间已到达并关闭");
				Debug_Serial.flush();
			}
		}
	}
	return false;
}

/*
 @brief   : 正反转DO开启
 @para    : 无
 @return  : 无
 */
void Forward_Reverse_DO_ON(void)
{
	if (Complete_Num!=12)//等待本轮循环结束
	{
		for (size_t i = 0; i < 12; i++)
		{
			if (Forward_Reverse_mode_NeedWait)//次数开始等待延时
			{
				if (Time_Count >= Forward_Reverse_mode_EndWait)
				{
					Forward_Reverse_mode_NeedWait  = false;
				}
			}
			if (!DO_Set[i] && !Forward_Reverse_mode_NeedWait)
			{
				DO_Set[i] = true;//DO状态已被设置

				if(Forward_Reverse_mode_Worktime[i] > 0)
				{
					Set_DO_relay(i, ON);
					DO_WayOpen[i] = true;//该路DO已打开
					DO_WayOpentime[i] = Time_Count;//记录该路DO的打开时间
					DO_WayClosetime[i] = DO_WayOpentime[i] + Forward_Reverse_mode_Worktime[i];//计算出该路DO的关闭时间
					Debug_Serial.println(String("第") + i + "路已设置并开启<Forward_Reverse_DO_ON>");
					Debug_Serial.println(String("DO_WayClosetime[") + i + "] = "+ DO_WayClosetime[i]);
					Debug_Serial.flush();

					//等待延时直到下一路开启
					if(Forward_Reverse_mode_Interval > 0)
					{
						Debug_Serial.println("等待延时直到下一路开启");
						Forward_Reverse_mode_NeedWait = true;
						Forward_Reverse_mode_EndWait = Time_Count + Forward_Reverse_mode_Interval;
						return;
					}
				}
				else
				{
					//表示不用开启该路
					Debug_Serial.println(String("第") + i + "路无需开启<Forward_Reverse_DO_ON>");
					DO_WayComplete[i] = true;//该路已完成
					Complete_Num++;//完成了一路，不用开启代表直接完成
					Debug_Serial.println(String("完成个数 = ") + Complete_Num);
					Debug_Serial.flush();
					if (Complete_Num == 12)
					{
						Debug_Serial.println("已完成本轮循环 Cycle completed<Forward_Reverse_DO_ON>");
						One_Cycle_Complete = true;//一轮循环已完成
					}
				}
			}
			else
			{
				switch (i)
				{
				case 0:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 1:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 2:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 3:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 4:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 5:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 6:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 7:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 8:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 9:if (Forward_Reverse_DO_OFF(i)) {return;}	break;
				case 10:if (Forward_Reverse_DO_OFF(i)) {return;}break;
				case 11:if (Forward_Reverse_DO_OFF(i)) {return;}break;
				default:Debug_Serial.println("default");	break;
				}
			}
		}
	}
	else//完成了一轮循环
	{
		Stop_Timer4();
		Debug_Serial.println("完成正反转循环 Complete Forward&Reverse Cycles");
		Debug_Serial.println("(｡◕ˇ∀ˇ◕）");
		Enter_Work_State = false;
		Complete_Num = 0;

		for (size_t i = 0; i < 12; i++)
		{
			Forward_Reverse_mode_Worktime[i] = 0;//
		}
		Forward_Reverse_mode_Interval = 0;
		for (size_t i = 0; i < 16; i++)
		{
			Delay_mode_Worktime[i] = 0;
		}
		Forward_Reverse_mode_NeedWait = false;
		Forward_Reverse_mode_EndWait = 0;
	}
}

/*
 @brief   : 正反转DO关闭
 @para    : 无
 @return  : 无
 */
bool Forward_Reverse_DO_OFF(unsigned char i)
{
	if (!DO_WayComplete[i])//该路没完成
	{
		if (DO_WayOpen[i])
		{
			if (Time_Count >= DO_WayClosetime[i])
			{
				Set_DO_relay(i, OFF);
				DO_WayOpen[i] = false;//该路DO已关闭
				Complete_Num++;//完成了一路
				Debug_Serial.println(String("第") + i + "路时间已到达并关闭");
				Debug_Serial.flush();
			}
		}
	}
	return false;
}


void BT_Stop_Interrupt(void)
{
  Timer2.detachCompare1Interrupt();
  Timer3.detachCompare1Interrupt();
  Timer4.detachCompare1Interrupt();
//   detachInterrupt(DEC_MANUAL_DOWN_PIN);
//   detachInterrupt(DEC_MANUAL_UP_PIN);
  	LoRa_Serial.end();
  	Debug_Serial.end();
	RS485_Serial.end();
}