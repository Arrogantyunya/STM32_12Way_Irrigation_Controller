// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_8Way_Irrigation_ControllerV1.0.0_alpha.ino
    Created:	2020/2/18 19:21:47
    Author:     刘家辉
*/

/*
 * 主函数文件。
 * setup部分的功能有：复用调试引脚、开启独立看门狗、设备串口波特率、初始化各类总线
 * 工程模式、注册服务器申请、校正因突发断电的电机开度卷膜等。
 * loop部分的功能有：矫正开度误差、LoRa监听服务器指令、手动卷膜监测、定时自检参数等。
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

/* 测试宏 */
#define DEBUG_PRINT 	0 	//打印调试信息
#define Reset_RTC 		1   //重置rtc时间
#define Reset_Binding	0	//重新绑定
/* 使能宏 */
#define SOFT_HARD_VERSION	1 //使能写入软件和硬件版本
#define USE_RTC				1 //使用RTC
#define USE_TIMER			0 //使用定时器
#define USE_COM				0 //使用串口发送数据
#define USE_KEY				1 //使用按键发送数据
/* 替换宏 */
#define Software_version_high 	0x01 	//软件版本的高位
#define Software_version_low 	0x00  	//软件版本的低位
#define Hardware_version_high 	0x01 	//硬件版本的高位
#define Hardware_version_low 	0x00	//硬件版本的低位
#define Init_Area				0x01	//初始区域ID


//全局变量
String comdata = "";//串口接收的字符串
unsigned char gSN_Code[9] = { 0x00 }; //设备出厂默认SN码全为0
unsigned char gRTC_Code[7] = { 20,20,00,00,00,00,00 };
bool DOStatus_Change = false;
bool Cyclic_intervalFlag = false;
bool Cyclic_timing = false;
unsigned int KeyDI7_num = 0;


//函数声明
void Request_Access_Network(void);		//检测是否已经注册到服务器成功
void Project_Debug(void);				//工程模式
void Key_Reset_LoRa_Parameter(void);	//按键重置LORA参数
void Irrigation_time_sharing_on(void);	//灌溉分时开启
void Key_cycle_irrigation(void);		//按键启动循环灌溉
void Regular_status_report(void);		//定时状态上报
void Change_status_report(void);		//状态改变上报
void Com_Set_Cyclic_interval(void);		//串口设置循环间隔



// the setup function runs once when you press reset or power the board
void setup()
{
	Some_Peripheral.Peripheral_GPIO_Pinmode();//进行引脚的pinmode配置

	Modbus_Coil.Modbus_Config();//添加线圈寄存器

	/*Serial Wire debug only (JTAG-DP disabled, SW-DP enabled)*/
	// C:\Program Files (x86)\Arduino\hardware\stm32\STM32F1\system\libmaple\stm32f1\include\series
	afio_cfg_debug_ports(AFIO_DEBUG_NONE);//设置为JTAG和SW禁用

	/*配置IWDG*/
	iwdg_init(IWDG_PRE_256, 1000); //6.5ms * 1000 = 6500ms.

	Serial.begin(9600); //USART1, 当使用USART下载程序：USART--->USART1
	LoRa_MHL9LF.BaudRate(9600);//#define LoRa_Serial     Serial1
	Serial2.begin(9600);//485的串口

	bkp_init();	//备份寄存器初始化使能
	EEPROM_Operation.EEPROM_GPIO_Config();		//设置EEPROM读写引脚
	Some_Peripheral.Peripheral_GPIO_Config();	//设置继电器，数字输入，模拟输入等外设引脚的模式，以及初始化状态
	iwdg_feed();

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
	LoRa_MHL9LF.Parameter_Init(false);
	LoRa_Para_Config.Save_LoRa_Config_Flag();

#if SOFT_HARD_VERSION
	Serial.println("");

	//软件版本存储程序
	if (Software_version_high == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR) &&
		Software_version_low == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("Software_version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	else
	{
		Vertion.Save_Software_version(Software_version_high, Software_version_low);
		Serial.println(String("Successfully store the software version, the current software version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	//硬件版本存储程序
	if (Hardware_version_high == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR) &&
		Hardware_version_low == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("Hardware_version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
	else
	{
		Vertion.Save_hardware_version(Hardware_version_high, Hardware_version_low);
		Serial.println(String("Successfully store the hardware version, the current hardware version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
#endif

#if Reset_RTC
	Private_RTC.Update_RTC(&gRTC_Code[0]);
#endif // Reset_RTC


	Project_Debug(); //工程模式

	//SN.Clear_SN_Access_Network_Flag(); //清除注册到服务器标志位

	/*Request access network(request gateway to save the device's SN code and channel)*/
	Request_Access_Network(); //检查是否注册到服务器

	while (SN.Self_check(gSN_Code) == false)
	{
		// LED_SELF_CHECK_ERROR;
		Serial.println("");
		Serial.println("Verify SN code failed, try to Retrieving SN code...");
		//Serial.println("验证SN代码失败，尝试找回SN代码…");
		Message_Receipt.Request_Device_SN_and_Channel(); //当本地SN码丢失，向服务器申请本机的SN码
		LoRa_Command_Analysis.Receive_LoRa_Cmd();		 //接收LORA参数
		MyDelayMs(3000);
		iwdg_feed();
	}
	Serial.println("SN self_check success...");//SN自检成功
	// LED_RUNNING;
	iwdg_feed();

	Realtime_Status_Reporting_Timer_Init(); //使用定时器2初始化自动上报状态周期
	Serial.println("Timed status reporting mechanism initialization completed...");//定时上报机制初始化完成
	Serial.println("");

	Self_Check_Parameter_Timer_Init(); //使用定时器3初始化自检参数功能自检周期
	Serial.println("Timed self check mechanism initialization completed...");//定时自检机制初始化完成
	Serial.println("");

	Irrigation_Timer_Init();//使用定时器4初始化灌溉计时
	Stop_Timer4();//不用的时候关闭
	Serial.println("Irrigation timing mechanism initialization completed...");//灌溉计时机制初始化完成
	Serial.println("");

	Serial.println("All configuration items are initialized. Welcome to use.  ~(*^__^*)~ ");//所有的设置项初始化完成，欢迎使用
	Serial.println("");
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

	//Modbus_Coil.Modbus_Realization(_empty, 0);//设置输出线圈状态，modbus实现

	Check_Store_Param_And_LoRa(); //检查存储参数以及LORA参数

	Regular_status_report();//定时状态上报

	Change_status_report();//状态改变上报

	Irrigation_time_sharing_on();//灌溉分时打开

	Key_cycle_irrigation();//按键启动循环灌溉

	Com_Set_Cyclic_interval();//串口设置循环间隔

	//Serial.println("loop");
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
			Serial.println("Write Inital SN success... <Request_Access_Network>");

		if (SN.Clear_Area_Number() && SN.Clear_Group_Number())
		{
			Serial.println("Already Clear area number and group number... <Request_Access_Network>");
		}

		unsigned char Default_WorkGroup[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
		if (SN.Save_Group_Number(Default_WorkGroup))
			Serial.println("Save Inital group number success... <Request_Access_Network>");
		if (SN.Save_Area_Number(Init_Area))
			Serial.println("Save Inital area number success... <Request_Access_Network>");

		Serial.println("");
		Serial.println("Not registered to server, please send \"S\"	<Request_Access_Network>");
		// LED_NO_REGISTER;
	}
	while (SN.Verify_SN_Access_Network_Flag() == false)
	{
		iwdg_feed();
		while (Serial.available() > 0)
		{
			comdata += char(Serial.read());  //每次读一个char字符，并相加
			delay(2);
		}

		if (comdata.length() > 0)
		{
			comdata.toUpperCase();
			Serial.println(comdata);
			if (comdata == String("S"))
			{
				comdata = "";
				Serial.println("Start sending registration data to server <Request_Access_Network>");
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
 @brief   : 工程模式。用于在单机工作的情况下。手动卷膜，可以测量脉冲数、电机电流、上下限位等。
			通过按键1，可以测试重置行程。
 @para    : 无
 @return  : 无
 */
void Project_Debug(void)
{
	// while (1)
	// {
	// 	iwdg_feed();
	// 	for (size_t i = 0; i < 12; i++)
	// 	{

	// 	}

	// }

}

/*
 @brief   : 按键重置LORA参数
 @para    : 无
 @return  : 无
 */
void Key_Reset_LoRa_Parameter(void)
{
	// if (digitalRead(SW_FUN1) == LOW)
	// {
	// 	MyDelayMs(100);
	// 	if (digitalRead(SW_FUN1) == LOW)
	// 	{
	// 		MyDelayMs(3000);
	// 		iwdg_feed();
	// 		if (digitalRead(SW_FUN2) == LOW)
	// 		{
	// 			MyDelayMs(100);
	// 			if (digitalRead(SW_FUN2) == LOW)
	// 			{
	// 				LoRa_Para_Config.Clear_LoRa_Config_Flag();
	// 				Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
	// 				Serial.println("清除LoRa配置标志成功...<Key_Reset_LoRa_Parameter>");
	// 				iwdg_feed();
	// 			}
	// 		}
	// 	}
	// }
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

		/*得到随机值*/
		unsigned char random_1 = random(0, 255);
		unsigned char random_2 = random(0, 255);
		Serial.println(String("random_1 = ") + String(random_1, HEX));
		Serial.println(String("random_2 = ") + String(random_2, HEX));


		/*这里上报实时状态*/
		Message_Receipt.Working_Parameter_Receipt(true, 1, random_1, random_2);

		Start_Status_Report_Timing();
		Serial.println("Scheduled status reporting completed... <Regular_status_report>");
	}
}

/*
 @brief   : 状态改变上报
 @para    : 无
 @return  : 无
 */
void Change_status_report(void)
{
	if (DOStatus_Change)
	{
		DOStatus_Change = false;
		Get_receipt = true;

		/*得到随机值*/
		unsigned char random_1 = random(0, 255);
		unsigned char random_2 = random(0, 255);
		Serial.println(String("random_1 = ") + String(random_1, HEX));
		Serial.println(String("random_2 = ") + String(random_2, HEX));

		/*这里上报实时状态*/
		Message_Receipt.Working_Parameter_Receipt(true, 1, random_1, random_2);

		Serial.println("Scheduled status reporting completed... <Change_status_report>");
	}
}



/*
 @brief   : 灌溉分时开启
 @para    : 无
 @return  : 无
 */
void Irrigation_time_sharing_on()
{
	iwdg_feed();
	if (retryCnt > 0)
	{
		Set_Irrigation_relay(11, ON);//开启循环时，Y12亮

		if (Cyclic_intervalFlag)
		{
#if USE_RTC
			Cyclic_intervalFlag = false;
			gRTCTime_arrive_Flag = false;
			Private_RTC.Set_Alarm();//设置RTC闹钟
			Serial.println("开始循环间隔计时");
			Cyclic_timing = true;
#elif USE_TIMER
			Cyclic_intervalFlag = false;
			gTime_arrive_Flag = false;
			//开始循环间隔计时
			Irrigation_time = Cyclic_interval;
			Start_Timer4();
			Serial.println("开始循环间隔计时");
			Cyclic_timing = true;
#else
#endif // USE_RTC
		}
		if (Cyclic_timing)
		{
#if USE_RTC
			if (gRTCTime_arrive_Flag)
			{
				Serial.println("循环时间间隔结束");
				Cyclic_timing = false;
				gRTCTime_arrive_Flag = false;
			}
#elif USE_TIMER
			if (gTime_arrive_Flag)
			{
				Serial.println("循环时间间隔结束");
				Stop_Timer4();
				Cyclic_timing = false;
				gTime_arrive_Flag = false;
			}
#else
#endif // USE_RTC
		}
		else
		{
			for (size_t i = 0; i < 16; i++)
			{
				if (Worktime[i] > 0/* && !Irrigation_use*/)
				{
					if (i % 2)
					{
						if (!Irrigation_use)
						{
							Irrigation_use = true;
							gTime_arrive_Flag = false;
							Irrigation_time = Worktime[i];
							Start_Timer4();
							Set_Irrigation_relay(i / 2, ON);
							DOStatus_Change = true;//状态改变标志位已改变

							Serial.println(String("开始第") + ((i - 1) / 2) + "路计时");
						}

						if (gTime_arrive_Flag)
						{
							Serial.println(String("第") + ((i - 1) / 2) + "路时间到达");
							Serial.println("------");
							Complete_Num++;//完成的个数++
							Stop_Timer4();
							Set_Irrigation_relay(i / 2, OFF);
							DOStatus_Change = true;//状态改变标志位已改变

							gTime_arrive_Flag = false;
							Irrigation_use = false;
							Worktime[i] = 0;

							if (Complete_Num == Need_Num)
							{
								Serial.println("已完成1次灌溉循环");
								Serial.println();
								Complete_Num = 0;
								retryCnt--;
								for (size_t i = 0; i < 24; i++)
								{
									Worktime[i] = Worktime_backups[i];
								}

								

								if (retryCnt > 0)
								{
									Cyclic_intervalFlag = true;
									Serial.println("继续循环");
									Serial.println(String("剩余循环次数为 = ") + retryCnt);
								}
								else
								{
									Cyclic_intervalFlag = false;
									Serial.println("完成所有的灌溉循环");
									Serial.println("(｡◕ˇ∀ˇ◕）");
								}
							}
						}
						return;
					}
					else
					{
						if (!Irrigation_use)
						{
							Irrigation_use = true;
							gTime_arrive_Flag = false;
							Irrigation_time = Worktime[i];
							Start_Timer4();
							Serial.println(String("开始第") + (i / 2) + "路间隔等待");
						}

						if (gTime_arrive_Flag)
						{
							Serial.println(String("第") + (i / 2) + "路间隔等待时间到达");
							Stop_Timer4();
							gTime_arrive_Flag = false;
							Irrigation_use = false;
							Worktime[i] = 0;
						}
						return;
					}
				}
			}
		}
	}
	else
	{
		Set_Irrigation_relay(11, OFF);//开启循环时，Y12亮
	}
}

/*
 @brief   : 按键启动循环灌溉
 @para    : 无
 @return  : 无
 */
void Key_cycle_irrigation(void)
{
	if (KeyDI7_num % 2)//代表需要启动
	{
		Get_receipt = true;
	}
	else
	{
		//Get_receipt = false;
	}
#if USE_COM
	while (Serial.available() > 0)
	{
		comdata += char(Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Serial.println(comdata);
		if (comdata == String("B"))
		{
			comdata = "";
			KeyDI7_num++;
			Serial.println("Start circulating irrigation button press <Key_cycle_irrigation>");
			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
			Serial.println("");
			if (KeyDI7_num % 2)//代表需要启动
			{
				Stop_Timer4();//先停止计时
				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
				Cyclic_intervalFlag = false;//
				gRTCTime_arrive_Flag = false;//
				gTime_arrive_Flag = false; //
				Cyclic_timing = false;//
				Irrigation_use = false;//
				Need_Num = 0;	//需要开启继电器的个数
				Complete_Num = 0;//完成开启的个数

				for (size_t i = 0; i < 8; i++)
				{
					Worktime[2 * i] = 0;
					Worktime[(2 * i) + 1] = 0;
					Worktime_backups[2 * i] = 0;
					Worktime_backups[(2 * i) + 1] = 0;
					Set_Irrigation_relay(i, OFF);
				}

				OpenSec = 2;//开启时间为20s
				DO_Interval = 2;//间隔时间为2s
				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）
				DO_Num = 1;//每次开启1路
				retryCnt = 0x3;//65535次循环

				for (size_t i = 0; i < 8; i++)
				{
					Worktime[2 * i] = DO_Interval;
					Worktime[(2 * i) + 1] = OpenSec;
					Worktime_backups[2 * i] = DO_Interval;
					Worktime_backups[(2 * i) + 1] = OpenSec;
				}
				Need_Num = 8;
			}
			else//代表关闭
			{
				Stop_Timer4();//先停止计时
				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
				Cyclic_intervalFlag = false;//
				gRTCTime_arrive_Flag = false;//
				gTime_arrive_Flag = false; //
				Cyclic_timing = false;//
				Irrigation_use = false;//
				Need_Num = 0;	//需要开启继电器的个数
				Complete_Num = 0;//完成开启的个数

				for (size_t i = 0; i < 8; i++)
				{
					Worktime[2 * i] = 0;
					Worktime[(2 * i) + 1] = 0;
					Worktime_backups[2 * i] = 0;
					Worktime_backups[(2 * i) + 1] = 0;
					Set_Irrigation_relay(i, OFF);
				}

				OpenSec = 0;//开启时间为0s
				DO_Interval = 0;//间隔时间为0s
				Cyclic_interval = 0;//循环间隔为0s（0）
				DO_Num = 1;//每次开启1路
				retryCnt = 0;//0次循环
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
			Serial.println("Start circulating irrigation button press <Key_cycle_irrigation>");
			Serial.println(String("KeyDI7_num = ") + KeyDI7_num);
			Serial.println("");
			/*等待按键释放*/
			while (digitalRead(DI7) == HIGH)
			{

			}

			if (KeyDI7_num % 2)//代表需要启动
			{
				Stop_Timer4();//先停止计时
				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
				Cyclic_intervalFlag = false;//
				gRTCTime_arrive_Flag = false;//
				gTime_arrive_Flag = false; //
				Cyclic_timing = false;//
				Irrigation_use = false;//
				Need_Num = 0;	//需要开启继电器的个数
				Complete_Num = 0;//完成开启的个数

				for (size_t i = 0; i < 8; i++)
				{
					Worktime[2 * i] = 0;
					Worktime[(2 * i) + 1] = 0;
					Worktime_backups[2 * i] = 0;
					Worktime_backups[(2 * i) + 1] = 0;
					Set_Irrigation_relay(i, OFF);
				}

				OpenSec = 20;//开启时间为20s
				DO_Interval = 2;//间隔时间为2s
				Cyclic_interval = InitState.Read_CyclicInterval();//循环间隔为1小时（3600）
				DO_Num = 1;//每次开启1路
				retryCnt = 0xFFFF;//65535次循环

				for (size_t i = 0; i < 8; i++)
				{
					Worktime[2 * i] = DO_Interval;
					Worktime[(2 * i) + 1] = OpenSec;
					Worktime_backups[2 * i] = DO_Interval;
					Worktime_backups[(2 * i) + 1] = OpenSec;
				}
				Need_Num = 8;
			}
			else//代表关闭
			{
				Stop_Timer4();//先停止计时
				rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//RTC报警特定中断,不知道具体是干嘛的，注释掉好像也一样的跑？
				Cyclic_intervalFlag = false;//
				gRTCTime_arrive_Flag = false;//
				gTime_arrive_Flag = false; //
				Cyclic_timing = false;//
				Irrigation_use = false;//
				Need_Num = 0;	//需要开启继电器的个数
				Complete_Num = 0;//完成开启的个数

				for (size_t i = 0; i < 8; i++)
				{
					Worktime[2 * i] = 0;
					Worktime[(2 * i) + 1] = 0;
					Worktime_backups[2 * i] = 0;
					Worktime_backups[(2 * i) + 1] = 0;
					Set_Irrigation_relay(i, OFF);
				}

				OpenSec = 0;//开启时间为0s
				DO_Interval = 0;//间隔时间为0s
				Cyclic_interval = 0;//循环间隔为0s（0）
				DO_Num = 1;//每次开启1路
				retryCnt = 0;//0次循环
			}
		}
	}
#else

#endif
}

/*
 @brief   : 串口设置循环间隔
 @para    : 无
 @return  : 无
 */
void Com_Set_Cyclic_interval(void)
{
	while (Serial.available() > 0)
	{
		comdata += char(Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Serial.println(comdata);
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
				Serial.println("设置并保存参数成功 <Com_Set_Cyclic_interval>");
			}
			else
			{
				Serial.println("设置并保存参数失败!!! <Com_Set_Cyclic_interval>");
			}
			Cyclic_interval = InitState.Read_CyclicInterval();
			Serial.println(String("Cyclic_interval = ") + Cyclic_interval);
			comdata = "";
		}
		else
		{
			comdata = "";
			Serial.println("输入错误 <Com_Set_Cyclic_interval>");
		}
	}
}

