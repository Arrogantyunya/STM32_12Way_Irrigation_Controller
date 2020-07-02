/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/3/11
 * 该文件主要功能是依托EEPROM和芯片自带备份寄存器来保存、校验、操作、读取设备相关的信息，
 * 如通用的SN码，软件、硬件版本号，区域号，工作组号等。私有信息如电机卷膜开度、测量行程标志
 * 位等。头文件中提供了各个类的公共接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/
#include "Memory.h"
#include "User_CRC8.h"
#include "fun_periph.h"
#include "Set_coil.h"
#include <libmaple/bkp.h>
#include <libmaple/iwdg.h>

// #include "boot_config.h"

/*创建EEPROM操作对象*/
EEPROM_Operations EEPROM_Operation;
/*创建SN码操作对象*/
SN_Operations SN;
/*创建LoRa配置对象*/
LoRa_Config LoRa_Para_Config;
/*创建软件硬件版本对象*/
Soft_Hard_Vertion Vertion;
/*创建Modbus控制器初始状态对象*/
ModbusController_InitState InitState;
/*创建正反转对象*/
Positive_Negative_MODE Pos_Nega_mode;

/*
 @brief     : 设置EEPROM读写引脚
 @para      : 无
 @return    : 无
 */
void EEPROM_Operations::EEPROM_GPIO_Config(void)
{
	pinMode(EP_WP_PIN, OUTPUT);
	digitalWrite(EP_WP_PIN, HIGH);
	I2C_Init();
}

/*
 @brief     : 重置EEPROM的值
 @para      : 无
 @return    : 无
 */
void EEPROM_Operations::EEPROM_Reset(void)
{
	EEPROM_Write_Enable();
	for (size_t i = EEPROM_MIN_ADDR; i < EEPROM_MAX_ADDR; i++)
	{
		AT24CXX_WriteOneByte(i, 0x00);
	}
	EEPROM_Write_Disable();
}

/*
 @brief     : 测试AT24C128
 @para      : 无
 @return    : 无
 */
unsigned char EEPROM_Operations::EEPROM_AT24C128_Test(void)
{
	iwdg_feed();
	EEPROM_Write_Enable();
	for (size_t i = EEPROM_MIN_ADDR; i < EEPROM_MAX_ADDR; i++)
	{
		AT24CXX_WriteOneByte(i, 0x11);
		if (i%500 == 0)
		{
			Debug_Println(String("[Debug] i = ") + i);
			iwdg_feed();
		}
	}
	EEPROM_Write_Disable();

	Debug_Println("....");

	for (size_t i = EEPROM_MIN_ADDR; i < EEPROM_MAX_ADDR; i++)
	{
		if (i%500 == 0)
		{
			Debug_Println(String("[Debug] i = ") + i);
			iwdg_feed();
		}
		if (!AT24CXX_ReadOneByte(i) == 0x11)
			return false;
	}

	return true;
}

/*
 @brief     : 将AT24C128的值重置
 @para      : 无
 @return    : 无
 */
unsigned char EEPROM_Operations::EEPROM_AT24C128_Reset(void)
{
	iwdg_feed();
	EEPROM_Write_Enable();
	for (size_t i = EEPROM_MIN_ADDR; i < EEPROM_MAX_ADDR; i++)
	{
		AT24CXX_WriteOneByte(i, 0x00);
		if (i%500 == 0)
		{
			Debug_Println(String("[Debug] i = ") + i);
			iwdg_feed();
		}
	}
	EEPROM_Write_Disable();

	Debug_Println("+++++++++");

	for (size_t i = EEPROM_MIN_ADDR; i < EEPROM_MAX_ADDR; i++)
	{
		if (!AT24CXX_ReadOneByte(i) == 0x00)
			return false;
		if (i%500 == 0)
		{
			Debug_Println(String("[Debug] i = ") + i);
			iwdg_feed();
		}
	}

	return true;
}

/*
 @brief     : 写SN码到EEPROM
 @para      : SN码数组
 @return    : true or false
 */
bool SN_Operations::Save_SN_Code(unsigned char *sn_code)
{
	EEPROM_Write_Enable();
	for (unsigned char i = 0; i < 9; i++)
		AT24CXX_WriteOneByte(SN_BASE_ADDR + i, sn_code[i]);

	unsigned char SN_Verify = GetCrc8(&sn_code[0], 9); //得到SN码的CRC8
	AT24CXX_WriteOneByte(SN_VERIFY_ADDR, SN_Verify);

	/*读出写入的SN码，生成一个CRC8，和写入的CRC8校验，判断写入是否成功*/
	unsigned char SN_CodeTemp[9];
	for (unsigned char i = 0; i < 9; i++)
		SN_CodeTemp[i] = AT24CXX_ReadOneByte(SN_BASE_ADDR + i);

	unsigned char SN_VerifyTemp = GetCrc8(&SN_CodeTemp[0], 9);
	if (SN_VerifyTemp != SN_Verify)
	{
		EEPROM_Write_Disable();
		return false;
	}
	else
	{
		if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) != 0x55)
			AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x55); //校验成功，SN写入成功标志位置0x55。

		EEPROM_Write_Disable();
		return true;
	}
}

/*
 @brief     : 写SN码到EEPROM的SN备份地址
			  注释参考 Save_SN_Code 函数
 @para      : SN 数组
 @return    : true or false
 */
bool SN_Operations::Save_BKP_SN_Code(unsigned char *sn_code)
{
	EEPROM_Write_Enable();
	for (unsigned char i = 0; i < 9; i++)
		AT24CXX_WriteOneByte(SN_BKP_BASE_ADDR + i, sn_code[i]);

	unsigned char SN_BKP_Verify = GetCrc8(&sn_code[0], 9);
	AT24CXX_WriteOneByte(SN_BKP_VERIFY_ADDR, SN_BKP_Verify);

	unsigned char SN_CodeTemp[9];
	for (unsigned char i = 0; i < 9; i++)
		SN_CodeTemp[i] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i);

	unsigned char SN_BKP_VerifyTemp = GetCrc8(&SN_CodeTemp[0], 9);
	if (SN_BKP_VerifyTemp != SN_BKP_Verify)
	{
		EEPROM_Write_Disable();
		return false;
	}
	else
	{
		if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) != 0x55)
			AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x55);

		EEPROM_Write_Disable();
		return true;
	}
}

/*
 @brief     : 从EEPROM读取SN码
 @para      : SN数组
 @return    : true or false
 */
bool SN_Operations::Read_SN_Code(unsigned char *sn_code)
{
	unsigned char SN_Temp[9] = {0};

	for (unsigned char i = 0; i < 9; i++)
		SN_Temp[i] = AT24CXX_ReadOneByte(SN_BASE_ADDR + i);

	unsigned char SN_Verify = AT24CXX_ReadOneByte(SN_VERIFY_ADDR);
	unsigned char SN_VerifyTemp = GetCrc8(&SN_Temp[0], 9);

	/*校验数据CRC8*/
	if (SN_Verify != SN_VerifyTemp)
		return false;
	else
	{
		for (unsigned char i = 0; i < 9; i++)
			sn_code[i] = SN_Temp[i];
		return true;
	}
}

/*
 @brief     : 从EEPROM读取备份的SN码
			  注释参考 Read_SN_Code 函数
 @para      : SN数组
 @return    : true or false
 */
bool SN_Operations::Read_BKP_SN_Code(unsigned char *sn_code)
{
	unsigned char SN_BKP_Temp[9] = {0};

	for (unsigned char i = 0; i < 9; i++)
		SN_BKP_Temp[i] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i);

	unsigned char SN_BKP_Verify = AT24CXX_ReadOneByte(SN_BKP_VERIFY_ADDR);
	unsigned char SN_BKP_VerifyTemp = GetCrc8(&SN_BKP_Temp[0], 9);

	if (SN_BKP_Verify != SN_BKP_VerifyTemp)
		return false;
	else
	{
		for (unsigned char i = 0; i < 9; i++)
			sn_code[i] = SN_BKP_Temp[i];
		return true;
	}
}

/*
 @brief     : 取保存在SN备份地址的SN码最后两位，生成随机数种子。
			  该随机数种子用于防止堵塞的回执应答随机延时。
 @para      : 随机种子指针变量
 @return    : 无
 */
void SN_Operations::Read_Random_Seed(unsigned char *random_seed)
{
	unsigned char RandomTemp[2];
	RandomTemp[0] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + 7);
	RandomTemp[1] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + 8);
	*random_seed = GetCrc8(RandomTemp, 2);
}

// /*
//  @brief     : 验证是否已经写入SN码，0x55表示已经写入SN码
//  @para      : 无
//  @return    : true or false
//  */
// bool SN_Operations::Verify_Save_SN_Code(void)
// {
// 	if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x55)
// 		return true;
// 	else
// 		return false;
// }

// /*
//  @brief     : 验证是否已经写入备份SN码，0x55表示已经写入SN码
//  @para      : None
//  @return    : true or false
//  */
// bool SN_Operations::Verify_Save_BKP_SN_Code(void)
// {
// 	if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x55)
// 		return true;
// 	else
// 		return false;
// }

/*
 @brief     : 清除保存SN码标志位，慎用！
			  如果清除了该标志位，下一次写入SN事件会无视已保存的SN码，覆盖它。
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Clear_SN_Save_Flag(void)
{
	if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x00)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x00);
	EEPROM_Write_Disable();
	/*判断是否已成功清除该标志位*/
	if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x00)
		return true;
	else
		return false;
}

/*
 @brief     : 清除保存备份SN码标志位，慎用！
			  如果清除了该标志位，下一次写入SN事件会无视已保存的SN码，覆盖它。
			  注释参考 Clear_SN_Save_Flag 函数
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Clear_BKP_SN_Save_Flag(void)
{
	if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x00)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x00);
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x00)
		return true;
	else
		return false;
}

/*
 @brief     : 如果注册到服务器成功，设置注册成功标志位
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Set_SN_Access_Network_Flag(void)
{
	if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(SN_ACCESS_NETWORK_FLAG_ADDR, 0X55);
	EEPROM_Write_Disable();
	/*判断标志位是否写入成功*/
	if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55)
		return true;
	else
		return false;
}

/*
 @brief     : 清除注册到服务器标志位，这将意味着本机将重新请求注册到服务器，慎用！
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Clear_SN_Access_Network_Flag(void)
{
	Info_Println("If you need to clear the registration, send \"YES\"");
	Debug_Serial.flush();
	iwdg_feed();
	delay(5000);
	iwdg_feed();
	while (Debug_Serial.available() > 0)
	{
		comdata += char(Debug_Serial.read());
		delay(2);
	}
	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Info_Println(comdata);

		if (comdata == String("YES"))
		{
			Info_Println("Clear the Registration");

			comdata = "";

			if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) != 0x00)
			{
				EEPROM_Write_Enable();
				AT24CXX_WriteOneByte(SN_ACCESS_NETWORK_FLAG_ADDR, 0x00);
				EEPROM_Write_Disable();
			}
			/*验证标志位是否清除成功*/
			if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x00)
				return true;
			else
				return false;
		}
		else
		{
			Error_Println("Input error");

			comdata = "";
			return false;
		}
	}
	else
	{
		Info_Println("END");
		comdata = "";
		return false;
	}
	return false;
}

/*
 @brief     : 验证是否已经注册到服务器，0x55表示已经注册。
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Verify_SN_Access_Network_Flag(void)
{
	bool BoolValue;
	AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55 ? BoolValue = true : BoolValue = false;
	return BoolValue;
}

/*
 @brief     : 验证读取的SN是否错误，如果其中一个SN失败，则用另一个完整的SN覆盖它。
 @para      : SN数组
 @return    : true or false
 */
bool SN_Operations::Self_check(unsigned char *sn_code)
{
	bool BoolValue = false;
	if (Read_SN_Code(sn_code) && Read_BKP_SN_Code(sn_code)) //如果读取SN码和备份SN码都正确
	{
		Info_Println("Read SN code and read backup SN code success...");
		BoolValue = true;
	}
	else if (Read_SN_Code(sn_code)) //如果仅仅是读取SN码成功，将SN码覆盖修正备份SN码保存区
	{
		Error_Println("Read SN code success but read backup SN code failed...");
		Save_BKP_SN_Code(sn_code) == true ? BoolValue = true : BoolValue = false; //覆盖后判断覆盖是否成功
	}
	else if (Read_BKP_SN_Code(sn_code)) //如果仅仅是读取备份SN码成功，将SN码覆盖修正SN码保存区
	{
		Error_Println("Read backup SN code success but read SN code failed...");
		Save_SN_Code(sn_code) == true ? BoolValue = true : BoolValue = false; //覆盖后判断覆盖是否成功
	}
	else //如果SN码和备份SN码都损坏，清除已保存SN码标志位，已保存备份SN码标志位，设备将需要重新申请一份SN码
	{
		Error_Println("All SN store ERROR!");
		Clear_SN_Save_Flag();
		Clear_BKP_SN_Save_Flag();
	}
	return BoolValue; //返回操作的最终结果
}

/*
 @brief     : 保存工作组号
 @para      : group number(array, 5byte)
 @return    : true or false
 */
bool SN_Operations::Save_Group_Number(unsigned char *group_num)
{
	bool SaveGroupFlag = false;

	/*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
	for (unsigned char i = 0; i < 5; i++)
	{
		if (group_num[i] != AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i))
		{
			SaveGroupFlag = true;
			break;
		}
	}
	if (SaveGroupFlag == false)
		return true;

	EEPROM_Write_Enable();
	for (unsigned char i = 0; i < 5; i++)
		AT24CXX_WriteOneByte(GROUP_NUMBER_BASE_ADDR + i, group_num[i]);

	unsigned char CRC8 = GetCrc8(&group_num[0], 5);
	AT24CXX_WriteOneByte(GROUP_NUMBER_VERIFY_ADDR, CRC8);

	/*判断是否保存数据成功*/
	for (unsigned char i = 0; i < 5; i++)
	{
		if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != group_num[i])
		{
			EEPROM_Write_Disable();
			return false;
		}
	}

	if (AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR) == CRC8)
	{
		if (AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) != 0x55)
		{
			AT24CXX_WriteOneByte(GROUP_NUMBER_FLAG_ADDR, 0x55); //保存数据成功，置标志位
		}
		EEPROM_Write_Disable();
		return true;
	}
	else
	{
		EEPROM_Write_Disable();
		return false;
	}
}

/*
 @brief     : 读取工作组号
 @para      : workgroup number(array 5byte)
 @return    : true or false
 */
bool SN_Operations::Read_Group_Number(unsigned char *group_num)
{
	unsigned char GroupNumber[5];
	unsigned char CRC8;
	for (unsigned char i = 0; i < 5; i++)
		GroupNumber[i] = AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i);

	CRC8 = GetCrc8(&GroupNumber[0], 5);
	if (CRC8 != AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR))
		return false;
	else
	{
		for (unsigned char i = 0; i < 5; i++)
			group_num[i] = GroupNumber[i];
	}
	return true;
}

/*
 @brief     : 验证保存的工作组号是否损坏
 @para      : None
 @return    : true or false
*/
bool SN_Operations::Check_Group_Number(void)
{
	unsigned char GroupTemp[5];

	for (unsigned char i = 0; i < 5; i++)
		GroupTemp[i] = AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i);

	unsigned char VerifyCRC8_Temp = GetCrc8(&GroupTemp[0], 5);
	if (VerifyCRC8_Temp == AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR))
		return true;
	else
		return false;
}

/*
 @brief     : 验证是否已经保存过工作组号
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Verify_Group_Number_Flag(void)
{
	bool BoolValue;
	AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) == 0X55 ? BoolValue = true : BoolValue = false;
	return BoolValue;
}

/*
 @brief     : 清除本机的工作组号，慎用！
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Clear_Group_Number(void)
{
	EEPROM_Write_Enable();
	for (unsigned char i = 0; i < 5; i++)
	{
		if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != 0x00)
		{
			AT24CXX_WriteOneByte(GROUP_NUMBER_BASE_ADDR + i, 0x00);
			if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != 0x00)
			{
				EEPROM_Write_Disable();
				return false;
			}
		}
	}
	if (AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) != 0x00) //保护EP，防止重复擦写。
		AT24CXX_WriteOneByte(GROUP_NUMBER_FLAG_ADDR, 0x00);	 //同时清除保存组号标志位

	EEPROM_Write_Disable();
	return true;
}

/*
 @brief     : 保存区域号
 @para      : area number
 @return    : true or false
 */
bool SN_Operations::Save_Area_Number(unsigned char area_num)
{
	/*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
	if (area_num == AT24CXX_ReadOneByte(AREA_ADDR))
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(AREA_ADDR, area_num);
	AT24CXX_WriteOneByte(AREA_VERIFY_ADDR, GetCrc8(&area_num, 1));
	/*验证是否保存数据成功*/
	if (AT24CXX_ReadOneByte(AREA_ADDR) == area_num)
	{
		if (AT24CXX_ReadOneByte(AREA_FLAG_ADDR) != 0x55)
		{
			AT24CXX_WriteOneByte(AREA_FLAG_ADDR, 0x55);
		}
		EEPROM_Write_Disable();
		return true;
	}
	else
	{
		EEPROM_Write_Disable();
		return false;
	}
}

/*
 @brief     : 读取区域号
 @para      : None
 @return    : Area number.
 */
unsigned char SN_Operations::Read_Area_Number(void)
{
	return (AT24CXX_ReadOneByte(AREA_ADDR));
}

/*
 @brief     : 验证保存的区域号是否损坏
 @para      : None
 @return    : true or false
*/
bool SN_Operations::Check_Area_Number(void)
{
	unsigned char AreaTemp = AT24CXX_ReadOneByte(AREA_ADDR);
	unsigned char VerifyTemp = GetCrc8(&AreaTemp, 1);

	if (VerifyTemp == AT24CXX_ReadOneByte(AREA_VERIFY_ADDR))
		return true;
	else
		return false;
}

/*
 @brief     : 验证区域号是否已经保存
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Verify_Area_Number_Flag(void)
{
	bool BoolValue;
	AT24CXX_ReadOneByte(AREA_FLAG_ADDR) == 0x55 ? BoolValue = true : BoolValue = false;
	return BoolValue;
}

/*
 @brief     : 清除区域号，慎用！
 @para      : None
 @return    : true or fasle
 */
bool SN_Operations::Clear_Area_Number(void)
{
	if (AT24CXX_ReadOneByte(AREA_ADDR) == 0x00)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(AREA_ADDR, 0X00); //默认区域号
	if (AT24CXX_ReadOneByte(AREA_ADDR) != 0x00)
	{
		EEPROM_Write_Disable();
		return false;
	}
	else
	{
		EEPROM_Write_Disable();
		return true;
	}
}

/*
 @brief     : 读取实时卷膜开度值
 @para      : None
 @return    : opening value(0 - 100)
 */
unsigned char SN_Operations::Read_RealTime_Opening_Value(void)
{
	return 0;
}

/*
 @brief     : 读取最大电流阈值。阈值步进0.1倍，最大10倍
 @para      : 无
 @return    : 电流阈值
*/
unsigned char SN_Operations::Read_Roll_High_Current_Limit_Value(void)
{
	return 0;
}

/*
 @brief     : 读取最小电压阈值
 @para      : 无
 @return    : 最小电压值
 */
unsigned char SN_Operations::Read_Roll_Low_Voltage_Limit_Value(void)
{
	return 0;
}

/*
 @brief     : 读取上报实时状态间隔值
			  Read report interval value of realtime status.
 @para      : None
 @return    : interval value.
 */
unsigned char SN_Operations::Read_Roll_Report_Status_Interval_Value(void)
{
	return 0;
}

/*
 @brief     : 配置LoRa参数完成标志位
 @para      : 无
 @return    : true or false
 */
bool LoRa_Config::Save_LoRa_Config_Flag(void)
{
	if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(LORA_PARA_CONFIG_FLAG_ADDR, 0x55);
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
		return true;
	else
		return false;
}

/*
 @brief     : 验证是否已经配置了LoRa参数
 @para      : 无
 @return    : true or false
 */
bool LoRa_Config::Verify_LoRa_Config_Flag(void)
{
	if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
		return true;
	else
		return false;
}

/*
 @brief     : 清除配置LoRa成功标志位
 @para      : 无
 @return    : true or false
 */
bool LoRa_Config::Clear_LoRa_Config_Flag(void)
{
	if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x00)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(LORA_PARA_CONFIG_FLAG_ADDR, 0x00);
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x00)
		return true;
	else
		return false;
}

bool LoRa_Config::Save_LoRa_Com_Mode_Flag(void)
{
	if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR) == 0x55)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(LORA_COM_MODE_FLAG_ADDR, 0x55);
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR == 0x55))
		return true;
	else
		return false;
}

bool LoRa_Config::Clear_LoRa_Com_Mode_Flag(void)
{
	if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR) == 0x00)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(LORA_COM_MODE_FLAG_ADDR, 0x00);
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR == 0x00))
		return true;
	else
		return false;
}

bool LoRa_Config::Save_LoRa_Com_Mode(unsigned char mode)
{
	unsigned char ModeCrc8;

	if (mode == 0xF0 || mode == 0xF1)
	{
		if (AT24CXX_ReadOneByte(LORA_COM_MODE_ADDR) == mode)
			return true;

		EEPROM_Write_Enable();
		AT24CXX_WriteOneByte(LORA_COM_MODE_ADDR, mode);
		ModeCrc8 = GetCrc8(&mode, 1);
		AT24CXX_WriteOneByte(LORA_COM_MODE_VERIFY_ADDR, ModeCrc8);

		if (AT24CXX_ReadOneByte(LORA_COM_MODE_ADDR) == mode)
		{
			Save_LoRa_Com_Mode_Flag();
			EEPROM_Write_Disable();
			return true;
		}
		else
		{
			EEPROM_Write_Disable();
			return false;
		}
	}
	else
	{
		Error_Println("Save LoRa communication mode Err");
		return false;
	}
}

unsigned char LoRa_Config::Read_LoRa_Com_Mode(void)
{
	unsigned char ComTemp;
	unsigned char Comcrc8;

	ComTemp = AT24CXX_ReadOneByte(LORA_COM_MODE_ADDR);
	Comcrc8 = GetCrc8(&ComTemp, 1);

	if (Comcrc8 == AT24CXX_ReadOneByte(LORA_COM_MODE_VERIFY_ADDR))
	{
		return ComTemp;
	}
	else
	{
		Save_LoRa_Com_Mode(0xF0);
		return 0xF0;
	}
}

bool LoRa_Config::Save_LoRa_Addr(unsigned char *addr)
{
	unsigned char TempBuf[8];
	unsigned char AddrCrc8, AddrTempCrc8;

	EEPROM_Write_Enable();
	AddrCrc8 = GetCrc8(&addr[0], 8);
	for (unsigned char i = 0; i < 8; i++)
	{
		AT24CXX_WriteOneByte(EP_LORA_ADDR_BASE_ADDR + i, addr[i]);
	}
	AT24CXX_WriteOneByte(EP_LORA_ADDR_VERIFY_ADDR, AddrCrc8);

	for (unsigned char i = 0; i < 8; i++)
	{
		TempBuf[i] = AT24CXX_ReadOneByte(EP_LORA_ADDR_BASE_ADDR + i);
		// Serial.println(String("TempBuf[") + i + "]=" + TempBuf[i]);
	}
	AddrTempCrc8 = GetCrc8(&TempBuf[0], 8);

	if (AddrCrc8 != AddrTempCrc8)
	{
		Clear_LoRa_Addr_Flag();
		EEPROM_Write_Disable();
		return false;
	}
	else
	{
		Info_Println("Save LoRa Addr OK...");
		Save_LoRa_Addr_Flag();
		return true;
	}
}

bool LoRa_Config::Read_LoRa_Addr(unsigned char *addr)
{
	unsigned char TempBuf[8];
	unsigned char AddrCrc8, AddrTempCrc8;

	for (unsigned char i = 0; i < 8; i++)
	{
		TempBuf[i] = AT24CXX_ReadOneByte(EP_LORA_ADDR_BASE_ADDR + i);
	}
	AddrCrc8 = GetCrc8(&TempBuf[0], 8);
	AddrTempCrc8 = AT24CXX_ReadOneByte(EP_LORA_ADDR_VERIFY_ADDR);

	if (AddrCrc8 != AddrTempCrc8)
		return false;

	for (unsigned char i = 0; i < 8; i++)
	{
		addr[i] = TempBuf[i];
	}
	Info_Println("Read LoRa Addr OK...");
	return true;
}

void LoRa_Config::Save_LoRa_Addr_Flag(void)
{
	if (AT24CXX_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0X55)
		return;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR, 0x55);
	EEPROM_Write_Disable();
}

void LoRa_Config::Clear_LoRa_Addr_Flag(void)
{
	if (AT24CXX_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0x00)
		return;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR, 0x00);
	EEPROM_Write_Disable();
}

bool LoRa_Config::Verify_LoRa_Addr_Flag(void)
{
	if (AT24CXX_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0x55)
		return true;
	else
		return false;
}

bool LoRa_Config::Verify_LoRa_TRMode_Flag(void)//验证Lora的TRmode是否被配置过
{
    if (AT24CXX_ReadOneByte(Lora_TRMode_Set_ADDR) == 0x55)
        return true;
	else
	{
		EEPROM_Write_Enable();
		AT24CXX_WriteOneByte(Lora_TRMode_Set_ADDR, 0x55);
		EEPROM_Write_Disable();
		return false;
	}
}

bool LoRa_Config::Save_LoRa_TRMode(unsigned char TRMode)//保存TRMode
{
    if (AT24CXX_ReadOneByte(Lora_TRMode_ADDR) == TRMode)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(Lora_TRMode_ADDR, TRMode);
	EEPROM_Write_Disable();

    return true;
}

unsigned char LoRa_Config::Read_LoRa_TRMode(void)//读取TRMode
{
    unsigned char TRMode = AT24CXX_ReadOneByte(Lora_TRMode_ADDR);
    return TRMode;
}


bool LoRa_Config::Verify_LoRa_SYNC_Flag(void)//验证Lora的SYNC是否被配置过
{
    if (AT24CXX_ReadOneByte(Lora_SYNC_Set_ADDR) == 0x55)
		return true;
	else
	{
		EEPROM_Write_Enable();
		AT24CXX_WriteOneByte(Lora_SYNC_Set_ADDR, 0x55);
		EEPROM_Write_Disable();
		return false;
	}
}

bool LoRa_Config::Save_LoRa_SYNC(unsigned char SYNC)//保存SYNC
{
    if (AT24CXX_ReadOneByte(LORA_SYNC_ADDR) == SYNC)
		return true;

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(LORA_SYNC_ADDR, SYNC);
	EEPROM_Write_Disable();

    return true;
}

unsigned char LoRa_Config::Read_LoRa_SYNC(void)//读取SYNC
{
    unsigned char LoRa_SYNC = AT24CXX_ReadOneByte(LORA_SYNC_ADDR);
    return LoRa_SYNC;
}





/*
 @brief     : 保存该设备的软件版本号
			  Save device's software version.
 @para      : Version number high byte, version number low byte.
 @return    : None
 */
void Soft_Hard_Vertion::Save_Software_version(unsigned char number_high, unsigned char number_low)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(SOFT_VERSION_BASE_ADDR, number_high);
	AT24CXX_WriteOneByte(SOFT_VERSION_BASE_ADDR + 1, number_low);
	EEPROM_Write_Disable();
}

unsigned char Soft_Hard_Vertion::Read_hardware_version(unsigned char number_addr)
{
	unsigned char version = AT24CXX_ReadOneByte(number_addr);
	return version;
}

unsigned char Soft_Hard_Vertion::Read_Software_version(unsigned char number_addr)
{
	unsigned char version = AT24CXX_ReadOneByte(number_addr);
	return version;
}

/*
 @brief     : 保存该设备的硬件版本号
			  Save device's hardware version.
 @para      : Version number high byte, version number low byte.
 @return    : None
 */
void Soft_Hard_Vertion::Save_hardware_version(unsigned char number_high, unsigned char number_low)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(HARD_VERSION_BASE_ADDR, number_high);
	AT24CXX_WriteOneByte(HARD_VERSION_BASE_ADDR + 1, number_low);
	EEPROM_Write_Disable();
}

/* 
 @brief     : 存储初始状态的标志位
			  Save device's InitState Flag.
 @para      : void
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_InitState_Flag(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(INIT_STATE_FLAG_ADDR, 0x55);
	EEPROM_Write_Disable();
	if (AT24CXX_ReadOneByte(INIT_STATE_FLAG_ADDR) == 0x55)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* 
 @brief     : 读取初始状态的标志位
			  Read device's InitState Flag.
 @para      : 
 @return    : 保存过初始状态返回true,未保存过返回false
 */
bool ModbusController_InitState::Read_InitState_Flag(void)
{
	if (AT24CXX_ReadOneByte(INIT_STATE_FLAG_ADDR) == 0x55)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* 
 @brief     : 清除初始状态的标志位
			  Clean device's InitState Flag.
 @para      : 
 @return    : 清除成功true，失败false
 */
bool ModbusController_InitState::Clean_InitState_Flag(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(INIT_STATE_FLAG_ADDR, 0x00);
	EEPROM_Write_Disable();
	if (AT24CXX_ReadOneByte(INIT_STATE_FLAG_ADDR) == 0x00)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* 
 @brief     : 存储DO初始状态
			  Save device's DO_InitState.
 @para      : 
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_DO_InitState(unsigned char *data)
{
	unsigned char DO_Init[8] = {0x00};

	EEPROM_Write_Enable();
	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		//Serial.println(String(*(data+i)));
		AT24CXX_WriteOneByte(DO_INIT_STATE_BASE_ADDR + i, *(data + i));
		//Serial.println(String(AT24CXX_ReadOneByte(DO_INIT_STATE_BASE_ADDR + i))+"~");
	}
	unsigned char DO_CRC8 = GetCrc8(data, 8);
	AT24CXX_WriteOneByte(DO_INIT_STATE_VERIFY_ADDR, DO_CRC8);
	//Serial.println(String("DO_CRC8 = ") + DO_CRC8);

	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		DO_Init[i] = AT24CXX_ReadOneByte(DO_INIT_STATE_BASE_ADDR + i);
	}
	unsigned char DO_crc8 = GetCrc8(DO_Init, 8);

	EEPROM_Write_Disable();

	if (DO_crc8 == DO_CRC8)
	{
		Info_Println("保存DO初始化值成功");
		return true;
	}

	Error_Println("保存DO初始化值失败!!!");
	return false;
}

/* 
 @brief     : 读取DO初始状态
			  Read device's DO_InitState.
 @para      : 
 @return    : None
 */
unsigned char *ModbusController_InitState::Read_DO_InitState(void)
{
	Info_Println("");
	static unsigned char DO_INIT[8] = {0x00}; //C 不支持在函数外返回局部变量的地址，除非定义局部变量为 static 变量。
	for (size_t i = 0; i < 8; i++)
	{
		DO_INIT[i] = AT24CXX_ReadOneByte(DO_INIT_STATE_BASE_ADDR + i);
	}
	/*这里需不需要加数组的校验呢*/
	return DO_INIT;
}

/* 
 @brief     : 清除DO初始状态
			  Clean device's DO_InitState.
 @para      : 
 @return    : None
 */
bool ModbusController_InitState::Clean_DO_InitState()
{
	Info_Println("暂时未完成");
	return false;
}

/* 
 @brief     : 存储AO初始状态
			  Save device's AO_InitState.
 @para      : 
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_AO_InitState(unsigned char *data)
{
	unsigned char AO_Init[8] = {0x00};

	EEPROM_Write_Enable();
	for (size_t i = 0; i < 8; i++)
	{
		//Serial.println(String(*(data+i)));
		AT24CXX_WriteOneByte(AO_INIT_STATE_BASE_ADDR + i, *(data + i));
		//Serial.println(String(AT24CXX_ReadOneByte(AO_INIT_STATE_BASE_ADDR + i))+"~");
	}
	unsigned char AO_CRC8 = GetCrc8(data, 8);
	AT24CXX_WriteOneByte(AO_INIT_STATE_VERIFY_ADDR, AO_CRC8);
	//Serial.println(String("AO_CRC8 = ") + AO_CRC8);

	for (size_t i = 0; i < 8; i++)
	{
		AO_Init[i] = AT24CXX_ReadOneByte(AO_INIT_STATE_BASE_ADDR + i);
	}
	unsigned char AO_crc8 = GetCrc8(AO_Init, 8);

	EEPROM_Write_Disable();

	if (AO_crc8 == AO_CRC8)
	{
		Info_Println("保存AO初始化值成功");
		return true;
	}

	Error_Println("保存AO初始化值失败!!!");
	return false;
}

/* 
 @brief     : 读取AO初始状态
			  Read device's AO_InitState.
 @para      : 
 @return    : None
 */
bool ModbusController_InitState::Read_AO_InitState(unsigned char *data)
{
	data[0] = data[0];
	Info_Println("暂时未完成");
	return false;
}

/* 
 @brief     : 清除AO初始状态
			  Clean device's AO_InitState.
 @para      : 
 @return    : None
 */
bool ModbusController_InitState::Clean_AO_InitState()
{
	Info_Println("暂时未完成");
	return false;
}

/*
 @brief     : 存储timeout时间
			  Save timeout.
 @para      :
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_Timeout(unsigned char High, unsigned char Low)
{
	if ((AT24CXX_ReadOneByte(Timeout_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(Timeout_END_ADDR) == Low))
	{
		Info_Println("储存Timeout成功");
		return true;
	}
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(Timeout_BASE_ADDR, High);
	AT24CXX_WriteOneByte(Timeout_END_ADDR, Low);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(Timeout_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(Timeout_END_ADDR) == Low))
	{
		Info_Println("储存Timeout成功");
		return true;
	}
	Error_Println("储存Timeout失败");
	return false;
}

/*
 @brief     : 读取timeout时间
			  Save timeout.
 @para      :
 @return    : Timeout
 */
unsigned int ModbusController_InitState::Read_Timeout(void)
{
	unsigned int Timeout = AT24CXX_ReadOneByte(Timeout_BASE_ADDR) * 0x100 + AT24CXX_ReadOneByte(Timeout_END_ADDR);
	return Timeout;
}

/*
 @brief     : 清除timeout时间
			  Save timeout.
 @para      :
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Clean_Timeout(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(Timeout_BASE_ADDR, 0x00);
	AT24CXX_WriteOneByte(Timeout_END_ADDR, 0x00);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(Timeout_BASE_ADDR) == 0x00) && (AT24CXX_ReadOneByte(Timeout_END_ADDR) == 0x00))
	{
		Info_Println("清除Timeout成功");
		return true;
	}
	Error_Println("清除Timeout失败");
	return false;
}

/*
 @brief     : 存储E000Interval时间
			  Save E000Interval.
 @para      :
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_E000Interval(unsigned char High, unsigned char Low)
{
	if ((AT24CXX_ReadOneByte(E000Interval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(E000Interval_END_ADDR) == Low))
	{
		Info_Println("储存E000Interval成功");
		return true;
	}

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(E000Interval_BASE_ADDR, High);
	AT24CXX_WriteOneByte(E000Interval_END_ADDR, Low);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(E000Interval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(E000Interval_END_ADDR) == Low))
	{
		Info_Println("储存E000Interval成功");
		return true;
	}
	Error_Println("储存E000Interval失败");
	return false;
}

/*
 @brief     : 读取E000Interval时间
			  Read E000Interval.
 @para      :
 @return    : E000Interval
 */
unsigned int ModbusController_InitState::Read_E000Interval(void)
{
	unsigned int E000Interval = AT24CXX_ReadOneByte(E000Interval_BASE_ADDR) * 0x100 + AT24CXX_ReadOneByte(E000Interval_END_ADDR);
	return E000Interval;
}

/*
@brief     : 清除E000Interval时间
			 Clean E000Interval.
@para      :
@return    : 清除成功true，失败false
*/
bool ModbusController_InitState::Clean_E000Interval(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(E000Interval_BASE_ADDR, 0x00);
	AT24CXX_WriteOneByte(E000Interval_END_ADDR, 0x00);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(E000Interval_BASE_ADDR) == 0x00) && (AT24CXX_ReadOneByte(E000Interval_END_ADDR) == 0x00))
	{
		Info_Println("清除E000Interval成功");
		return true;
	}
	Error_Println("清除E000Interval失败");
	return false;
}

/*
 @brief     : 存储StopInterval时间
			  Save StopInterval.
 @para      :
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_StopInterval(unsigned char High, unsigned char Low)
{
	if ((AT24CXX_ReadOneByte(StopInterval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(StopInterval_END_ADDR) == Low))
	{
		Info_Println("储存StopInterval成功");
		return true;
	}

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(StopInterval_BASE_ADDR, High);
	AT24CXX_WriteOneByte(StopInterval_END_ADDR, Low);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(StopInterval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(StopInterval_END_ADDR) == Low))
	{
		Info_Println("储存StopInterval成功");
		return true;
	}
	Error_Println("储存StopInterval失败");
	return false;
}

/*
 @brief     : 读取StopInterval时间
			  Read StopInterval.
 @para      :
 @return    : StopInterval
 */
unsigned int ModbusController_InitState::Read_StopInterval(void)
{
	unsigned int StopInterval = AT24CXX_ReadOneByte(StopInterval_BASE_ADDR) * 0x100 + AT24CXX_ReadOneByte(StopInterval_END_ADDR);
	return StopInterval;
}

/*
@brief     : 清除StopInterval时间
			 Clean StopInterval.
@para      :
@return    : 清除成功true，失败false
*/
bool ModbusController_InitState::Clean_StopInterval(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(StopInterval_BASE_ADDR, 0x00);
	AT24CXX_WriteOneByte(StopInterval_END_ADDR, 0x00);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(StopInterval_BASE_ADDR) == 0x00) && (AT24CXX_ReadOneByte(StopInterval_END_ADDR) == 0x00))
	{
		Info_Println("清除StopInterval成功");
		return true;
	}
	Error_Println("清除StopInterval失败");
	return false;
}

//------------------------------------------
/*
 @brief     : 存储WorkInterval时间
			  Save WorkInterval.
 @para      :
 @return    : 保存成功true，失败false
 */
bool ModbusController_InitState::Save_WorkInterval(unsigned char High, unsigned char Low)
{
	if ((AT24CXX_ReadOneByte(WorkInterval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(WorkInterval_END_ADDR) == Low))
	{
		Info_Println("储存WorkInterval成功");
		return true;
	}

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(WorkInterval_BASE_ADDR, High);
	AT24CXX_WriteOneByte(WorkInterval_END_ADDR, Low);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(WorkInterval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(WorkInterval_END_ADDR) == Low))
	{
		Info_Println("储存WorkInterval成功");
		return true;
	}
	Error_Println("储存WorkInterval失败");
	return false;
}

/*
 @brief     : 读取WorkInterval时间
			  Read WorkInterval.
 @para      :
 @return    : WorkInterval
 */
unsigned int ModbusController_InitState::Read_WorkInterval(void)
{
	unsigned int WorkInterval = AT24CXX_ReadOneByte(WorkInterval_BASE_ADDR) * 0x100 + AT24CXX_ReadOneByte(WorkInterval_END_ADDR);
	return WorkInterval;
}

/*
@brief     : 清除WorkInterval时间
			 Clean WorkInterval.
@para      :
@return    : 清除成功true，失败false
*/
bool ModbusController_InitState::Clean_WorkInterval(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(WorkInterval_BASE_ADDR, 0x00);
	AT24CXX_WriteOneByte(WorkInterval_END_ADDR, 0x00);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(WorkInterval_BASE_ADDR) == 0x00) && (AT24CXX_ReadOneByte(WorkInterval_END_ADDR) == 0x00))
	{
		Info_Println("清除WorkInterval成功");
		return true;
	}
	Error_Println("清除WorkInterval失败");
	return false;
}

/*
@brief     : 存储CyclicInterval时间
			 Save CyclicInterval.
@para      :
@return    : 保存成功true，失败false
*/
bool ModbusController_InitState::Save_CyclicInterval(unsigned char High, unsigned char Low)
{
	if ((AT24CXX_ReadOneByte(Cyclic_interval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(Cyclic_interval_END_ADDR) == Low))
	{
		Info_Println("储存CyclicInterval成功");
		return true;
	}

	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(Cyclic_interval_BASE_ADDR, High);
	AT24CXX_WriteOneByte(Cyclic_interval_END_ADDR, Low);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(Cyclic_interval_BASE_ADDR) == High) && (AT24CXX_ReadOneByte(Cyclic_interval_END_ADDR) == Low))
	{
		Info_Println("储存CyclicInterval成功");
		return true;
	}
	Error_Println("储存CyclicInterval失败");
	return false;
}

/*
@brief     : 读取CyclicInterval时间
			 Read CyclicInterval.
@para      :
@return    : CyclicInterval
*/
unsigned int ModbusController_InitState::Read_CyclicInterval()
{
	unsigned int CyclicInterval = AT24CXX_ReadOneByte(Cyclic_interval_BASE_ADDR) * 0x100 + AT24CXX_ReadOneByte(Cyclic_interval_END_ADDR);
	return CyclicInterval;
}

/*
@brief     : 清除CyclicInterval时间
			 Clean CyclicInterval.
@para      :
@return    : 清除成功true，失败false
*/
bool ModbusController_InitState::Clean_CyclicInterval(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(Cyclic_interval_BASE_ADDR, 0x00);
	AT24CXX_WriteOneByte(Cyclic_interval_END_ADDR, 0x00);
	EEPROM_Write_Disable();

	if ((AT24CXX_ReadOneByte(Cyclic_interval_BASE_ADDR) == 0x00) && (AT24CXX_ReadOneByte(Cyclic_interval_END_ADDR) == 0x00))
	{
		Info_Println("清除CyclicInterval成功");
		return true;
	}
	Error_Println("清除CyclicInterval失败");
	return false;
}


/*
@brief     : 存储AI关联路数
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Save_AI_Relation_Way(unsigned char* data)
{
	EEPROM_Write_Enable();
	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		AT24CXX_WriteOneByte(AI_Relation_Way_BASE_ADDR+i, data[i]);
		// Debug_Serial.print(AT24CXX_ReadOneByte(AI_Relation_Way_BASE_ADDR+i));
		// Debug_Serial.print(" ");
	}	
	EEPROM_Write_Disable();

	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		if (AT24CXX_ReadOneByte(AI_Relation_Way_BASE_ADDR+i) == data[i])
		{
			
		}
		else
		{
			Error_Println("储存AI_Relation_Way失败!!!");
			return false;
		}
	}

	Info_Println("储存AI_Relation_Way成功");
	return true;
}

/*
@brief     : 读取AI关联路数
@para      :
@return    : 
*/
unsigned char Positive_Negative_MODE::Read_AI_Relation_Way(unsigned char which_Way)
{
	if (which_Way > 5) return 0;

	unsigned char data = 0;

	data = AT24CXX_ReadOneByte(AI_Relation_Way_BASE_ADDR+which_Way);
	return data;
}

/*
@brief     : 清除AI关联路数
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Clean_AI_Relation_Way(void)
{
	EEPROM_Write_Enable();
	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		AT24CXX_WriteOneByte(AI_Relation_Way_BASE_ADDR + i, 0x00);
	}	
	EEPROM_Write_Disable();

	for (unsigned char i = 0; i < 8; i++)
	{
		if (AT24CXX_ReadOneByte(AI_Relation_Way_BASE_ADDR+i) == 0x00)
		{
			
		}
		else
		{
			Error_Println("清除AI_Relation_Way失败!!!");
			return false;
		}
	}

	Info_Println("清除AI_Relation_Way成功");
	return true;
}


/*
@brief     : 保存是否反向
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Save_WayIS_Reverse(unsigned char* data)
{
	EEPROM_Write_Enable();
	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		AT24CXX_WriteOneByte(WayIS_Reverse + i, data[i]);
	}
	EEPROM_Write_Disable();

	for (unsigned char i = 0; i < 8; i++)
	{
		if (AT24CXX_ReadOneByte(WayIS_Reverse + i) == data[i])
		{
			
		}
		else
		{
			Error_Println("保存是否反向失败!!! <Save_WayIS_Reverse>");
			return false;
		}
	}

	Info_Println("保存是否反向成功 <Save_WayIS_Reverse>");
	return true;
}

/*
@brief     : 读取是否反向
@para      :
@return    : 成功true，失败false
*/
unsigned char Positive_Negative_MODE::Read_WayIS_Reverse(unsigned char which_Way)
{
	if (which_Way > 5) return 0;

	unsigned char data = 0;

	data = AT24CXX_ReadOneByte(WayIS_Reverse + which_Way);

	return data;
}

/*
@brief     : 清除是否反向
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Clean_WayIS_Reverse(void)
{
	EEPROM_Write_Enable();
	#if B400
	for (size_t i = 0; i < 4; i++)
	#else
	for (size_t i = 0; i < 8; i++)
	#endif
	{
		AT24CXX_WriteOneByte(WayIS_Reverse + i, 0x00);
	}
	EEPROM_Write_Disable();

	for (unsigned char i = 0; i < 8; i++)
	{
		if (AT24CXX_ReadOneByte(WayIS_Reverse + i) == 0x00)
		{
			
		}
		else
		{
			Error_Println("清除是否反向失败!!!");
			return false;
		}
	}

	Info_Println("清除是否反向成功");
	return true;
}


/*
@brief     : 保存AI关联以及阈值倍数被设置
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Save_A009_Seted(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(A009_Seted, 0x55);	
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(A009_Seted) == 0x55)
	{
		Info_Println("保存AI关联以及反向标志位成功");
		return true;
	}
	else
	{
		Error_Println("保存AI关联以及反向标志位失败!!!");
		return false;
	}
}
/*
@brief     : 读取AI关联以及阈值倍数被设置
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Read_A009_Seted(void)
{
	if (AT24CXX_ReadOneByte(A009_Seted) == 0x55)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*
@brief     : 清除AI关联以及阈值倍数被设置
@para      :
@return    : 成功true，失败false
*/
bool Positive_Negative_MODE::Clean_A009_Seted(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(A009_Seted, 0x00);	
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(A009_Seted) == 0x00)
	{
		Info_Println("清除AI关联以及反向标志位成功");
		return true;
	}
	else
	{
		Error_Println("清除AI关联以及反向标志位失败!!!");
		return false;
	}
}

//保存电流倍数
bool Positive_Negative_MODE::Save_EleCurrent_Times(unsigned char data)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(EleCurrent_Times, data);	
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(EleCurrent_Times) == data)
	{
		Info_Println("保存电流倍数成功");
		return true;
	}
	else
	{
		Error_Println("保存电流倍数失败!!!");
		return false;
	}
}
//读取电流倍数
unsigned char Positive_Negative_MODE::Read_EleCurrent_Times(void)
{
	unsigned char data = 0;
	data = AT24CXX_ReadOneByte(EleCurrent_Times);
	return data;
}
//清除电流倍数
bool Positive_Negative_MODE::Clean_EleCurrent_Times(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(EleCurrent_Times, 0x00);	
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(EleCurrent_Times) == 0x00)
	{
		Info_Println("清除电流倍数成功");
		return true;
	}
	else
	{
		Error_Println("清除电流倍数失败!!!");
		return false;
	}
}


//保存A00D标志位
bool Positive_Negative_MODE::Save_A00D_Seted(bool Flag)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(A00D_Seted, Flag);	
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(A00D_Seted) == Flag)
	{
		Info_Println("保存A00D标志位成功");
		return true;
	}
	else
	{
		Error_Println("保存A00D标志位失败!!!");
		return false;
	}
}
//读取A00D的状态
bool Positive_Negative_MODE::Read_A00D_Seted(void)
{
	bool Flag = false;
	Flag = AT24CXX_ReadOneByte(A00D_Seted);
	return Flag;
}
//清除A00D被设置标志位
bool Positive_Negative_MODE::Clean_A00D_Seted(void)
{
	EEPROM_Write_Enable();
	AT24CXX_WriteOneByte(A00D_Seted, false);	
	EEPROM_Write_Disable();

	if (AT24CXX_ReadOneByte(A00D_Seted) == false)
	{
		Info_Println("清除A00D标志位成功");
		return true;
	}
	else
	{
		Error_Println("清除A00D标志位失败!!!");
		return false;
	}
}




/* 连续顺序写/读一串数据缓存 */
film_mem_err Film_MEM_Write_Buffer(film_u32 base_addr, film_u8 *buf, film_u32 len)
{
	unsigned char try_num = 0;

	iwdg_feed();
	EEPROM_Write_Enable();

	for (size_t i = 0; i < len; i++)
	{
		AT24C_02.AT24CXX_WriteOneByte(base_addr+i, buf[i]);

		while (AT24C_02.AT24CXX_ReadOneByte(base_addr+i) != buf[i])
		{
			iwdg_feed();
			try_num++;
			Error_Println("ReadOneByte[" + (base_addr+i) + "] = " + AT24C_02.AT24CXX_ReadOneByte(base_addr+i));
			Error_Println("buf[" + i + "]= " + buf[i] + " , try_num = " + try_num);

			if (try_num > 10) goto write_err;
			
			AT24C_02.AT24CXX_WriteOneByte(base_addr+i, buf[i]);	
		}
		try_num = 0;
	}
	EEPROM_Write_Disable();
	Debug_Println("OKOKOK~~~");
	return FILM_MEM_OK;

	write_err:
		EEPROM_Write_Disable();
		Error_Println("ERRR");
		return FILM_MEM_W_ERR;
}


film_mem_err Film_MEM_Read_Buffer(film_u32 base_addr, film_u8 *buf, film_u32 len)
{
	Debug_Serial.print(">>>>>");
	for (unsigned int i = 0; i < len; i++)
	{
		buf[i] = AT24C_02.AT24CXX_ReadOneByte(base_addr+i);
		Debug_Serial.print(buf[i],HEX);
		Debug_Serial.print(" ");
	}
	Debug_Serial.println("<<<<<");
	return FILM_MEM_OK;
}



/* 远程升级 */
// bt_u8 BT_MEM_Write_Bytes(bt_u8 base_addr, bt_u8 *buf, bt_u8 len)
// {
//     uint8_t i;

//     EEPROM_Write_Enable();
//     for (i = 0; i < len; i++)
//         AT24C_02.AT24CXX_WriteOneByte(base_addr + i, buf[i]);
//     EEPROM_Write_Disable();

//     return 1;
// }

// bt_u8 BT_MEM_Read_Bytes(bt_u8 base_addr, bt_u8 *buf, bt_u8 len)
// {
//     uint8_t i;

//     for (i = 0; i < len; i++)
//         buf[i] = AT24C_02.AT24CXX_ReadOneByte(base_addr + i);

//     return 1;
// }

// void BT_Fill_ReservedByte(bt_u8 *re_buf) 
// {
//     SN.Read_SN_Code(&re_buf[0]);
//     re_buf[9] = SN.Read_Area_Number();
//     re_buf[10] = 0;
// }
