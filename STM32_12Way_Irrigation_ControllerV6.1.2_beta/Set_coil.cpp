#include "Set_coil.h"
#include <libmaple/iwdg.h>
#include "Modbus.h"
#include "ModbusSerial.h"
#include "fun_periph.h"
 #include "Memory.h"
#include "Command_Analysis.h"
// #include "receipt.h"
// #include "Private_Timer.h"
// #include "fun_periph.h"
// #include "LoRa.h"
#include<math.h>
#include <Arduino.h>


Modbus_Coils Modbus_Coil;
ModbusSerial mb;

bool gTime_arrive_Flag = false;//定时器时间到达的标志位
unsigned int Irrigation_time = 0x00;//灌溉时间（也就是定时器需要定时的时间，可以是单个间隔时间，单个开启时间，循环间隔时间）

bool Unkonwn_Open_Flag[8] = {false};
unsigned char Unkonwn_Open_RunOpen[8] = {0};

/*
 @brief   : 进行modbus库的配置
 @para    : None
 @return  : None
 */
void Modbus_Coils::Modbus_Config(void)
{
	// Config Modbus Serial(port, speed, byte format)
	mb.config(&Serial1, 9600, SERIAL_8N1);
	// Set the Slave ID (1-247)
	mb.setSlaveId(1);

	// Add register
	mb.addCoil(KCZJ1_COIL);
	mb.addCoil(KCZJ2_COIL);
	mb.addCoil(KCZJ3_COIL);
	mb.addCoil(KCZJ4_COIL);
	mb.addCoil(KCZJ5_COIL);
	mb.addCoil(KCZJ6_COIL);
	mb.addCoil(KCZJ7_COIL);
	mb.addCoil(KCZJ8_COIL);
	mb.addCoil(KCZJ9_COIL);
	mb.addCoil(KCZJ10_COIL);
	mb.addCoil(KCZJ11_COIL);
	mb.addCoil(KCZJ12_COIL);

	mb.addCoil(KCZJ13_COIL);
	mb.addCoil(KCZJ14_COIL);
	mb.addCoil(KCZJ15_COIL);
	mb.addCoil(KCZJ16_COIL);
	mb.addCoil(KCZJ17_COIL);
	mb.addCoil(KCZJ18_COIL);
	mb.addCoil(KCZJ19_COIL);
	mb.addCoil(KCZJ20_COIL);
	mb.addCoil(KCZJ21_COIL);
	mb.addCoil(KCZJ22_COIL);
	mb.addCoil(KCZJ23_COIL);
	mb.addCoil(KCZJ24_COIL);

	mb.addCoil(KCZJ25_COIL);
	mb.addCoil(KCZJ26_COIL);
	mb.addCoil(KCZJ27_COIL);
	mb.addCoil(KCZJ28_COIL);
	mb.addCoil(KCZJ29_COIL);
	mb.addCoil(KCZJ30_COIL);
	mb.addCoil(KCZJ31_COIL);
	mb.addCoil(KCZJ32_COIL);
	mb.addCoil(KCZJ33_COIL);
	mb.addCoil(KCZJ34_COIL);
	mb.addCoil(KCZJ35_COIL);
	mb.addCoil(KCZJ36_COIL);

	mb.addCoil(KCZJ37_COIL);
	mb.addCoil(KCZJ38_COIL);
	mb.addCoil(KCZJ39_COIL);
	mb.addCoil(KCZJ40_COIL);
	mb.addCoil(KCZJ41_COIL);
	mb.addCoil(KCZJ42_COIL);
	mb.addCoil(KCZJ43_COIL);
	mb.addCoil(KCZJ44_COIL);
	mb.addCoil(KCZJ45_COIL);
	mb.addCoil(KCZJ46_COIL);
	mb.addCoil(KCZJ47_COIL);
	mb.addCoil(KCZJ48_COIL);

	mb.addCoil(KCZJ49_COIL);
	mb.addCoil(KCZJ50_COIL);
	mb.addCoil(KCZJ51_COIL);
	mb.addCoil(KCZJ52_COIL);
	mb.addCoil(KCZJ53_COIL);
	mb.addCoil(KCZJ54_COIL);
	mb.addCoil(KCZJ55_COIL);
	mb.addCoil(KCZJ56_COIL);
	mb.addCoil(KCZJ57_COIL);
	mb.addCoil(KCZJ58_COIL);
	mb.addCoil(KCZJ59_COIL);
	mb.addCoil(KCZJ60_COIL);

	mb.addCoil(KCZJ61_COIL);
	mb.addCoil(KCZJ62_COIL);
	mb.addCoil(KCZJ63_COIL);
	mb.addCoil(KCZJ64_COIL);

	mb.addIsts(DI1_ISTS);
	mb.addIsts(DI2_ISTS);
	mb.addIsts(DI3_ISTS);
	mb.addIsts(DI4_ISTS);
	mb.addIsts(DI5_ISTS);
	mb.addIsts(DI6_ISTS);
	mb.addIsts(DI7_ISTS);
	mb.addIsts(DI8_ISTS);

	mb.addIreg(AI1_IREG);
	mb.addIreg(AI2_IREG);
	mb.addIreg(AI3_IREG);
	mb.addIreg(AI4_IREG);
	mb.addIreg(AI5_IREG);
	mb.addIreg(AI6_IREG);
	mb.addIreg(AI7_IREG);
	mb.addIreg(AI8_IREG);

	mb.addHreg(Variable_1, 127);
}

/*
 @brief   : 设置输出线圈默认值
 @param   : void
 @return  : 设置成功返回true，失败返回false
 */
bool Modbus_Coils::Set_Coil_DefaultValue(void)
{
	unsigned char count = 0;
	if(mb.Coil(KCZJ1_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ2_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ3_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ4_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ5_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ6_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ7_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ8_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ9_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ10_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ11_COIL,0x0000) == true)	count++;
	if(mb.Coil(KCZJ12_COIL,0x0000) == true)	count++;

    

	if (count == 12)
	{
		// unsigned char *_empty = NULL;
		Modbus_Coil.Modbus_Realization();//设置输出线圈状态，modbus实现
		Info_Println("Set Coil DefaultValue Success.");
		return true;
	}
	else
	{
		Error_Println("Set Coil DefaultValue Failure!");
		return false;
	}
}

/*
 @brief   : 设置输出线圈初始值
 @param   : void
 @return  : 设置成功返回true，失败返回false
 */
bool Modbus_Coils::Set_Coil_InitValue(void)
{
	unsigned char DO_Init[8] = { 0x00 };
	unsigned char *data = InitState.Read_DO_InitState();//读取DO的初始化状态
	//Serial.println("...........");
	for (size_t i = 0; i < 8; i++)
	{
		//Serial.println(*(data+i));
		DO_Init[i] = *(data + i);
		//Serial.println(DO_Init[i],HEX);
	}
#if PLC_V1	//只有12个DO输出

	if (Init_DO_1to8(DO_Init[0]) && Init_DO_9to16(DO_Init[1]))
	{
		// unsigned char *_empty = NULL;
		Modbus_Coil.Modbus_Realization();//设置输出线圈状态，modbus实现
		Info_Println("PLC_V1 DO外设引脚初始化成功");
		return true;
	}
	
	return false;
#elif PLC_V2

#endif // PLC_Vx	
}

/*
 @brief   : 设置输出线圈状态，modbus实现
 @param   : 1.一个数组指针(如果不需要使用数组，传入一个指向NULL的指针)
			2.数组的长度(如果传入的是指向NULL的指针，该参数填0)
 @return  : void
 */
void Modbus_Coils::Modbus_Realization(unsigned char* modbusPacket, int Length)
{
	mb.task(modbusPacket,Length);

    digitalWrite(KCZJ1, !mb.Coil(KCZJ1_COIL));
    digitalWrite(KCZJ2, !mb.Coil(KCZJ2_COIL));
    digitalWrite(KCZJ3, !mb.Coil(KCZJ3_COIL));
    digitalWrite(KCZJ4, !mb.Coil(KCZJ4_COIL));
    digitalWrite(KCZJ5, !mb.Coil(KCZJ5_COIL));
    digitalWrite(KCZJ6, !mb.Coil(KCZJ6_COIL));
    digitalWrite(KCZJ7, !mb.Coil(KCZJ7_COIL));
    digitalWrite(KCZJ8, !mb.Coil(KCZJ8_COIL));
    digitalWrite(KCZJ9, !mb.Coil(KCZJ9_COIL));
    digitalWrite(KCZJ10, !mb.Coil(KCZJ10_COIL));
    digitalWrite(KCZJ11, !mb.Coil(KCZJ11_COIL));
    digitalWrite(KCZJ12, !mb.Coil(KCZJ12_COIL));
	
	/*Serial.println("----------------------");
	delay(1000);
	Serial.println(String("mb.Coil(KCZJ1_COIL) = ") + mb.Coil(KCZJ1_COIL));
	Serial.println(String("mb.Coil(KCZJ2_COIL) = ") + mb.Coil(KCZJ2_COIL));
	Serial.println(String("mb.Coil(KCZJ3_COIL) = ") + mb.Coil(KCZJ3_COIL));
	Serial.println(String("mb.Coil(KCZJ4_COIL) = ") + mb.Coil(KCZJ4_COIL));
	Serial.println(String("mb.Coil(KCZJ5_COIL) = ") + mb.Coil(KCZJ5_COIL));
	Serial.println(String("mb.Coil(KCZJ6_COIL) = ") + mb.Coil(KCZJ6_COIL));
	Serial.println(String("mb.Coil(KCZJ7_COIL) = ") + mb.Coil(KCZJ7_COIL));
	Serial.println(String("mb.Coil(KCZJ8_COIL) = ") + mb.Coil(KCZJ8_COIL));
	Serial.println(String("mb.Coil(KCZJ9_COIL) = ") + mb.Coil(KCZJ9_COIL));
	Serial.println(String("mb.Coil(KCZJ10_COIL) = ") + mb.Coil(KCZJ10_COIL));
	Serial.println(String("mb.Coil(KCZJ11_COIL) = ") + mb.Coil(KCZJ11_COIL));
	Serial.println(String("mb.Coil(KCZJ12_COIL) = ") + mb.Coil(KCZJ12_COIL));
	Serial.println(String("mb.Coil(KCZJ13_COIL) = ") + mb.Coil(KCZJ13_COIL));
	Serial.println(String("mb.Coil(KCZJ14_COIL) = ") + mb.Coil(KCZJ14_COIL));
	Serial.println(String("mb.Coil(KCZJ15_COIL) = ") + mb.Coil(KCZJ15_COIL));
	Serial.println(String("mb.Coil(KCZJ16_COIL) = ") + mb.Coil(KCZJ16_COIL));*/

    mb.Ists(DI1_ISTS, digitalRead(DI1));
    mb.Ists(DI2_ISTS, digitalRead(DI2));
    mb.Ists(DI3_ISTS, digitalRead(DI3));
    mb.Ists(DI4_ISTS, digitalRead(DI4));
    mb.Ists(DI5_ISTS, digitalRead(DI5));
    mb.Ists(DI6_ISTS, digitalRead(DI6));
    mb.Ists(DI7_ISTS, digitalRead(DI7));
    mb.Ists(DI8_ISTS, digitalRead(DI8));

    mb.Ireg(AI1_IREG, analogRead(AI1));
    mb.Ireg(AI2_IREG, analogRead(AI2));
    mb.Ireg(AI3_IREG, analogRead(AI3));
    mb.Ireg(AI4_IREG, analogRead(AI4));
    mb.Ireg(AI5_IREG, analogRead(AI5));
    mb.Ireg(AI6_IREG, analogRead(AI6));
    mb.Ireg(AI7_IREG, analogRead(AI7));
    mb.Ireg(AI8_IREG, analogRead(AI8));
}


/*
 @brief   : DO1-DO8的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_1to8(unsigned char num)
{
	Info_Println("开始设置线圈值<Init_DO_1to8>");
	unsigned char n1 = 0;unsigned char n2 = 1; unsigned char count = 0;
	String DO_Init_1 = String(num, BIN);/* String DO_Init_2 = String(DO_Init[1], BIN);*/
	Debug_Println(String("DO_Init_1 = ") + DO_Init_1); /*Serial.println(String("DO_Init_2 = ") + DO_Init_2);*/
	if (DO_Init_1.length() == 8)
	{
		if (mb.Coil(KCZJ1_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ2_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ3_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ4_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ5_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ6_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 7)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ3_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ4_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ5_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ6_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 6)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ3_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ4_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ5_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ6_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 5)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ3_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ4_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ5_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ6_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 4)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ3_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ4_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ5_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ6_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 3)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ3_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ4_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ5_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ6_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 2)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ3_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ4_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ5_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ6_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ7_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_1.length() == 1)
	{
		if (mb.Coil(KCZJ1_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ2_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ3_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ4_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ5_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ6_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ7_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ8_COIL, (DO_Init_1.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_1.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else
	{
		return false;
	}


	if (count == 8)
	{
		Info_Println("设置线圈值成功");
		return true;
	}
	Error_Println("设置线圈值失败");
	return false;
}

/*
 @brief   : DO9-DO16的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_9to16(unsigned char num)
{
	Info_Println("开始设置线圈值<Init_DO_9to16>");
	unsigned char n1 = 0;unsigned char n2 = 1; unsigned char count = 0;
	String DO_Init_2 = String(num, BIN);
	Debug_Println(String("DO_Init_2 = ") + DO_Init_2);
	if (DO_Init_2.length() == 8)
	{
		if (mb.Coil(KCZJ9_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ10_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ11_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ12_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ13_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ14_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 7)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ11_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ12_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ13_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ14_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 6)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ11_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		if (mb.Coil(KCZJ12_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ13_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ14_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 5)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ11_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ12_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ13_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ14_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 4)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ11_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ12_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ13_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ14_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 3)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ11_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ12_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ13_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ14_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 2)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, 0xFF00) == true)	count++;
		if (mb.Coil(KCZJ11_COIL, 0xFF00) == true)	count++;
		if (mb.Coil(KCZJ12_COIL, 0xFF00) == true)	count++;
		//if (mb.Coil(KCZJ13_COIL, 0xFF00) == true)	count++;
		//if (mb.Coil(KCZJ14_COIL, 0xFF00) == true)	count++;
		//if (mb.Coil(KCZJ15_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else if (DO_Init_2.length() == 1)
	{
		if (mb.Coil(KCZJ9_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ10_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ11_COIL, 0x0000) == true)	count++;
		if (mb.Coil(KCZJ12_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ13_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ14_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ15_COIL, 0x0000) == true)	count++;
		//if (mb.Coil(KCZJ16_COIL, (DO_Init_2.substring(n1, n2) > String(0) ? 0xFF00 : 0x0000)) == true)	{count++;}n1++;n2++/*Serial.println((DO_Init_2.substring(n++, n) > String(0) ? 0xFF00 : 0x0000), HEX)*/;
	}
	else
	{
		return false;
	}


	if (count == 4)
	{
		Info_Println("设置线圈值成功<Init_DO_9to16>");
		return true;
	}
	Error_Println("设置线圈值失败<Init_DO_9to16>");
	return false;
}

/*
 @brief   : DO17-DO824的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_17to24(unsigned char num)
{
	num = num + 0;
	Info_Println();
	return true;
}

/*
 @brief   : DO25-DO32的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_25to32(unsigned char num)
{
	num = num + 0;
	Info_Println();
	return true;
}

/*
 @brief   : DO33-DO40的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_33to40(unsigned char num)
{
	num = num + 0;
	Info_Println();
	return true;
}

/*
 @brief   : DO41-DO48的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_41to48(unsigned char num)
{
	num = num + 0;
	Info_Println();
	return true;
}

/*
 @brief   : DO49-DO56的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_DO_49to56(unsigned char num)
{
	num = num + 0;
	Info_Println();
	return true;
}

/*
 @brief   : DO1-DO8的线圈初始化
 @param   : void
 @return  : void
 */
bool Modbus_Coils::Init_AO_1to8()
{
	Info_Println();
	return true;
}

/*
 @brief   : 得到DI1-8的值
 @param   : void
 @return  : DI1to8
 */
unsigned char Modbus_Coils::Get_DI_1to8(void)
{
	unsigned char DI1to8 = 0x00;
	for (size_t i = 0; i < 8; i++)
	{
		switch (i)
		{
		case 0: digitalRead(DI8) == HIGH ? DI1to8 += 128 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 1:	digitalRead(DI7) == HIGH ? DI1to8 += 64 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 2: digitalRead(DI6) == HIGH ? DI1to8 += 32 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 3:	digitalRead(DI5) == HIGH ? DI1to8 += 16 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 4: digitalRead(DI4) == HIGH ? DI1to8 += 8 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 5:	digitalRead(DI3) == HIGH ? DI1to8 += 4 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 6: digitalRead(DI2) == HIGH ? DI1to8 += 2 : DI1to8 += 0;
			/*Serial.println(String("DI1to8 = ") + DI1to8);*/ break;
		case 7:	digitalRead(DI1) == HIGH ? DI1to8 += 1 : DI1to8 += 0;
			// Serial.println(String("DI1to8 = ") + DI1to8); break;
		default:
			break;
		}
	}
	
	return DI1to8;
}

/*
 @brief   : 得到DO1-8的值
 @param   : void
 @return  : DO1to8
 */
unsigned char Modbus_Coils::Get_DO_1to8(void)
{
	unsigned char DO1to8 = 0x00;
	for (size_t i = 0; i < 8; i++)
	{
		switch (i)
		{
		case 0: digitalRead(KCZJ8) == HIGH ? DO1to8 &= ~(1<<7) : DO1to8 |= 1<<7;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 1:	digitalRead(KCZJ7) == HIGH ? DO1to8 &= ~(1<<6) : DO1to8 |= 1<<6;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 2: digitalRead(KCZJ6) == HIGH ? DO1to8 &= ~(1<<5) : DO1to8 |= 1<<5;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 3:	digitalRead(KCZJ5) == HIGH ? DO1to8 &= ~(1<<4) : DO1to8 |= 1<<4;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 4: digitalRead(KCZJ4) == HIGH ? DO1to8 &= ~(1<<3) : DO1to8 |= 1<<3;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 5:	digitalRead(KCZJ3) == HIGH ? DO1to8 &= ~(1<<2) : DO1to8 |= 1<<2;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 6: digitalRead(KCZJ2) == HIGH ? DO1to8 &= ~(1<<1) : DO1to8 |= 1<<1;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		case 7:	digitalRead(KCZJ1) == HIGH ? DO1to8 &= ~(1<<0) : DO1to8 |= 1<<0;
			/* Serial.println(String("DO1to8 = ") + DO1to8); */ break;
		default:
			break;
		}
	}

	return DO1to8;
}

/*
 @brief   : 得到DO9-16的值
 @param   : void
 @return  : DO9to16
 */
unsigned char Modbus_Coils::Get_DO_9to16(void)
{
	unsigned char DO9to16 = 0x00;
#if PLC_V1
	for (size_t i = 0; i < 8; i++)
	{
		switch (i)
		{
		case 0: DO9to16 += 0;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 1:	DO9to16 += 0;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 2: DO9to16 += 0;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 3:	DO9to16 += 0;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 4: digitalRead(KCZJ12) == HIGH ? DO9to16 &= ~(1<<3) : DO9to16 |= 1<<3;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 5:	digitalRead(KCZJ11) == HIGH ? DO9to16 &= ~(1<<2) : DO9to16 |= 1<<2;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 6: digitalRead(KCZJ10) == HIGH ? DO9to16 &= ~(1<<1) : DO9to16 |= 1<<1;
			/*Serial.println(String("DO9to16 = ") + DO9to16);*/ break;
		case 7:	digitalRead(KCZJ9) == HIGH ? DO9to16 &= ~(1<<0) : DO9to16 |= 1<<0;
			/* Serial.println(String("DO9to16 = ") + DO9to16); */ break;
		default: break;
		}
	}
#elif PLC_V2
	switch (i)
	{
	case 0: digitalRead(KCZJ9) == HIGH ? DI1to8 += 0 : DI1to8 += 128;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 1:	digitalRead(KCZJ10) == HIGH ? DI1to8 += 0 : DI1to8 += 64;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 2: digitalRead(KCZJ11) == HIGH ? DI1to8 += 0 : DI1to8 += 32;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 3:	digitalRead(KCZJ12) == HIGH ? DI1to8 += 0 : DI1to8 += 16;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 4: digitalRead(KCZJ13) == HIGH ? DI1to8 += 0 : DI1to8 += 8;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 5:	digitalRead(KCZJ14) == HIGH ? DI1to8 += 0 : DI1to8 += 4;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 6: digitalRead(KCZJ15) == HIGH ? DI1to8 += 0 : DI1to8 += 2;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	case 7:	digitalRead(KCZJ16) == HIGH ? DI1to8 += 0 : DI1to8 += 1;
		Serial.println(String("DO1to8 = ") + DI1to8); break;
	default:
		break;
	}
#endif

	return DO9to16;
}

/*
 @brief   : 得到AI1-8的值
 @param   : void
 @return  :
 */
unsigned char * Modbus_Coils::Get_AI_1to8(void)
{
#if PLC_V1
	static unsigned char AI_1to8[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned long x[8] = {0x00};
	u_int AI[8] = {0x00};
	iwdg_feed();
	
	for (size_t i_1 = 0; i_1 <8; i_1++)
	{
		for (size_t i = 0; i < 10; i++)
		{
			switch (i_1)
			{
			case 0:	x[0] += analogRead(AI1);delay(10);break;
			case 1:	x[1] += analogRead(AI2);delay(10);break;
			case 2:	x[2] += analogRead(AI3);delay(10);break;
			case 3:	x[3] += analogRead(AI4);delay(10);break;
			case 4:	x[4] += analogRead(AI5);delay(10);break;
			case 5:	x[5] += analogRead(AI6);delay(10);break;
			case 6:	x[6] += analogRead(AI7);delay(10);break;
			case 7:	x[7] += analogRead(AI8);delay(10);break;
			default:break;
			}
		}	
	}
	
	AI[0] = x[0]/10;
	AI[1] = x[1]/10;
	AI[2] = x[2]/10;
	AI[3] = x[3]/10;
	AI[4] = x[4]/10;
	AI[5] = x[5]/10;
	AI[6] = x[6]/10;
	AI[7] = x[7]/10;

	// x[0] = 488;//0000 0001 1110 1000
	// AI_1to8[0] = x[0] >> 8;
	// Serial.println(String("AI_1to8[0] = ") + String(AI_1to8[0], BIN));
	// AI_1to8[1] = x[0];
	// Serial.println(String("AI_1to8[1] = ") + String(AI_1to8[1], BIN));

	for (size_t i = 0; i < 8; i++)
	{
		AI_1to8[2*i] = AI[i] >> 8;
		AI_1to8[(2*i)+1] = AI[i];
	}
	return AI_1to8;
#elif PLC_V2

#endif // PLC_V1
}

/*
 @brief   : 得到某路的AI()
 @param   : void
 @return  :
 */
unsigned short Modbus_Coils::Get_which_AI(unsigned char which_Way)
{
	unsigned long data = 0;

	#if PLC_V1
		switch (which_Way-1)
		{
			case 0:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI1);delay(1);
				}
				data = data/10;	break;
			}
			case 1:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI2);delay(1);
				}
				data = data/10;	break;
			}
			case 2:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI3);delay(1);
				}
				data = data/10;	break;
			}
			case 3:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI4);delay(1);
				}
				data = data/10;	break;
			}
			case 4:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI5);delay(1);
				}
				data = data/10;	break;
			}
			case 5:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI6);delay(1);
				}
				data = data/10;	break;
			}
			case 6:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI7);delay(1);
				}
				data = data/10;	break;
			}
			case 7:
			{
				for (unsigned char i = 0; i < 10; i++){
					data += analogRead(AI8);delay(1);
				}
				data = data/10;	break;
			}
			default: break;
		}
		// Debug_Serial.println(String("第") + (which_Way) + "的电流为：" + data);
		return data;
	#elif PLC_V2

	#endif// PLC_V1
}

film_u32 Film_Read_Analog_Ele_Current_CH(film_u8 ch)//读取一路电机的电流值，单位是mA
{
	film_u32 data = Modbus_Coil.Get_which_AI(ch);
	return data*10;
	return data*Pos_Nega_mode.Read_EleCurrent_Times();
}

// Film_Forward,
// Film_Reversal,
// Film_Stop
void Film_Ctrl_Motor_CH(film_u8 ch, Film_DIR dir)//根据传入的路数和方向，控制该路电机转动
{
	Set_Way_Motor(ch, dir);
}


//设置电机的正反转与停止
void Set_Way_Motor(unsigned char ch, unsigned char status)
{
	if (status == Film_Forward)
	{
		if (!Pos_Nega_mode.Read_WayIS_Reverse((ch-1)))
		{
			Info_Println(String("第") + ch + "路为Film_Forward,目标正常");
			Set_DO_relay((2*(ch-1)),ON);
			Set_DO_relay(((2*(ch-1))+1),OFF);
		}
		else
		{
			Info_Println(String("第") + ch + "路为Film_Forward,目标反向");
			Set_DO_relay((2*(ch-1)),OFF);
			Set_DO_relay(((2*(ch-1))+1),ON);
		}
	}
	else if (status == Film_Reversal)
	{
		if (!Pos_Nega_mode.Read_WayIS_Reverse((ch-1)))
		{
			Info_Println(String("第") + ch + "路为Film_Reversal,目标正常");
			Set_DO_relay((2*(ch-1)),OFF);
			Set_DO_relay(((2*(ch-1))+1),ON);
		}
		else
		{
			Info_Println(String("第") + ch + "路为Film_Reversal,目标反向");
			Set_DO_relay((2*(ch-1)),ON);
			Set_DO_relay(((2*(ch-1))+1),OFF);
		}
	}
	else if (status == Film_Stop)
	{
		Set_DO_relay((2*(ch-1)),OFF);
		Set_DO_relay(((2*(ch-1))+1),OFF);
	}
	else
	{
		Error_Println("异常电机工作状态设置!!! <Set_Way_Motor>");
		Set_DO_relay((2*(ch-1)),OFF);
		Set_DO_relay(((2*(ch-1))+1),OFF);
	}

	unsigned long Now = millis();
	while (millis() - Now < A00B_Interval*1000)
	{
		LoRa_Command_Analysis.Receive_LoRa_Cmd();//从网关接收LoRa数据
	}
}

//设置继电器的开启
void Set_DO_relay(unsigned char way, bool value)
{
	switch (way)
	{
		case 0:digitalWrite(KCZJ1, value);value>0?mb.Coil(KCZJ1_COIL,0x0000):mb.Coil(KCZJ1_COIL,0xff00); break;
		case 1:digitalWrite(KCZJ2, value);value>0?mb.Coil(KCZJ2_COIL,0x0000):mb.Coil(KCZJ2_COIL,0xff00); break;
		case 2:digitalWrite(KCZJ3, value);value>0?mb.Coil(KCZJ3_COIL,0x0000):mb.Coil(KCZJ3_COIL,0xff00); break;
		case 3:digitalWrite(KCZJ4, value);value>0?mb.Coil(KCZJ4_COIL,0x0000):mb.Coil(KCZJ4_COIL,0xff00); break;
		case 4:digitalWrite(KCZJ5, value);value>0?mb.Coil(KCZJ5_COIL,0x0000):mb.Coil(KCZJ5_COIL,0xff00); break;
		case 5:digitalWrite(KCZJ6, value);value>0?mb.Coil(KCZJ6_COIL,0x0000):mb.Coil(KCZJ6_COIL,0xff00); break;
		case 6:digitalWrite(KCZJ7, value);value>0?mb.Coil(KCZJ7_COIL,0x0000):mb.Coil(KCZJ7_COIL,0xff00); break;
		case 7:digitalWrite(KCZJ8, value);value>0?mb.Coil(KCZJ8_COIL,0x0000):mb.Coil(KCZJ8_COIL,0xff00); break;
		case 8:digitalWrite(KCZJ9, value);value>0?mb.Coil(KCZJ9_COIL,0x0000):mb.Coil(KCZJ9_COIL,0xff00); break;
		case 9:digitalWrite(KCZJ10, value);value>0?mb.Coil(KCZJ10_COIL,0x0000):mb.Coil(KCZJ10_COIL,0xff00); break;
		case 10:digitalWrite(KCZJ11, value);value>0?mb.Coil(KCZJ11_COIL,0x0000):mb.Coil(KCZJ11_COIL,0xff00); break;
		case 11:digitalWrite(KCZJ12, value);value>0?mb.Coil(KCZJ12_COIL,0x0000):mb.Coil(KCZJ12_COIL,0xff00); break;
		default:Error_Println("异常!!强制停止");break;
	}
}

/* 异常处理卷膜开度未知 */
void ExceptionHandle_Film_M_Unkonwn_Open(void)
{
	// unsigned char Open_Way[8] = {0x00};//
	// unsigned char Open_F_Way[8] = {0x00};//
	// unsigned char ch_num = 0;//
	// unsigned char ch_F_num = 0;//
	// unsigned char open_buf[8] = {0x00};//
	// unsigned char open_F_buf[8] = {0x00};//

	// for (unsigned char  i = 0; i < MOTOR_CHANNEL; i++)
	// {
	// 	if (Film_Read_Motor_Status_CH(i+1) == Film_M_Unkonwn_Open)
	// 	{
	// 		Unkonwn_Open_Flag[i] = true;//
	// 		Unkonwn_Open_RunOpen[i] = Film_Read_Run_Opening_CH(i+1);//备份上一次开度为
	// 		Debug_Serial.println(String("第") + (i+1) + "路开度未知，目标开度为：" + Unkonwn_Open_RunOpen[i]);

	// 		Open_F_Way[ch_F_num] = i+1;
	// 		open_F_buf[ch_F_num] = 0x00;
	// 		ch_F_num++; 
	// 	}
	// 	else if (Film_Read_Motor_Status_CH(i+1) == Film_M_F_Open_OK && Unkonwn_Open_Flag[i])
	// 	{
	// 		Unkonwn_Open_Flag[i] = false;//
	// 		Debug_Serial.println(String("第") + (i+1) + "路开度复位，目标开度为：" + Unkonwn_Open_RunOpen[i]);
	// 		Open_Way[ch_num] = i+1;
	// 		open_buf[ch_num] = Unkonwn_Open_RunOpen[i];	
	// 		ch_num++;
	// 	}	
	// }

	// if (ch_F_num)
	// {
	// 	/* 这里是设置强开的线路 */
	// 	Film_Set_Open_Value(&Open_F_Way[0],ch_F_num,&open_F_buf[0]);
	// 	if (Film_New_Task_Handler(&Open_F_Way[0],ch_F_num,FILM_F_ROLL) == Film_OK)
	// 		Film_Motor_Run_Task(&Open_F_Way[0],ch_F_num,FILM_F_ROLL);
	// }
	// else if (ch_num)
	// {
	// 	Film_Set_Open_Value(&Open_Way[0],ch_num,&open_buf[0]);
	// 	if (Film_New_Task_Handler(&Open_Way[0],ch_num,FILM_ROLL) == Film_OK)
	// 		Film_Motor_Run_Task(&Open_Way[0],ch_num,FILM_ROLL);
	// }
}