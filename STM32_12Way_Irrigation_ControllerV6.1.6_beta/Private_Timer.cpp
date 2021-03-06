#include "Private_Timer.h"
// #include "fun_periph.h"
#include "Security.h"
#include "receipt.h"
#include "Memory.h"
#include "Set_coil.h"
#include "Command_Analysis.h"
#include <Arduino.h>

#include "boot_protocol.h"

/*Timer timing time*/
#define TIMER_NUM             1000000L * 1 //1s
#define CHECK_TIMER_NUM       1000000L

volatile static unsigned int gSelfCheckNum;
volatile static unsigned int gStateReportNum;
/*volatile static */unsigned int Time_Count;
volatile static unsigned int Time3_Count = 0;


/*
 @brief   : 使用定时器2初始化自动上报状态周期
 @param   : 无
 @return  : 无
 */
void Realtime_Status_Reporting_Timer_Init(void)
{
	Timer2.setPeriod(100000); // in microseconds，100ms
	Timer2.attachCompare1Interrupt(Timer2_Interrupt);
	Timer2.setCount(0);
}

/*
 @brief   : 使用定时器3初始化自检参数功能自检周期
 @param   : 无
 @return  : 无
 */
void Timer3_Init(void)
{
	Timer3.setPeriod(100000); // in microseconds，100mS
	Timer3.attachCompare1Interrupt(Timer3_Interrupt);
	Timer3.setCount(0);
}

/*
 @brief   : 使用定时器4初始化灌溉计时
 @param   : 无
 @return  : 无
 */
void Irrigation_Timer_Init(void)
{
	Timer4.setPeriod(TIMER_NUM); // in microseconds，1S
	Timer4.attachCompare1Interrupt(Timer4_Interrupt);
	Timer4.setCount(0);
}


/*
 @brief   : 开始上报状态周期计时
 @param   : 无
 @return  : 无
 */
void Start_Status_Report_Timing(void)
{
	Timer2.resume();
	Timer2.setCount(0);
}

/*
 @brief   : 开始自检周期计时
 @param   : 无
 @return  : 无
 */
void Start_Timer3(void)
{
	Timer3.resume();
	Timer3.setCount(0);
}

/*
 @brief   : 开始灌溉计时
 @param   : 无
 @return  : 无
 */
void Start_Timer4(void)
{
	Timer4.resume();
	Timer4.setCount(0);
	Time_Count = 0;
}

/*
 @brief   : 停止状态上报周期计时
 @param   : 无
 @return  : 无
 */
void Stop_Status_Report_Timing(void)
{
	Timer2.pause();
}

/*
 @brief   : 停止灌溉计时
 @param   : 无
 @return  : 无
 */
void Stop_Timer4(void)
{
	Timer4.pause();
	Time_Count = 0;
}

/*
 @brief   : 上报实时状态定时器2计时中断处理函数
 @param   : 无
 @return  : 无
 */
void Timer2_Interrupt(void)
{
	gStateReportNum++;
	/*大于0秒 && 小于1800秒（30分钟）才能开启自动上报*/
	if(Enter_Work_State)
	{
		if (gStateReportNum >= InitState.Read_WorkInterval()*10) //
		{
			gStateReportNum = 0;
			gStateReportFlag = true;
		}
	}
	else
	{
		if (gStateReportNum >= InitState.Read_StopInterval()*10) //
		{
			gStateReportNum = 0;
			gStateReportFlag = true;
		}
	}
	// Debug_Serial.println(String("BT_Query_CycVal = ") + BT_Query_CycVal);
	// Debug_Serial.println(String("BT_Rand_Seed_Num = ") + BT_Rand_Seed_Num);
	// BT_Query_CycVal++;
  	// BT_Rand_Seed_Num++;
}

/*
 @brief   : 自检计时定时器3计时中断处理函数
 @param   : 无
 @return  : 无
 */
void Timer3_Interrupt(void)
{
	// BT_Rcv_Tim_Num++;
	// BT_Query_CycVal++;
	// BT_Rand_Seed_Num++;

	Time3_Count++;
	if (Time3_Count < 10) return;
	Time3_Count = 0;
	Film_Load_Tim_Var();
}

/*
 @brief   : 灌溉定时器4计时中断处理函数
 @param   : 无
 @return  : 无
 */
void Timer4_Interrupt(void)
{
	gSelfCheckNum++;
	if (gSelfCheckNum >= 14400) //4 hours	14400
	{
		gSelfCheckNum = 0;
		gCheckStoreParamFlag = true;
	}
	
	Time_Count++;
	// Debug_Serial.println(String("Time_Count = ") + Time_Count);
}


