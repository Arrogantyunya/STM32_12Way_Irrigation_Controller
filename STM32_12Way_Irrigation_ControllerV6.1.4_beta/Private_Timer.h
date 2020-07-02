#ifndef _PRIVATE_TIMER_H
#define _PRIVATE_TIMER_H

// #include "boot_config.h"
// #include "boot_mem.h"
// #include "boot_protocol.h"

void Realtime_Status_Reporting_Timer_Init(void);
void Timer3_Init(void);
void Irrigation_Timer_Init(void);

void Start_Status_Report_Timing(void);
void Start_Timer3(void);
void Start_Timer4(void);

void Stop_Status_Report_Timing(void);
void Stop_Timer4(void);

void Timer2_Interrupt(void);
void Timer3_Interrupt(void);
void Timer4_Interrupt(void);

extern unsigned int Time_Count;


#endif