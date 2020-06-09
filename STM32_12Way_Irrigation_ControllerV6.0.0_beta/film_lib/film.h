#ifndef _FILM_H
#define _FILM_H

#include <stdio.h>

// #include "Motor.h"

#include "film_config.h"

#define USE_RESET_ROLL_CHECK                  1

#define FILM_DETECT_START_TIME                3U //启动限位检测时间

#define FILM_LOWER_CUR_VALUE                  150 //电流值下限，用来判断电机是否在运动，单位mA

#define FILM_DEFAULT_LOW_CUR_VALUE            300 //经验值，应用的电机的空转最小电流值
#define FILM_DEFAULT_CUR_VALUE                1000 //读取的电流值异常，默认的经验电流值，单位mA

#define FILM_ROLL_OVERTIME                    2000U  //卷膜允许最大时长(S)

#define FILM_CUR_COLLECT_FREQ                 4 //电流采集频率(S)


#define FILM_SET_FLAG                         0x55U
#define FILM_RESET_FLAG                       0x00U

#define FILM_ALL_OPEN                         100U
#define FILM_ALL_CLOSE                        0U

/* 该卷膜通用库，最大只能使用32路电机 */
#define FILM_CH_1        (1 << 0)
#define FILM_CH_2        (1 << 1)
#define FILM_CH_3        (1 << 2)
#define FILM_CH_4        (1 << 3)
#define FILM_CH_5        (1 << 4)
#define FILM_CH_6        (1 << 5)
#define FILM_CH_7        (1 << 6)
#define FILM_CH_8        (1 << 7)
#define FILM_CH_9        (1 << 8)
#define FILM_CH_10       (1 << 9)
#define FILM_CH_11       (1 << 10)
#define FILM_CH_12       (1 << 11)
#define FILM_CH_13       (1 << 12)
#define FILM_CH_14       (1 << 13)
#define FILM_CH_15       (1 << 14)
#define FILM_CH_16       (1 << 15)
#define FILM_CH_17       (1 << 16)
#define FILM_CH_18       (1 << 17)
#define FILM_CH_19       (1 << 18)
#define FILM_CH_20       (1 << 19)
#define FILM_CH_21       (1 << 20)
#define FILM_CH_22       (1 << 21)
#define FILM_CH_23       (1 << 22)
#define FILM_CH_24       (1 << 23)
#define FILM_CH_25       (1 << 24)
#define FILM_CH_26       (1 << 25)
#define FILM_CH_27       (1 << 26)
#define FILM_CH_28       (1 << 27)
#define FILM_CH_29       (1 << 28)
#define FILM_CH_30       (1 << 29)
#define FILM_CH_31       (1 << 30)
#define FILM_CH_32       (1 << 31)

#define FILM_TRA_MOTOR(i, x)         for (i = 0; i < MOTOR_CHANNEL; i++){  \
                                    if (!(x & (1 << i))) continue;

#define FILM_TRA_BLOCK_END        }

/* 电机方向 */
typedef enum {
  Film_Forward,
  Film_Reversal,
  Film_Stop
}Film_DIR;

/* Film类函数处理结果 */
typedef enum{
  Film_OK,
  Film_No_Reset,
  Film_CH_Err,
  Film_MEM_Exp,
}film_err;

/* 电机运动模式 */
typedef enum{
  FILM_RESET = 0x00U,
  FILM_ROLL = 0x01U,
  FILM_F_ROLL = 0x02U
}film_m_act;

/* 电机状态 */
typedef enum{
  Film_M_OK,  //电机正常
  Film_M_Open_OK, //卷膜完成
  Film_M_Reset_OK, //重置行程完成
  Film_M_Wait_Chk, //到达限位，等待确认
  Film_M_OverEleCur, //电机过流
  Film_M_MEM_Exp = 0x02U, //电机储存信息异常
  Film_M_Up_Limit_Exp = 0x04U,  //上限位异常
  Film_M_Down_Limit_Exp = 0x05U, //下限为异常
  Film_M_Run_Exp = 0x07U, //电机异常（检测到电压过低）
}film_m_sta;

typedef enum{
  Film_EleCur_OK,
  Film_EleCur_UnInit,  //电机还未保存电流
  Film_EleCur_Exp, //电流保存异常
  Film_EleCur_Over //电流过流
}film_cur_sta;

struct Film_PCB{
  Film_DIR motor_dir[MOTOR_CHANNEL];  //电机运动方向
  film_u32 m_channel;         //电机挂载记录
  film_u8 reset_m_dir_flag; //重置行程电机方向标志位(需要初始化)
  film_u8 second_stage_flag;  //重置行程第二阶段标志位(需要初始化)

  film_u8 run_open_val[MOTOR_CHANNEL];  //最近的开度值
  film_u8 last_open_val[MOTOR_CHANNEL]; //上一次的开度值
  film_u8 rt_open_val[MOTOR_CHANNEL]; //实时开度值
  film_u8 adjust_open_flag[MOTOR_CHANNEL]; //检测到开度不准，校正开度标志位(需要初始化)

  film_u16 total_roll_time[MOTOR_CHANNEL];  //重置行程后测量完整卷膜时长
  film_u16 run_roll_time[MOTOR_CHANNEL];  //运行的卷膜时长

  film_m_sta run_motor_exp_sta[MOTOR_CHANNEL]; //电机运行异常状态(需要初始化)
  film_u8 run_motor_sta_flag[MOTOR_CHANNEL]; //电机运行时的状态是否确认记录标志位(需要初始化)

  film_cur_sta ele_cur_sta[MOTOR_CHANNEL]; //电流状态
  film_double over_ele_cur[MOTOR_CHANNEL]; //过流电流值，卷膜电流不能超过该值
  film_u16 run_ele_cur[MOTOR_CHANNEL]; //实时采集的电流值
  film_u16 clt_ele_cur[MOTOR_CHANNEL]; //重置行程里，采集的卷膜电流
  film_u32 clt_ele_cur_num[MOTOR_CHANNEL]; //采集电流的次数（需要初始化）
  film_u32 lower_ele_cur_tim[MOTOR_CHANNEL];  //电流值过低时计时判断(需要初始化)
  film_u32 over_ele_cur_tim[MOTOR_CHANNEL]; //过流计时判断（需要初始化）
  film_u32 lower_ele_cur_m_channel; //低电流检测名单(需要初始化)

  film_u8 detect_m_overtime_flag; //允许检测电机超时机制标志位(需要初始化)
  
  film_u8 ctrl_timing_en_sig; //使能计时标记(需要初始化)
  film_u32 ctrl_timing_num[MOTOR_CHANNEL]; //每路电机的卷膜计时(需要初始化)
  film_u32 ctrl_com_timing_num; //给非电机流程计时用的
};

void Film_Param_Init(void);
film_err Film_Set_Ele_Cur_Threshold(film_u8 *ch_buf, film_u8 *cur_lmt ,film_u8 ch_num);

film_err Film_Motor_Reset_Rolling(film_u8 *ch_buf, film_u8 ch_num);
film_err Film_Motor_Rolling(film_u8 *open_val, film_u8 *ch_buf, film_u8 ch_num);

void Film_Load_Tim_Var(film_u32 fre_div_times, film_u32 cur_times);   //开发人员需要将该函数放入定时器处理函数中
void Film_Load_Tim_Var(void);

#endif
