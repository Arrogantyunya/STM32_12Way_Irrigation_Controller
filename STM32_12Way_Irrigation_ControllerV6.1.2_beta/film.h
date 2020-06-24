#ifndef _FILM_H
#define _FILM_H

#include <stdio.h>

// #include "Motor.h"

#include "film_config.h"

#define FILM_DETECT_START_TIME                3U //启动限位检测时间

#define FILM_LOWER_CUR_VALUE                  150 //电流值下限，用来判断电机是否在运动，单位mA

#define FILM_DEFAULT_LOW_CUR_VALUE            140 //经验值，应用的电机的空转最小电流值
#define FILM_DEFAULT_CUR_VALUE                1000 //读取的电流值异常，默认的经验电流值，单位mA

#define FILM_ROLL_OVERTIME                    2000U  //卷膜允许最大时长(S)

#define FILM_CUR_COLLECT_FREQ                 4 //电流采集频率(S)


#define FILM_SET_FLAG                         0x55U
#define FILM_RESET_FLAG                       0x00U

#define FILM_ALL_OPEN                         100U
#define FILM_ALL_CLOSE                        0U
#define FILM_UNKNOWN_OPEN                     0xFFU  

#define FILM_TEMP_OPEN                        FILM_ALL_CLOSE


#define FILM_TRA_MOTOR_ACT(i, j, x)           for (i = 1; i <= 3; i++){ \
                                                x = Film_Select_Obj_Channel((film_m_act)i);  \
                                                for (j = 0; j < MOTOR_CHANNEL; j++){ \
                                                  if (!(x & (1 << j))) continue;

#define FILM_TRA_ACT_BLOCK_END              }\
                                          }  

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
  Film_New_Task,
  Film_CH_Err,
  Film_MEM_Exp,
}film_err;

/* 电机运动模式 */
typedef enum{
  FILM_IDIE,
  FILM_RESET,
  FILM_ROLL,
  FILM_F_ROLL
}film_m_act;

typedef enum{
  Film_Reset_First,
  Film_Reset_Second
}film_reset_stage;

/* 电机状态 */
typedef enum{
  Film_M_IDIE,  //电机空闲
  Film_M_Opening, //正在卷膜
  Film_M_F_Opening, //正在强制卷膜
  Film_M_Reseting, //正在重置行程
  Film_M_Open_OK, //卷膜完成
  Film_M_F_Open_OK, //强制卷膜完成
  FILM_M_First_Reset_OK,  //到达重置行程起始位置
  Film_M_Reset_OK, //重置行程完成
  Film_M_F_Stop_OK, //强制停止成功
  Film_M_Wait_Chk, //到达限位，等待确认
  Film_M_OverEleCur, //电机过流
  Film_M_MEM_Exp, //电机储存信息异常
  Film_M_Up_Limit_Exp,  //上限位异常
  Film_M_Down_Limit_Exp, //下限位异常
  Film_M_Run_Exp, //电机异常（检测到电压过低）
  Film_M_Unkonwn_Open //未知开度（开度卷膜途中断电，且实时开度在全关或者全开上，为了防止服务器以为还正常关棚或着开棚，让服务器目前不确定是否在全开或者全关）
}film_m_sta;

typedef enum{
  Film_EleCur_OK,
  Film_EleCur_UnInit,  //电机还未保存电流
  Film_EleCur_Exp, //电流保存异常
  Film_EleCur_Over //电流过流
}film_cur_sta;

struct Film_PCB{

  Film_DIR motor_dir[MOTOR_CHANNEL];  //电机运动方向
  film_u8 reset_m_dir_flag[MOTOR_CHANNEL]; //重置行程电机方向标志位(需要初始化)
  film_u8 second_stage_flag[MOTOR_CHANNEL];  //重置行程第二阶段标志位(需要初始化)

  film_u8 new_task_request_flag;   //新任务请求标志位（用来实现运行中更改运动任务）(需要初始化)

  film_u8 auto_open_roll_flag[MOTOR_CHANNEL]; //重置完成后自动卷膜到目标开度标志位（需要初始化）

  film_u32 m_reset_channel;         //电机重置行程任务挂载记录
  film_u32 m_open_channel;        //电机开度卷膜任务挂载记录
  film_u32 m_f_open_channel;      //电机强制卷膜任务挂载记录 

  film_m_act m_run_mode[MOTOR_CHANNEL]; //电机当前运行模式(需要初始化)

  film_u8 m_force_stop_flag[MOTOR_CHANNEL]; //强制停止标志位(需要初始化)
  film_u8 re_ok_before_f_roll_flag[MOTOR_CHANNEL]; //强制卷膜前是否重置行程完成的标志位(需要初始化)

  film_u8 run_open_val[MOTOR_CHANNEL];  //最近的开度值
  film_u8 last_open_val[MOTOR_CHANNEL]; //上一次的开度值
  film_u8 rt_open_val[MOTOR_CHANNEL]; //实时开度值
  film_u8 tmp_open_val[MOTOR_CHANNEL]; //暂存开度值，用于一些复杂的操作。
  film_u8 adjust_open_flag[MOTOR_CHANNEL]; //检测到开度不准，校正开度标志位(需要初始化)

  film_u16 total_roll_time[MOTOR_CHANNEL];  //重置行程后测量完整卷膜时长
  film_u16 run_roll_time[MOTOR_CHANNEL];  //运行的卷膜时长
  film_u8 run_time_save_flag[MOTOR_CHANNEL];  //运动时长保存标志位(需要初始化)

  film_m_sta run_motor_exp_sta[MOTOR_CHANNEL]; //电机运行异常状态(需要初始化)

  film_cur_sta ele_cur_sta[MOTOR_CHANNEL]; //电流状态
  film_double over_ele_cur[MOTOR_CHANNEL]; //过流电流值，卷膜电流不能超过该值
  film_u16 run_ele_cur[MOTOR_CHANNEL]; //实时采集的电流值
  film_u16 clt_ele_cur[MOTOR_CHANNEL]; //重置行程里，采集的卷膜电流
  film_u32 clt_ele_cur_num[MOTOR_CHANNEL]; //采集电流的次数（需要初始化）
  film_u32 clt_ele_tim_num; //采集电流计时次数（S）
  film_u32 lower_ele_cur_tim[MOTOR_CHANNEL];  //电流值过低时计时判断(需要初始化)
  film_u32 over_ele_cur_tim[MOTOR_CHANNEL]; //过流计时判断（需要初始化）
  film_u32 lower_ele_cur_m_channel; //低电流检测名单(需要初始化)

  film_u8 detect_m_overtime_flag; //允许检测电机超时机制标志位(需要初始化)
  
  film_u8 ctrl_timing_en_sig; //使能计时标记(需要初始化)
  film_u32 ctrl_timing_num[MOTOR_CHANNEL]; //每路电机的卷膜计时(需要初始化)
  film_u32 ctrl_com_timing_num; //给非电机流程计时用的
};

/* 建议放在上电后初始化流程部分 */
void Film_Param_Init(void);
void Film_Check_Exp_Open_Task(void);
void Film_Motor_Clear_EleCurrent(void);

/* 放在main循环里 */
void Film_Switch_Task(void);
void Film_Auto_Open_Task(void);

/* 放在接受服务器指令流程里 */
film_err Film_Set_Open_Value(film_u8 *ch_buf, film_u8 ch_num, film_u8 *open_buf);
film_err Film_New_Task_Handler(film_u8 *ch_buf, film_u8 ch_num, film_m_act act);
film_err Film_Motor_Run_Task(film_u8 *ch_buf, film_u8 ch_num, film_m_act act);

/* 提供给开发者的些电机参数查询函数 */
film_u8 Film_Read_RT_Opening_CH(film_u8 ch);
film_m_sta Film_Read_Motor_Status_CH(film_u8 ch);

/* 放在接收服务器指令流程里 */
film_err Film_Set_Ele_Cur_Threshold(film_u8 *ch_buf, film_u8 *cur_lmt ,film_u8 ch_num);
void Film_Set_Force_Stop(film_u8 *ch_buf, film_u8 ch_num);

/* 放在开发者实现的定时器处理函数里（必须1S来调用一次该函数） */
void Film_Load_Tim_Var(void);


#endif
