#include "film.h"
#include "film_mem.h"

film_8 FM_PrintDBG_Buf[PRINT_DBG_LEN];
Film_PCB film_pcb;

/* 读取一路电机的电流值，单位是mA */
_film_(weak) film_u32 Film_Read_Analog_Ele_Current_CH(film_u8 ch)
{
  FILM_DEBUGF(BT_CB_DBG, (FM_PrintDBG_Buf, "warning: [Film_Read_Analog_Ele_Current_CH] function is not implemented!\n"));
  return 0;
}
/* 根据传入的路数和方向，控制该路电机转动 */
_film_(weak) void Film_Ctrl_Motor_CH(film_u8 ch, Film_DIR dir)
{
  FILM_DEBUGF(BT_CB_DBG, (FM_PrintDBG_Buf, "warning: [Film_Ctrl_Motor_CH] function is not implemented!\n"));
}

/* 卷膜时的监听任务，开发者来提供具体事项，如接收服务器指令 */
_film_(weak) void Film_Listening_Task(void)
{
  FILM_DEBUGF(BT_CB_DBG, (FM_PrintDBG_Buf, "warning: [Film_Listening_Task] function is not implemented!\n"));
}
/* 卷膜流程中的状态信息投递任务，开发者根据传入的状态来提供具体事项，如将状态上报给服务器 */
_film_(weak) void Film_Status_Msg_Delivery_Task(film_u8 ch, film_m_sta sta)
{
  FILM_DEBUGF(BT_CB_DBG, (FM_PrintDBG_Buf, "warning: [Film_Status_Msg_Delivery_Task] function is not implemented!\n"));  
}

/* 函数声明 */
film_err Film_Motor_Start_Task(film_u8 *ch_buf, film_u8 ch_num, film_m_act act);
void Film_Motor_Monitor_Task(void);
void Film_Motor_End_Task(void);

void Film_Store_Exp_Handler_CH(film_u8 ch, film_m_act act);

/* 一些数据处理函数声明 */
void Film_u8Tou16(film_u8 *src_buf, film_u16 *obj_buf, film_u32 src_len);
void Film_u16Tou8(film_u16 *src_buf, film_u8 *obj_buf, film_u32 src_len);

void Film_Param_Init(void)
{
  film_u8 i;
  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    film_pcb.run_motor_exp_sta[i] = Film_M_OK;
    film_pcb.run_motor_sta_flag[i] = 0;
    film_pcb.clt_ele_cur_num[i] = 0;
    film_pcb.lower_ele_cur_tim[i] = 0;
    film_pcb.ctrl_timing_num[i] = 0;
    film_pcb.second_stage_flag[i] = 0;
    film_pcb.reset_m_dir_flag[i] = 0;
  }
  film_pcb.ctrl_com_timing_num = 0;
  film_pcb.detect_m_overtime_flag = 0;
}

void Film_Set_Open_Value(film_u8 *ch_buf, film_u8 ch_num, film_u8 *open_buf)
{
  film_u8 i;
  for (i = 0; i < ch_num; i++)
    film_pcb.run_open_val[ch_buf[i] - 1] = open_buf[i];
}

film_err Film_Motor_Run_Task(film_u8 *ch_buf, film_u8 ch_num, film_m_act act)
{
  film_err sta;

  sta = Film_Motor_Start_Task(&ch_buf[0], ch_num, act);
  if (sta) return sta;

  Film_Motor_Monitor_Task();
  Film_Motor_End_Task();

  return Film_OK;
}

/******************************卷膜定时相关函数****************************************/
/* 开发人员需要将该函数放入定时器处理函数中 */
void Film_Load_Tim_Div_Var(film_u32 fre_div_times, film_u32 cur_times)
{
  film_u8 i;

  if (cur_times < fre_div_times) return;

  FILM_TRA_MOTOR(i, film_pcb.ctrl_timing_en_sig)
      film_pcb.ctrl_timing_num[i]++;
  FILM_TRA_BLOCK_END

  film_pcb.ctrl_com_timing_num++;
}

/* 调用到用户实现的定时器处理函数中 */
void Film_Load_Tim_Var(void)
{
  film_u8 i;
  
  FILM_TRA_MOTOR(i, film_pcb.ctrl_timing_en_sig)
      film_pcb.ctrl_timing_num[i]++;
  FILM_TRA_BLOCK_END

  film_pcb.ctrl_com_timing_num++; 
}

/* 卷膜启动工作，开启控制心跳计数 */
void Film_Start_Timming_CH(film_u8 ch)
{
  film_pcb.ctrl_timing_en_sig |= (1 << (ch - 1));
  film_pcb.ctrl_timing_num[ch - 1] = 0;
}

/* 卷膜停止工作，停止控制心跳计数 */
void Film_Stop_Timming_CH(film_u8 ch)
{
  film_pcb.ctrl_timing_en_sig &= ~(1 << (ch - 1));
  film_pcb.ctrl_timing_num[ch - 1] = 0;
}

/**********************************************与服务器下发参数设置有关函数****************************************************/
film_err Film_Set_Ele_Cur_Threshold(film_u8 *ch_buf, film_u8 *cur_lmt ,film_u8 ch_num)
{
  film_u8 i;
  if (ch_num > MOTOR_CHANNEL || !ch_num) return Film_CH_Err;
  for (i = 0; i < ch_num; i++)
  {
    if (cur_lmt[i] < 1 || cur_lmt[i] > 100) cur_lmt[i] = 20;
    if (Film_MEM_Save_Param_CH(FILM_MEM_ELE_CUR_LMT_BASE_ADDR, &cur_lmt[i], ch_buf[i]) != FILM_MEM_OK) return Film_MEM_Exp;
  }
  return Film_OK;
}

film_err Film_Set_RT_status_Report_Frq(film_u8 *ch_buf, film_u8 *sta_frq ,film_u8 ch_num)
{
  film_u8 i;
  if (ch_num > MOTOR_CHANNEL || !ch_num) return Film_CH_Err;
  for (i = 0; i < ch_num; i++)
  {
    if (sta_frq[i] < 1 || sta_frq[i] > 10) sta_frq[i] = 3;  //sta_frq * 10S
    if (Film_MEM_Save_Param_CH(FILM_MEM_RT_STA_FRQ_BASE_ADDR, &sta_frq[i], ch_buf[i]) != FILM_MEM_OK) return Film_MEM_Exp;
  }
  return Film_OK;
}

/***************************************************************************************************************************/

/****************************************与电机运动有关函数***************************************/

void Film_Set_Motor_Status_CH(film_u8 ch, film_m_sta sta)
{
  film_pcb.run_motor_exp_sta[ch - 1] = sta;
  film_pcb.run_motor_sta_flag[ch - 1] = FILM_SET_FLAG;
}

  /* 复位空闲的电机（将空闲的电机从任务通道上移除） */
void Film_Channel_Reset(void)
{
  film_u8 i;

  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    if (film_pcb.m_run_mode[i] != FILM_IDIE) continue;
    film_pcb.m_reset_channel &= ~(1 << i);
    film_pcb.m_open_channel &= ~(1 << i);
    film_pcb.m_f_open_channel &= ~(1 << i);
  }
}

/* 挂载电机和设置运动模式 */
void Film_Load_Motor_CH(film_u8 ch, film_m_act act)
{
  /* 如果该路电机还在强制卷膜，禁止切换到重置行程或者强制卷膜 */
  if (film_pcb.m_run_mode[ch - 1] == FILM_F_ROLL) return;

  switch (act)
  {
    case FILM_RESET : 
      film_pcb.m_reset_channel |= (1 << (ch - 1));
      film_pcb.m_open_channel &= ~(1 << (ch - 1));
      film_pcb.m_f_open_channel &= ~(1 << (ch - 1));
    break;

    case FILM_ROLL : 
      film_pcb.m_open_channel |= (1 << (ch - 1));
      film_pcb.m_reset_channel &= ~(1 << (ch - 1));
      film_pcb.m_f_open_channel &= ~(1 << (ch - 1));
    break;

    case FILM_F_ROLL :
      film_pcb.m_f_open_channel |= (1 << (ch - 1));
      film_pcb.m_reset_channel &= ~(1 << (ch - 1));
      film_pcb.m_open_channel &= ~(1 << (ch - 1));
    break;
  }
  film_pcb.m_run_mode[ch - 1] = act;
}
/* 挂起电机和复位运动模式 */
void Film_Pending_Motor_CH(film_u8 ch, film_m_act act)
{
  if (act == FILM_RESET)
    film_pcb.m_reset_channel &= ~(1 << (ch - 1));
  else if (act == FILM_ROLL)
    film_pcb.m_open_channel &= ~(1 << (ch - 1));
  else if (act == FILM_F_ROLL)
    film_pcb.m_f_open_channel &= ~(1 << (ch - 1));

  film_pcb.m_run_mode[ch - 1] = FILM_IDIE; 
}

/* 将一路电机从一个任务通道转移到另一个任务通道 */
void Film_Move_Load_Motor_CH(film_u8 ch, film_m_act src_act, film_m_act obj_act)
{
  if (src_act == FILM_ROLL)
  {
    film_pcb.m_open_channel &= ~(1 << ch - 1);
    if (obj_act == FILM_RESET) film_pcb.m_reset_channel |= (1 << (ch - 1));
    else if (obj_act == FILM_F_ROLL) film_pcb.m_f_open_channel |= (1 << (ch - 1));
  }

  film_pcb.m_run_mode[ch - 1] = obj_act;
}

film_u32 Film_Select_Obj_Channel(film_m_act act)
{
  if (act == FILM_RESET) return film_pcb.m_reset_channel;
  else if (act == FILM_ROLL) return film_pcb.m_open_channel;
  else if (act == FILM_F_ROLL)return film_pcb.m_f_open_channel;
}

/* 设置电机方向 */
void Film_Set_Motor_Dir(film_m_act act)
{
  film_u32 obj_channel;
  film_u8 i;
  
  if (act == FILM_RESET)  obj_channel = film_pcb.m_reset_channel;
  else if (act == FILM_ROLL) obj_channel = film_pcb.m_open_channel;
  else if (act == FILM_F_ROLL) obj_channel = film_pcb.m_f_open_channel;

  FILM_TRA_MOTOR(i, obj_channel)
    switch (act)
    {
      case FILM_RESET : 
        film_pcb.motor_dir[i] = (!film_pcb.reset_m_dir_flag[i]) ? Film_Reversal : Film_Forward;
        film_pcb.reset_m_dir_flag[i] = ~film_pcb.reset_m_dir_flag[i]; 
      break;

      case FILM_ROLL : 
        if (film_pcb.last_open_val[i] < film_pcb.run_open_val[i])
          film_pcb.motor_dir[i] = Film_Forward;
        else if (film_pcb.last_open_val[i] > film_pcb.run_open_val[i])
          film_pcb.motor_dir[i] = Film_Reversal;
        else  /* 开度相同，挂起该电机，设置卷膜完成，投递消息 */
        {
          film_pcb.motor_dir[i] = Film_Stop; 
          Film_Pending_Motor_CH(i + 1, FILM_ROLL);
          Film_Set_Motor_Status_CH(i + 1, Film_M_Open_OK);
          Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
        }
      break;

      case FILM_F_ROLL :
        film_pcb.motor_dir[i] = (film_pcb.run_open_val[i]) ? Film_Forward : Film_Reversal;
      break; 
    }
  FILM_TRA_BLOCK_END
}

/* 取出被挂载的运动部件和他们的运行开度，上一次开度，计算出每个挂载的电机的方向 */
void Film_Motor_Start_Run(film_u32 obj_ch)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, obj_ch)
    //if (film_pcb.run_motor_exp_sta[i] > Film_M_Wait_Chk) continue;
    film_pcb.run_motor_sta_flag[i] = FILM_RESET_FLAG;
    Film_Ctrl_Motor_CH(i + 1, film_pcb.motor_dir[i]);
    if (film_pcb.motor_dir[i] == Film_Forward)
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Forward\n", i + 1));
    else
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Reversal\n", i + 1));

    Film_Start_Timming_CH(i + 1);
  FILM_TRA_BLOCK_END
}

void Film_Finish_Rolling(film_u8 ch)
{
  Film_DIR dir_tmp = Film_Stop;
  Film_Ctrl_Motor_CH(ch, dir_tmp);
  Film_Stop_Timming_CH(ch);
}
/***************************************************************************************************************/

/**************************************与电机流程控制有关函数*****************************************************/

/* 保存该路数的三个开度值到储存器 */
film_err Film_Save_Open_Val_CH(film_u8 ch)
{
  if (Film_MEM_Save_Param_CH(FILM_MEM_RT_OPEN_BASE_ADDR, &film_pcb.rt_open_val[ch - 1], ch)) return Film_MEM_Exp;
  if (Film_MEM_Save_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.last_open_val[ch - 1], ch)) return Film_MEM_Exp;
  if (Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[ch - 1], ch)) return Film_MEM_Exp;

  return Film_OK;
}

film_err Film_Clear_All_Opening_CH(film_u8 ch)
{ 
  film_u8 mem_sta_buf[3];
  film_u8 err_flag = FILM_RESET_FLAG;
  film_u8 i = 0;

  film_pcb.run_open_val[ch - 1] = 0;
  mem_sta_buf[i++] = Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[ch - 1], ch);
  film_pcb.last_open_val[ch - 1] = 0;
  mem_sta_buf[i++] = Film_MEM_Save_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.run_open_val[ch - 1], ch);
  film_pcb.rt_open_val[ch - 1] = 0;
  mem_sta_buf[i++] = Film_MEM_Save_Param_CH(FILM_MEM_RT_OPEN_BASE_ADDR, &film_pcb.run_open_val[ch - 1], ch);

  for (i = 0; i < sizeof(mem_sta_buf); i++)
  {
    if (!mem_sta_buf[i]) continue;
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor opening clear ERR! [Film_Clear_All_Opening_CH]\n", ch));
    err_flag = FILM_SET_FLAG;
    break;
  }
  if (err_flag)
  {
    Film_Set_Motor_Status_CH(ch, Film_M_MEM_Exp);
    Film_Status_Msg_Delivery_Task(ch, film_pcb.run_motor_exp_sta[ch - 1]);
    return Film_MEM_Exp;
  }

  return Film_OK;
}

film_err Film_Clear_Reset_OK_Flag_CH(film_u8 ch)
{
  if (!Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, ch, FILM_MEM_FLAG_RESET_MODE)) return Film_OK;

  Film_Set_Motor_Status_CH(ch, Film_M_MEM_Exp);
  Film_Status_Msg_Delivery_Task(ch, film_pcb.run_motor_exp_sta[ch - 1]);
  return Film_MEM_Exp;
}

void Film_Save_ResetRoll_Time(void)
{ 
  film_u8 time_tmp[2];
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_reset_channel)
    if (!film_pcb.second_stage_flag[i]) continue;
    if (film_pcb.run_motor_exp_sta[i] != Film_M_Wait_Chk) continue;
    if (film_pcb.run_time_save_flag[i]) continue; //如果已保存，不需要重复错误保存（因为有些保存的会等着其他还在运动的电机，再次重复保存，保存的时间是等待的时间）
    film_pcb.total_roll_time[i] = film_pcb.ctrl_timing_num[i];
    Film_u16Tou8(&film_pcb.total_roll_time[i], &time_tmp[0], 1);
    if (Film_MEM_Save_Param_CH(FILM_MEM_ROLL_TIME_BASE_ADDR, &time_tmp[0], i + 1))
      Film_Store_Exp_Handler_CH(i + 1, FILM_RESET);
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor total roll time is %dS\n", i + 1, film_pcb.total_roll_time[i]));
    film_pcb.run_time_save_flag[i] = FILM_SET_FLAG;
  FILM_TRA_BLOCK_END
}

/* 根据绝对开度值，计算出挂载的电机需要的卷膜时长 */
void Film_Calculate_Roll_Time(void)
{
  film_u8 t_roll_time_tmp[2];  
  film_u8 abs_open_val;  //要运行的绝对开度值
  film_u8 i;
  
  /* 遍历挂载的电机 */
  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    /* 读取卷膜总时长 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_ROLL_TIME_BASE_ADDR, &t_roll_time_tmp[0], i + 1))
    {
      Film_Store_Exp_Handler_CH(i + 1, FILM_ROLL);
      continue;
    }
    Film_u8Tou16(&t_roll_time_tmp[0], &film_pcb.total_roll_time[i], sizeof(t_roll_time_tmp));
    /* 得到绝对开度，然后用来计算卷膜时长 */
    abs_open_val = (film_pcb.motor_dir[i] == Film_Forward) ? (film_pcb.run_open_val[i] - film_pcb.last_open_val[i]) \
    : (film_pcb.last_open_val[i] - film_pcb.run_open_val[i]);
    film_pcb.run_roll_time[i] = abs_open_val * 0.01 * film_pcb.total_roll_time[i] + 0.5;
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor run roll time is %dS\n", i + 1, film_pcb.run_roll_time[i]));
  FILM_TRA_BLOCK_END
}

/* 除了检查，还兼具将没有重置行程的转移到其他任务通道 */
void Film_Check_Reset_Roll_Flag(void)
{
  film_u8 reset_flag[MOTOR_CHANNEL];
  film_u8 i;

  Film_MEM_Get_Flag(FILM_MEM_RE_OK_FLAG_BASE_ADDR, &reset_flag[0]);

  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    if (reset_flag[i] == FILM_MEM_SET_FLAG) continue;
    /* 如果开度是全开或全关，转移到强制卷膜 */
    if (film_pcb.run_open_val[i] == FILM_ALL_CLOSE || film_pcb.run_open_val[i] == FILM_ALL_OPEN)
    {
      Film_Move_Load_Motor_CH(i + 1, FILM_ROLL, FILM_F_ROLL);
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor did not reset its route, move to force open task ! [Film_Check_Reset_Roll_Flag]\n", i + 1));
      continue;
    }
    /* 如果是其他开度，转移到重置行程 */
    Film_Move_Load_Motor_CH(i + 1, FILM_ROLL, FILM_RESET);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor did not reset its route, move to reset task ! [Film_Check_Reset_Roll_Flag]\n", i + 1));
  FILM_TRA_BLOCK_END
}


film_u8 Film_Reset_Stage_Status_Check(film_reset_stage stage)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_reset_channel)
    if (stage == Film_Reset_First)
    {
      if (film_pcb.second_stage_flag[i] || film_pcb.run_motor_exp_sta[i] != Film_M_Wait_Chk) return 0;
    }
    else
    {
      if (!film_pcb.second_stage_flag[i] || film_pcb.run_motor_exp_sta[i] != Film_M_Wait_Chk) return 0;
    }
  FILM_TRA_BLOCK_END

  return 1;
}

film_u8 Film_Motor_Channel_Sig(void)
{
  if (film_pcb.m_reset_channel) return 0;
  if (film_pcb.m_open_channel) return 0;
  if (film_pcb.m_f_open_channel) return 0;

  return 1;
}
/********************************************************************************************************************/


/******************************************************与电流检测机制有关函数***********************************************************/
/* 初始化电机电流检测 */
void Film_Motor_EleCurrent_Init(void)
{
  film_u8 cur_tmp[2], cur_flag, cur_lmt;
  film_u16 saved_cur;
  film_u32 obj_channel;
  film_u8 i, j;

  FILM_TRA_MOTOR_ACT(i, j, obj_channel)
    /* 检查是否保存过电流值 */
    if (Film_MEM_Get_Flag_CH(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, &cur_flag, (j + 1)))
    {
      film_pcb.ele_cur_sta[j] = Film_EleCur_Exp;
      FM_PRINT((FM_PrintDBG_Buf, "The current storage of the %d th motor is abnormal! [Film_Motor_EleCurrent_Init]\n", j + 1));
    }
    else if (cur_flag != FILM_MEM_SET_FLAG)  
    {
      film_pcb.ele_cur_sta[j] = Film_EleCur_UnInit;
      FM_PRINT((FM_PrintDBG_Buf, "The current value of the %d th motor is null [Film_Motor_EleCurrent_Init]\n", j + 1));
    }
    else 
      film_pcb.ele_cur_sta[j] = Film_EleCur_OK;

    if (film_pcb.ele_cur_sta[j] != Film_EleCur_OK) continue;
    /* 读取电流阈值，这里可以容错，允许储存错误 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_ELE_CUR_LMT_BASE_ADDR, &cur_lmt, (j + 1))) cur_lmt = 20;
    if (cur_lmt < 1 || cur_lmt > 100) cur_lmt = 20; //赋值默认值，2.0倍电流阈值 
    FM_PRINT((FM_PrintDBG_Buf, "The current threshold of the %d th motor is %.1lf times [Film_Motor_EleCurrent_Init]\n", j + 1, (film_double)cur_lmt * 0.1));
    /* 读取电流值 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_ELE_CUR_BASE_ADDR, &cur_tmp[0], (j + 1)))
      saved_cur = FILM_DEFAULT_CUR_VALUE;
    else
      Film_u8Tou16(&cur_tmp[0], &saved_cur, 2);
    /* 计算出过流阈值 */
    film_pcb.over_ele_cur[j] = (film_double)saved_cur * ((film_double)cur_lmt * 0.1);
    FM_PRINT((FM_PrintDBG_Buf, "The maximum current value of the %d th motor is %lf mA\n", j + 1, film_pcb.over_ele_cur[j]));
  FILM_TRA_ACT_BLOCK_END
}

void Film_Detect_OverCur(void)
{
  film_u32 obj_channel;
  film_u8 i, j;
  
  FILM_TRA_MOTOR_ACT(i, j, obj_channel)
    if (film_pcb.ctrl_timing_num[j] < FILM_DETECT_START_TIME) continue;
    if (film_pcb.ele_cur_sta[j] > Film_EleCur_OK) continue;

    film_pcb.run_ele_cur[j] = Film_Read_Analog_Ele_Current_CH(j + 1);
    if (film_pcb.run_ele_cur[j] < film_pcb.over_ele_cur[j]) 
    {
      film_pcb.over_ele_cur_tim[j] = film_pcb.ctrl_timing_num[j];
      continue;
    }
    if (!film_pcb.over_ele_cur_tim[j]) 
      film_pcb.over_ele_cur_tim[j] = film_pcb.ctrl_timing_num[j];
    
    if (film_pcb.ctrl_timing_num[j] < film_pcb.over_ele_cur_tim[j] + 3) continue;

    Film_Finish_Rolling(j + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor over current and it is : %dmA [Film_Detect_OverCur]\n", j + 1, film_pcb.run_ele_cur[j]));
    Film_Set_Motor_Status_CH(j + 1, Film_M_OverEleCur);
    Film_Pending_Motor_CH(j + 1, (film_m_act)i);
    Film_Status_Msg_Delivery_Task(j + 1, film_pcb.run_motor_exp_sta[j]);
    /* 开度卷膜需要保存当前开度 */
    if (i != FILM_ROLL) continue;
    film_pcb.run_open_val[j] = film_pcb.rt_open_val[j];
    film_pcb.last_open_val[j] = film_pcb.rt_open_val[j];
    if (Film_Save_Open_Val_CH(j + 1) != Film_OK)
      Film_Store_Exp_Handler_CH(j + 1, (film_m_act)i);
  FILM_TRA_ACT_BLOCK_END
}

void Film_Detect_LowerCur(film_m_act act)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.lower_ele_cur_m_channel)
    film_pcb.run_ele_cur[i] = Film_Read_Analog_Ele_Current_CH(i + 1);
    if (film_pcb.run_ele_cur[i] > FILM_LOWER_CUR_VALUE)
    {
      film_pcb.lower_ele_cur_tim[i] = film_pcb.ctrl_timing_num[i];
      continue;
    } 

    if (!film_pcb.lower_ele_cur_tim[i])
      film_pcb.lower_ele_cur_tim[i] = film_pcb.ctrl_timing_num[i];
    
    if (film_pcb.ctrl_timing_num[i] < film_pcb.lower_ele_cur_tim[i] + 1) continue;
    Film_Finish_Rolling(i + 1);
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor exception! [Film_Detect_LowerCur]\n", i + 1));
    if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE))
    {
      Film_Store_Exp_Handler_CH(i + 1, act);
      continue;
    }
    Film_Set_Motor_Status_CH(i + 1, Film_M_Run_Exp);
    Film_Pending_Motor_CH(i + 1, act);
    Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
    film_pcb.lower_ele_cur_m_channel &= ~(1 << i);
  FILM_TRA_BLOCK_END
}

/* 重置行程中，采集电流 */
void Film_Collect_Ele_Cur(void)
{
  film_u32 clt_ele_cur_tmp;
  film_u8 i;

  if (film_pcb.clt_ele_tim_num + FILM_CUR_COLLECT_FREQ > film_pcb.ctrl_com_timing_num) return;  //每采集周期才采集一次

  FILM_TRA_MOTOR(i, film_pcb.m_reset_channel)
    if ((!film_pcb.second_stage_flag[i]) || (film_pcb.run_motor_exp_sta[i] == Film_M_Wait_Chk)) continue;
    clt_ele_cur_tmp = Film_Read_Analog_Ele_Current_CH(i + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor clt_ele_cur_tmp is : %dmA [Film_Collect_Ele_Cur]\n", i + 1, clt_ele_cur_tmp));
    if (clt_ele_cur_tmp < FILM_DEFAULT_LOW_CUR_VALUE) continue;
    film_pcb.clt_ele_cur[i] += clt_ele_cur_tmp;
    film_pcb.clt_ele_cur_num[i]++;
  FILM_TRA_BLOCK_END

  film_pcb.clt_ele_tim_num = film_pcb.ctrl_com_timing_num;
}

/* 计算和保存电流值 */
void Film_Cal_Save_Ele_Cur_CH(film_u8 ch)
{
  film_u8 cur_tmp[2];

  film_pcb.clt_ele_cur[ch - 1] /= film_pcb.clt_ele_cur_num[ch - 1];
  Film_u16Tou8(&film_pcb.clt_ele_cur[ch - 1], &cur_tmp[0], 1);
  Film_MEM_Save_Param_CH(FILM_MEM_ELE_CUR_BASE_ADDR, &cur_tmp[0], ch);
  Film_MEM_Set_Flag_CH(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, ch, FILM_MEM_FLAG_SET_MODE);
  FM_PRINT((FM_PrintDBG_Buf, "The %dth motor collect current is : %dmA [Film_Cal_Save_Ele_Cur_CH]\n", ch, film_pcb.clt_ele_cur[ch - 1]));
}
/***************************************************************************************************************/

/****************************************************异常检测相关函数********************************************/

/*储存异常处理，包含了挂起储存异常的电机，设置异常状态，投递状态 */
void Film_Store_Exp_Handler_CH(film_u8 ch, film_m_act act)
{
  Film_Pending_Motor_CH(ch, act);
  Film_Set_Motor_Status_CH(ch, Film_M_MEM_Exp);
  Film_Status_Msg_Delivery_Task(ch, film_pcb.run_motor_exp_sta[ch - 1]);
}

/* 开度卷膜中，检测除了全关和全开之外的其他开度是否已经到达 */
void Film_Detect_Opening(void)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    if (film_pcb.run_open_val[i] == FILM_ALL_CLOSE || film_pcb.run_open_val[i] == FILM_ALL_OPEN) continue;
    if (film_pcb.ctrl_timing_num[i] < film_pcb.run_roll_time[i]) continue;
    Film_Finish_Rolling(i + 1);
    film_pcb.last_open_val[i] = film_pcb.run_open_val[i];
    film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
    if (Film_Save_Open_Val_CH(i + 1) == Film_MEM_Exp)
    {
      Film_Store_Exp_Handler_CH(i + 1, FILM_ROLL);
      continue;
    }
    Film_Set_Motor_Status_CH(i + 1, Film_M_Open_OK);
    Film_Pending_Motor_CH(i + 1, FILM_ROLL);
    Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reached opening... [Film_Detect_Opening]\n", i + 1));    
  FILM_TRA_BLOCK_END
}

void Film_Detect_Limit(void)
{ 
  film_u32 obj_channel;
  film_u8 i, j;

  FILM_TRA_MOTOR_ACT(i, j, obj_channel)
    /* 如果卷膜时间少于检测限位开始等待时间，不进行检测 */
    if (film_pcb.ctrl_timing_num[j] < FILM_DETECT_START_TIME) continue;
 
    film_pcb.detect_m_overtime_flag = 1;  //允许后面的电机超时机制检测

    film_pcb.run_ele_cur[j] = Film_Read_Analog_Ele_Current_CH(j + 1);
    /* 如果读取的电流值大于最低电流阈值，正常 */
    if (film_pcb.run_ele_cur[j] > FILM_LOWER_CUR_VALUE) 
    {
      film_pcb.lower_ele_cur_tim[j] = film_pcb.ctrl_timing_num[j];
      continue;
    }
    /* 如果读取的电流值小于最低电流值，开始计时判断是否真的小于最低电流值 */
    if (!film_pcb.lower_ele_cur_tim[j])
      film_pcb.lower_ele_cur_tim[j] = film_pcb.ctrl_timing_num[j];

    if (film_pcb.ctrl_timing_num[j] < (film_pcb.lower_ele_cur_tim[j] + 3)) continue;

    /* 如果检测到电流低于最低电流阈值3S钟*/
    if (i == FILM_RESET)
    {
      if (film_pcb.run_motor_exp_sta[j] == Film_M_Wait_Chk) continue;
      if (!film_pcb.second_stage_flag[j])
        FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach down limit...(RESET) [Film_Detect_Limit]\n", j + 1));
      else FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach up limit...(RESET) [Film_Detect_Limit]\n", j + 1));
      film_pcb.run_open_val[j] = (film_pcb.second_stage_flag[j]) ? FILM_ALL_OPEN : FILM_ALL_CLOSE;   
    }  
    else
    {
      /* 全开和全关不需要校正 */
      if (film_pcb.run_open_val[j] == FILM_ALL_CLOSE || film_pcb.run_open_val[j] == FILM_ALL_OPEN) 
      {
        if (film_pcb.run_open_val[j] == FILM_ALL_CLOSE)
          FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach down limit... [Film_Detect_Limit]\n", j + 1));
        else
          FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach up limit... [Film_Detect_Limit]\n", j + 1));
      }
      else
      {
        film_pcb.adjust_open_flag[j] = FILM_SET_FLAG;
        FM_PRINT((FM_PrintDBG_Buf, "The %dth motor opening error, need to correct the opening [Film_Detect_Limit]\n", j + 1));        
      }
    }
    Film_Set_Motor_Status_CH(j + 1, Film_M_Wait_Chk);
  FILM_TRA_ACT_BLOCK_END
}

/*
 @brief   : 卷膜超时处理。如果在卷膜过程中电流小于正常值，说明异常；如果卷膜时间超过最大时间阈值，也是异常
 @para    : 当前动作
 @return  : true or false
 */
void Film_Detect_Motor_OverTime(void)
{
  film_u32 obj_channel;
  film_u8 i, j;

  if (!film_pcb.detect_m_overtime_flag) return;
  film_pcb.detect_m_overtime_flag = 0;

  FILM_TRA_MOTOR_ACT(i, j, obj_channel)
    if (film_pcb.ctrl_timing_num[j] < FILM_ROLL_OVERTIME) continue;
    Film_Finish_Rolling(j + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth of motor overtime! [Film_Detect_Motor_OverTime]\n", j + 1));
    if (Film_Clear_All_Opening_CH(j + 1) || Film_Clear_Reset_OK_Flag_CH(j + 1))
    {
      Film_Store_Exp_Handler_CH(j + 1, (film_m_act)i);
      continue;
    }
    film_pcb.motor_dir[j] == Film_Forward ? Film_Set_Motor_Status_CH(j + 1, Film_M_Up_Limit_Exp) : Film_Set_Motor_Status_CH(j + 1, Film_M_Down_Limit_Exp);
    Film_Pending_Motor_CH(j + 1, (film_m_act)i);
    Film_Status_Msg_Delivery_Task(j + 1, film_pcb.run_motor_exp_sta[j]);
  FILM_TRA_ACT_BLOCK_END
}

/* 校验检测电机是否真的到达了限位，方法是往反方向走3S，同时检测电流值 */
void Film_Verify_Motor_Limit(film_m_act act)
{
  film_u32 obj_channel;
  film_u32 chk_tim_num = 0;
  film_u8 i;

  if (act == FILM_RESET) obj_channel = film_pcb.m_reset_channel;
  else if (act == FILM_ROLL) obj_channel = film_pcb.m_open_channel;
  else if (act == FILM_F_ROLL) obj_channel = film_pcb.m_f_open_channel;

  film_pcb.lower_ele_cur_m_channel = 0;

  FILM_TRA_MOTOR(i, obj_channel)
    if (film_pcb.run_motor_exp_sta[i] != Film_M_Wait_Chk) continue;
    FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor prepare check limit... [Film_Verify_Motor_Limit]\n", i + 1));
    Film_Finish_Rolling(i + 1);
    film_pcb.lower_ele_cur_m_channel |= (1 << i);
    film_pcb.motor_dir[i] = (film_pcb.motor_dir[i] == Film_Forward) ? Film_Reversal : Film_Forward;
    film_pcb.lower_ele_cur_tim[i] = 0;
  FILM_TRA_BLOCK_END
  if (!film_pcb.lower_ele_cur_m_channel) return;
  
  Film_Motor_Start_Run(film_pcb.lower_ele_cur_m_channel);
  film_pcb.ctrl_com_timing_num = 0;

  while (1)
  {
    Film_Detect_LowerCur(act);
    if (film_pcb.ctrl_com_timing_num < chk_tim_num + 3) continue;
    FILM_TRA_MOTOR(i, film_pcb.lower_ele_cur_m_channel)
      Film_Finish_Rolling(i + 1);
      FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor check limit success... [Film_Verify_Motor_Limit]\n", i + 1));

      if (film_pcb.run_motor_exp_sta[i] == Film_M_Run_Exp) continue; 
      if (act == FILM_RESET) 
      {
        if (!film_pcb.second_stage_flag[i]) Film_Set_Motor_Status_CH(i + 1, FILM_M_First_Reset_OK);
        else Film_Set_Motor_Status_CH(i + 1, Film_M_Reset_OK);

        if (!film_pcb.second_stage_flag[i]) continue;
        film_pcb.run_open_val[i] = FILM_ALL_OPEN;
        film_pcb.last_open_val[i] = FILM_ALL_OPEN;
        film_pcb.rt_open_val[i] = FILM_ALL_OPEN;
        if (Film_Save_Open_Val_CH(i + 1) ||
        Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_SET_MODE))
        {
          Film_Store_Exp_Handler_CH(i + 1, FILM_RESET);
          FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor save open or set flag exception!(RESET)  [Film_Verify_Motor_Limit]\n", i + 1));
          continue;
        }
        Film_Cal_Save_Ele_Cur_CH(i + 1);
        Film_Pending_Motor_CH(i + 1, FILM_RESET);
      }
      else
      {
        film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
        film_pcb.last_open_val[i] = film_pcb.run_open_val[i];
        if (act == FILM_ROLL)
        {
          Film_Set_Motor_Status_CH(i + 1, Film_M_Open_OK);
          Film_Pending_Motor_CH(i + 1, FILM_ROLL);
        }
        else if (act == FILM_F_ROLL)
        {
          Film_Set_Motor_Status_CH(i + 1, Film_M_F_Open_OK);
          Film_Pending_Motor_CH(i + 1, FILM_F_ROLL);
        }
      }
      Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
    FILM_TRA_BLOCK_END
    break;
  }
}

void Film_Seek_Reset_Origin(void)
{
  film_u8 i;

  if (!Film_Reset_Stage_Status_Check(Film_Reset_First)) return;

  /* 检查是否到达了限位，将电机异常的剔除 */
  Film_Verify_Motor_Limit(FILM_RESET);
  FILM_TRA_MOTOR(i, film_pcb.m_reset_channel)
    film_pcb.second_stage_flag[i] = FILM_SET_FLAG;
  FILM_TRA_BLOCK_END

  Film_Set_Motor_Dir(FILM_RESET); //换向
  Film_Motor_Start_Run(film_pcb.m_reset_channel);
  film_pcb.ctrl_com_timing_num = 0;
  film_pcb.clt_ele_tim_num = 0;
}
/*********************************************************************************************************************/

/*
 *重置行程参数配置。在重置行程前，挂载需要重置行程的电机
 */
film_err Film_Motor_Start_Task(film_u8 *ch_buf, film_u8 ch_num, film_m_act act)
{
  film_u8 i;

  if (ch_num > MOTOR_CHANNEL || !ch_num) return Film_CH_Err; /* 请求挂载的电机数目超过设置上限 */

  /* 复位空闲电机通道 */
  Film_Channel_Reset();

  /* 根据挂载的运动类型，挂载到相应的通道上 */
  for (i = 0; i < ch_num; i++)
  {
    Film_Load_Motor_CH(ch_buf[i], act);
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor has been loaded...[Film_Motor_Start_Task]\n", ch_buf[i]));
    if (act != FILM_RESET) continue;
    /* 清除重置完成标志位 */
    if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, ch_buf[i], FILM_MEM_FLAG_RESET_MODE))
      Film_Store_Exp_Handler_CH(ch_buf[i], FILM_RESET);    
  }

  /* 开度卷膜需要设置的参数 */
  Film_Check_Reset_Roll_Flag(); /* 没有重置过行程，判断是转移挂载到重置行程，还是强制卷膜 */ 
  /* 储存挂载电机的运行开度，并读取上一次开度值*/
  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    if (Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) ||
    Film_MEM_Read_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.last_open_val[i], i + 1))
    {
      Film_Store_Exp_Handler_CH(i + 1, FILM_ROLL);
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor open value exception!\n", i + 1));
      continue;
    }
    FM_PRINT((FM_PrintDBG_Buf, "The run open value of the %dth motor is %d%%\n", i+1 , film_pcb.run_open_val[i]));
    FM_PRINT((FM_PrintDBG_Buf, "The last open value of the %dth motor is %d%%\n", i+1 , film_pcb.last_open_val[i]));
  FILM_TRA_BLOCK_END

  /* 强制卷膜开度方向设置 */
  FILM_TRA_MOTOR(i, film_pcb.m_f_open_channel)
    if (Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1))
    {
      Film_Store_Exp_Handler_CH(i + 1, FILM_F_ROLL);
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor force open value exception!\n", i + 1));
      continue;      
    }
    FM_PRINT((FM_PrintDBG_Buf, "The force run open value of the %dth motor is %d%%\n", i+1 , film_pcb.run_open_val[i]));
  FILM_TRA_BLOCK_END   

  /* 重置行程 */
  FILM_TRA_MOTOR(i, film_pcb.m_reset_channel)
    film_pcb.reset_m_dir_flag[i] = FILM_RESET_FLAG;
    film_pcb.run_time_save_flag[i] = FILM_RESET_FLAG;
    film_pcb.second_stage_flag[i] = FILM_RESET_FLAG;
  FILM_TRA_BLOCK_END

  /* 设置电机的方向 */
  Film_Set_Motor_Dir(FILM_RESET);
  Film_Set_Motor_Dir(FILM_ROLL);
  Film_Set_Motor_Dir(FILM_F_ROLL);
    /* 初始化电流机制 */
  Film_Motor_EleCurrent_Init();
  /* 计算出挂载的电机需要的卷膜时长 */
  Film_Calculate_Roll_Time();
  /* 挂载电机开始运动 */
  Film_Motor_Start_Run(film_pcb.m_reset_channel);
  Film_Motor_Start_Run(film_pcb.m_open_channel);
  Film_Motor_Start_Run(film_pcb.m_f_open_channel);
  film_pcb.ctrl_com_timing_num = 0;
  film_pcb.clt_ele_tim_num = 0; 

  return Film_OK;
}

void Film_Motor_Monitor_Task(void)
{
  while (1)
  { 
    /* 开度检测 */
    Film_Detect_Opening();
    /* 限位检测 */
    Film_Detect_Limit();
    /* 超时检测 */
    Film_Detect_Motor_OverTime();
    /* 过流检测 */
    Film_Detect_OverCur();

    /* 重置行程基准点搜索*/
    Film_Seek_Reset_Origin();
    /* 重置行程中电流采集 */
    Film_Collect_Ele_Cur();
    /* 保存重置行程运动时长 */
    Film_Save_ResetRoll_Time();

    /* 信息监听 */
    Film_Listening_Task();

    if (Film_Reset_Stage_Status_Check(Film_Reset_Second)) 
      Film_Verify_Motor_Limit(FILM_RESET);
      
    Film_Verify_Motor_Limit(FILM_ROLL);
    Film_Verify_Motor_Limit(FILM_F_ROLL);
    
    if (Film_Motor_Channel_Sig()) break;
  }
  Serial.println("OKOKOKOKOK");
}

void Film_Motor_End_Task(void)
{
  /* 保存采集的电流值 */
  //Film_Cal_Save_Ele_Cur();
  /* 保存采集的电压值 */

  /* 保存三个开度 */
  //Film_Save_Open_Val();

  //film_pcb.second_stage_flag[0] = 0;
}
/*********************************************************************************************/


/* 提供给外部的一些查询函数 */
film_u8 Film_Read_RT_Open_CH(film_u8 ch, film_u8 *open_val)
{
  if (ch < 1 || ch > MOTOR_CHANNEL) return 0xFF;
  return (film_pcb.rt_open_val[ch - 1]);
}

film_u8 Film_Read_Last_Open_CH(film_u8 ch, film_u8 *open_val)
{
  if (ch < 1 || ch > MOTOR_CHANNEL) return 0xFF;
  return (film_pcb.last_open_val[ch - 1]);
}

film_u8 Film_Read_Run_Open_CH(film_u8 ch, film_u8 *open_val)
{
  if (ch < 1 || ch > MOTOR_CHANNEL) return 0xFF;
  return (film_pcb.run_open_val[ch - 1]);
}

void Film_u8Tou16(film_u8 *src_buf, film_u16 *obj_buf, film_u32 src_len)
{
  film_u32 i, j = 0;
  for (i = 0; i < src_len; i++)
  {
    if (i % 2 == 0) obj_buf[j] = (src_buf[i] << 8);
    else obj_buf[j++] |= src_buf[i];
  }
}

void Film_u16Tou8(film_u16 *src_buf, film_u8 *obj_buf, film_u32 src_len)
{
  film_u32 i, j = 0;
  for (i = 0; i < (src_len * 2); i++)
    obj_buf[i] = (i % 2 == 0) ? src_buf[j] >> 8 : src_buf[j++] & 0xFF;
}
