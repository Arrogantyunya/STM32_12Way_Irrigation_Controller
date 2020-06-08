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
film_err Film_Motor_Monitor_Task(film_m_act act);
film_err Film_Motor_End_Task(film_m_act act);
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
  }
  film_pcb.ctrl_com_timing_num = 0;
  film_pcb.detect_m_overtime_flag = 0;
  film_pcb.second_stage_flag = 0;
}

film_err Film_Motor_Reset_Rolling(film_u8 *ch_buf, film_u8 ch_num)
{
  film_err sta;
  /* 挂载电机，设置电机方向，初始化电流检测机制。重置行程默认先关棚找基准点 */
  sta = Film_Motor_Start_Task(&ch_buf[0], ch_num, FILM_RESET);
  if (sta) return sta;
  sta = Film_Motor_Monitor_Task(FILM_RESET);
  if (sta) return sta;
  sta = Film_Motor_End_Task(FILM_RESET);
  if (sta) return sta;

  return Film_OK;
}

/* 开度卷膜 */
film_err Film_Motor_Rolling(film_u8 *open_val, film_u8 *ch_buf, film_u8 ch_num)
{
  film_err err;
  film_u8 i;

  for (i = 0; i < ch_num; i++)
    film_pcb.run_open_val[ch_buf[i] - 1] = open_val[i];

  err = Film_Motor_Start_Task(&ch_buf[0], ch_num, FILM_ROLL);
  if (err != Film_OK) return err;

  Film_Motor_Monitor_Task(FILM_ROLL);
  Film_Motor_End_Task(FILM_ROLL);

  return Film_OK;
}

film_err Film_Force_Rolling(film_u8 *open_val, film_u8 *ch_buf, film_u8 ch_num)
{

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

void Film_Load_Tim_Var(void)
{
  film_u8 i;
  
  FILM_TRA_MOTOR(i, film_pcb.ctrl_timing_en_sig)
      film_pcb.ctrl_timing_num[i]++;
  FILM_TRA_BLOCK_END

  film_pcb.ctrl_com_timing_num++; 
}

/* 卷膜启动工作，开启控制心跳计数 */
void Film_Start_Timming(void)
{
  film_u8 i;

  film_pcb.ctrl_timing_en_sig = film_pcb.m_channel;

  FILM_TRA_MOTOR(i, film_pcb.ctrl_timing_en_sig)
      film_pcb.ctrl_timing_num[i] = 0;
  FILM_TRA_BLOCK_END
}

/* 卷膜停止工作，停止控制心跳计数 */
void Film_Stop_Timming(film_u8 ch)
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
/* 设置电机方向 */
void Film_Set_Motor_Dir(film_m_act act)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
  switch (act)
  {
    case FILM_RESET : 
      film_pcb.motor_dir[i] = (!film_pcb.reset_m_dir_flag) ? Film_Reversal : Film_Forward;
      film_pcb.reset_m_dir_flag = ~film_pcb.reset_m_dir_flag; 
    break;

    case FILM_ROLL : 
      if (film_pcb.last_open_val[i] < film_pcb.run_open_val[i])
        film_pcb.motor_dir[i] = Film_Forward;
      else if (film_pcb.last_open_val[i] > film_pcb.run_open_val[i])
        film_pcb.motor_dir[i] = Film_Reversal;
      else
        film_pcb.motor_dir[i] = Film_Stop;    
    break;

    case FILM_F_ROLL :
      film_pcb.motor_dir[i] = (film_pcb.run_open_val[i]) ? Film_Forward : Film_Reversal;
    break; 
  }
  FILM_TRA_BLOCK_END
}

/* 取出被挂载的运动部件和他们的运行开度，上一次开度，计算出每个挂载的电机的方向 */
void Film_Motor_Start_Run(void)
{
  film_u8 i;
  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (film_pcb.run_motor_exp_sta[i] > Film_M_Wait_Chk) continue;
    film_pcb.run_motor_sta_flag[i] = FILM_RESET_FLAG;
    Film_Ctrl_Motor_CH(i + 1, film_pcb.motor_dir[i]);
    if (film_pcb.motor_dir[i] == Film_Forward)
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Forward\n", i + 1));
    else
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Reversal\n", i + 1));
  FILM_TRA_BLOCK_END
  Film_Start_Timming();
}

void Film_Finish_Rolling(film_u8 ch)
{
  Film_DIR dir_tmp = Film_Stop;
  Film_Stop_Timming(ch);
  Film_Ctrl_Motor_CH(ch, dir_tmp);
}
/***************************************************************************************************************/

/**************************************与电机流程控制有关函数*****************************************************/
void Film_Set_Exp_Status_CH(film_u8 ch, film_m_sta sta)
{
  film_pcb.run_motor_exp_sta[ch - 1] = sta;
  film_pcb.run_motor_sta_flag[ch - 1] = FILM_SET_FLAG;
}

void Film_Save_Open_Val(void)
{
  film_u8 i;
  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    Film_MEM_Save_Param_CH(FILM_MEM_RT_OPEN_BASE_ADDR, &film_pcb.rt_open_val[i], i + 1);
    Film_MEM_Save_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.last_open_val[i], i + 1); 
    Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1); 
  FILM_TRA_BLOCK_END
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
    Film_Set_Exp_Status_CH(ch, Film_M_MEM_Exp);
    Film_Status_Msg_Delivery_Task(ch, film_pcb.run_motor_exp_sta[ch - 1]);
    return Film_MEM_Exp;
  }

  return Film_OK;
}

film_err Film_Clear_Reset_OK_Flag_CH(film_u8 ch)
{
  if (!Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, ch, FILM_MEM_FLAG_RESET_MODE)) return Film_OK;

  Film_Set_Exp_Status_CH(ch, Film_M_MEM_Exp);
  Film_Status_Msg_Delivery_Task(ch, film_pcb.run_motor_exp_sta[ch - 1]);
  return Film_MEM_Exp;
}

void Film_Save_ResetRoll_Time(void)
{ 
  film_u8 time_tmp[2];
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    film_pcb.total_roll_time[i] = film_pcb.ctrl_timing_num[i];
    Film_u16Tou8(&film_pcb.total_roll_time[i], &time_tmp[0], 1);
    Film_MEM_Save_Param_CH(FILM_MEM_ROLL_TIME_BASE_ADDR, &time_tmp[0], i + 1);
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor total roll time is %dS\n", i + 1, film_pcb.total_roll_time[i]));
  FILM_TRA_BLOCK_END
}

/* 根据绝对开度值，计算出挂载的电机需要的卷膜时长 */
film_err Film_Calculate_Roll_Time(void)
{
  film_u8 t_roll_time_tmp[2];  
  film_u8 abs_open_val;  //要运行的绝对开度值
  film_u8 i;
  
  /* 遍历挂载的电机 */
  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    /* 读取卷膜总时长 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_ROLL_TIME_BASE_ADDR, &t_roll_time_tmp[0], i + 1)) return Film_MEM_Exp;
    Film_u8Tou16(&t_roll_time_tmp[0], &film_pcb.total_roll_time[i], sizeof(t_roll_time_tmp));
    /* 得到绝对开度，然后用来计算卷膜时长 */
    abs_open_val = (film_pcb.motor_dir[i] == Film_Forward) ? (film_pcb.run_open_val[i] - film_pcb.last_open_val[i]) \
    : (film_pcb.last_open_val[i] - film_pcb.run_open_val[i]);
    film_pcb.run_roll_time[i] = abs_open_val * 0.01 * film_pcb.total_roll_time[i] + 0.5;
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor run roll time is %dS\n", i + 1, film_pcb.run_roll_time[i]));
  FILM_TRA_BLOCK_END

  return Film_OK;
}

film_u8 Film_Check_Reset_Roll_Flag(void)
{
  film_u8 reset_flag[MOTOR_CHANNEL];
  film_u8 i;

  Film_MEM_Get_Flag(FILM_MEM_RE_OK_FLAG_BASE_ADDR, &reset_flag[0]);

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (reset_flag[i] == FILM_MEM_SET_FLAG) continue;
    //Film_Load_Reset_Motor_Handle_CH(i + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor did not reset its route! [Film_Check_Reset_Roll_Flag]\n", i + 1));
    return 0;
  FILM_TRA_BLOCK_END
  return 1;
}

film_u8 Film_Monitor_Sig(void)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (film_pcb.run_motor_sta_flag[i] != FILM_SET_FLAG) return 0;
  FILM_TRA_BLOCK_END

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    film_pcb.run_motor_sta_flag[i] = FILM_RESET_FLAG;
  FILM_TRA_BLOCK_END
  return 1;
}
/********************************************************************************************************************/


/******************************************************与电流检测机制有关函数***********************************************************/
/* 初始化电机电流检测 */
void Film_Motor_EleCurrent_Init(void)
{
  film_u8 cur_tmp[2], cur_flag, cur_lmt;
  film_u16 saved_cur;
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    /* 检查是否保存过电流值 */
    if (Film_MEM_Get_Flag_CH(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, &cur_flag, (i + 1)))
    {
      film_pcb.ele_cur_sta[i] = Film_EleCur_Exp;
      FM_PRINT((FM_PrintDBG_Buf, "The current storage of the %d th motor is abnormal! [Film_Motor_EleCurrent_Init]\n", i + 1));
    }
    else if (cur_flag != FILM_MEM_SET_FLAG)  
    {
      film_pcb.ele_cur_sta[i] = Film_EleCur_UnInit;
      FM_PRINT((FM_PrintDBG_Buf, "The current value of the %d th motor is null [Film_Motor_EleCurrent_Init]\n", i + 1));
    }
    else 
      film_pcb.ele_cur_sta[i] = Film_EleCur_OK;

    if (film_pcb.ele_cur_sta[i] != Film_EleCur_OK) continue;
    /* 读取电流阈值，这里可以容错，允许储存错误 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_ELE_CUR_LMT_BASE_ADDR, &cur_lmt, (i + 1))) cur_lmt = 20;
    if (cur_lmt < 1 || cur_lmt > 100) cur_lmt = 20; //赋值默认值，2.0倍电流阈值 
    FM_PRINT((FM_PrintDBG_Buf, "The current threshold of the %d th motor is %.1lf times [Film_Motor_EleCurrent_Init]\n", i + 1, (film_double)cur_lmt * 0.1));
    /* 读取电流值 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_ELE_CUR_BASE_ADDR, &cur_tmp[0], (i + 1)))
      saved_cur = FILM_DEFAULT_CUR_VALUE;
    else
      Film_u8Tou16(&cur_tmp[0], &saved_cur, 2);
    /* 计算出过流阈值 */
    film_pcb.over_ele_cur[i] = (film_double)saved_cur * ((film_double)cur_lmt * 0.1);
    FM_PRINT((FM_PrintDBG_Buf, "The maximum current value of the %d th motor is %lf mA\n", i + 1, film_pcb.over_ele_cur[i]));
  FILM_TRA_BLOCK_END
}

void Film_Detect_OverCur(void)
{
  film_u8 i;
  
  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (film_pcb.ctrl_timing_num[i] < FILM_DETECT_START_TIME) continue;
    if (film_pcb.ele_cur_sta[i] > Film_EleCur_OK) continue;

    film_pcb.run_ele_cur[i] = Film_Read_Analog_Ele_Current_CH(i + 1);
    if (film_pcb.run_ele_cur[i] < film_pcb.over_ele_cur[i]) 
    {
      film_pcb.over_ele_cur_tim[i] = film_pcb.ctrl_timing_num[i];
      continue;
    }
    if (!film_pcb.over_ele_cur_tim[i]) 
      film_pcb.over_ele_cur_tim[i] = film_pcb.ctrl_timing_num[i];
    
    if (film_pcb.ctrl_timing_num[i] < film_pcb.over_ele_cur_tim[i] + 3) continue;

    Film_Finish_Rolling(i + 1);
    Film_Set_Exp_Status_CH(i + 1, Film_M_OverEleCur);
    film_pcb.ele_cur_sta[i] = Film_EleCur_Over; //电流状态也同步更新，这样下次遍历就会过滤该电机
    Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor over current and it is : %dmA [Film_Detect_OverCur]\n", i + 1, film_pcb.run_ele_cur[i]));
  FILM_TRA_BLOCK_END
}

void Film_Detect_LowerCur(void)
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
    film_pcb.run_motor_exp_sta[i] = Film_M_Run_Exp;
    film_pcb.run_motor_sta_flag[i] = FILM_SET_FLAG;
  FILM_TRA_BLOCK_END
}

/* 重置行程中，采集电流 */
void Film_Collect_Ele_Cur(void)
{
  static film_u32 clt_tim_tmp = 0;
  film_u32 clt_ele_cur_tmp;
  film_u8 i;

  if (clt_tim_tmp + FILM_CUR_COLLECT_FREQ > film_pcb.ctrl_com_timing_num) return;  //每采集周期才采集一次

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    clt_ele_cur_tmp = Film_Read_Analog_Ele_Current_CH(i + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor clt_ele_cur_tmp is : %dmA [Film_Collect_Ele_Cur]\n", i + 1, clt_ele_cur_tmp));
    if (clt_ele_cur_tmp < FILM_DEFAULT_LOW_CUR_VALUE) continue;
    film_pcb.clt_ele_cur[i] += clt_ele_cur_tmp;
    film_pcb.clt_ele_cur_num[i]++;
  FILM_TRA_BLOCK_END

  clt_tim_tmp = film_pcb.ctrl_com_timing_num;
}

/* 计算和保存电流值 */
void Film_Cal_Save_Ele_Cur(void)
{
  film_u8 cur_tmp[2];
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    film_pcb.clt_ele_cur[i] /= film_pcb.clt_ele_cur_num[i];
    Film_u16Tou8(&film_pcb.clt_ele_cur[i], &cur_tmp[0], 1);
    Film_MEM_Save_Param_CH(FILM_MEM_ELE_CUR_BASE_ADDR, &cur_tmp[0], i + 1);
    Film_MEM_Set_Flag_CH(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_SET_MODE);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor collect current is : %dmA [Film_Cal_Save_Ele_Cur]\n", i + 1, film_pcb.clt_ele_cur[i]));
  FILM_TRA_BLOCK_END
}
/***************************************************************************************************************/

/****************************************************异常检测相关函数********************************************/

void Film_Detect_Opening(void)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (film_pcb.run_open_val[i] == FILM_ALL_CLOSE || film_pcb.run_open_val[i] == FILM_ALL_OPEN) continue;
    if (film_pcb.ctrl_timing_num[i] < film_pcb.run_roll_time[i]) continue;
    Film_Finish_Rolling(i + 1);
    film_pcb.run_motor_exp_sta[i] = Film_M_OK;
    film_pcb.run_motor_sta_flag[i] = FILM_SET_FLAG;
    film_pcb.last_open_val[i] = film_pcb.run_open_val[i];
    film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
    Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reached opening... [Film_Detect_Opening]\n", i + 1));    
  FILM_TRA_BLOCK_END
}

void Film_Detect_Limit(film_m_act act)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    /* 如果卷膜时间少于检测限位开始等待时间，不进行检测 */
    if (film_pcb.ctrl_timing_num[i] < FILM_DETECT_START_TIME) continue;

    film_pcb.detect_m_overtime_flag = 1;  //允许后面的电机超时机制检测

    film_pcb.run_ele_cur[i] = Film_Read_Analog_Ele_Current_CH(i + 1);
    /* 如果读取的电流值大于最低电流阈值，正常 */
    if (film_pcb.run_ele_cur[i] > FILM_LOWER_CUR_VALUE) 
    {
      film_pcb.lower_ele_cur_tim[i] = film_pcb.ctrl_timing_num[i];
      continue;
    }
    /* 如果读取的电流值小于最低电流值，开始计时判断是否真的小于最低电流值 */
    if (!film_pcb.lower_ele_cur_tim[i])
      film_pcb.lower_ele_cur_tim[i] = film_pcb.ctrl_timing_num[i];

    if (film_pcb.ctrl_timing_num[i] < (film_pcb.lower_ele_cur_tim[i] + 3)) continue;

    /* 如果检测到电流低于最低电流阈值3S钟*/
    if (act == FILM_RESET)
    {
      if (!film_pcb.second_stage_flag)
        FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach down limit...(RESET) [Film_Detect_Limit]\n", i + 1));
      else FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach up limit...(RESET) [Film_Detect_Limit]\n", i + 1));
      film_pcb.run_open_val[i] = (film_pcb.second_stage_flag) ? FILM_ALL_OPEN : FILM_ALL_CLOSE;   
    }  
    else
    {
      /* 全开和全关不需要校正 */
      if (film_pcb.run_open_val[i] == FILM_ALL_CLOSE || film_pcb.run_open_val[i] == FILM_ALL_OPEN) 
      {
        if (film_pcb.run_open_val[i] == FILM_ALL_CLOSE)
          FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach down limit... [Film_Detect_Limit]\n", i + 1));
        else
          FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach up limit... [Film_Detect_Limit]\n", i + 1));
        continue;
      }
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor opening error, need to correct the opening [Film_Detect_Limit]\n", i + 1));
      film_pcb.adjust_open_flag[i] = FILM_SET_FLAG;
    }
    film_pcb.run_motor_exp_sta[i] = Film_M_Wait_Chk;
    film_pcb.run_motor_sta_flag[i] = FILM_SET_FLAG;
  FILM_TRA_BLOCK_END
}

/*
 @brief   : 卷膜超时处理。如果在卷膜过程中电流小于正常值，说明异常；如果卷膜时间超过最大时间阈值，也是异常
 @para    : 当前动作
 @return  : true or false
 */
film_err Film_Detect_Motor_OverTime(void)
{
  film_u8 i;

  if (!film_pcb.detect_m_overtime_flag) return Film_OK;
  film_pcb.detect_m_overtime_flag = 0;

  /* 电机异常机制，原来的函数处理还没采用，后面再看 */
  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (film_pcb.ctrl_timing_num[i] < FILM_ROLL_OVERTIME) continue;
    Film_Finish_Rolling(i + 1);
    film_pcb.run_motor_exp_sta[i] = (film_pcb.motor_dir[i] == Film_Forward) ? Film_M_Up_Limit_Exp : Film_M_Down_Limit_Exp;
    film_pcb.run_motor_sta_flag[i] = FILM_SET_FLAG;
    if (Film_Clear_All_Opening_CH(i + 1)) return Film_MEM_Exp;
    if (Film_Clear_Reset_OK_Flag_CH(i + 1)) return Film_MEM_Exp;
    FM_PRINT((FM_PrintDBG_Buf, "The %dth of motor overtime! [Film_Detect_Motor_OverTime]\n", i + 1));
  FILM_TRA_BLOCK_END

  return Film_OK;
}

/* 校验检测电机是否真的到达了限位，方法是往反方向走3S，同时检测电流值 */
void Film_Verify_Motor_Limit(film_m_act act)
{
  film_u32 chk_tim_num = 0;
  film_u8 i;

  film_pcb.lower_ele_cur_m_channel = 0;
  film_pcb.ctrl_com_timing_num = 0;

  FILM_TRA_MOTOR(i, film_pcb.m_channel)
    if (film_pcb.run_motor_exp_sta[i] != Film_M_Wait_Chk) continue;
    FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor prepare check limit... [Film_Verify_Motor_Limit]\n", i + 1));
    Film_Finish_Rolling(i + 1);
    film_pcb.lower_ele_cur_m_channel |= (1 << i);
    film_pcb.motor_dir[i] = (film_pcb.motor_dir[i] == Film_Forward) ? Film_Reversal : Film_Forward;
    Film_Motor_Start_Run();
    film_pcb.lower_ele_cur_tim[i] = 0;
  FILM_TRA_BLOCK_END

  if (!film_pcb.lower_ele_cur_m_channel) return;

  while (1)
  {
    Film_Detect_LowerCur();
    FILM_TRA_MOTOR(i, film_pcb.lower_ele_cur_m_channel)
      if (film_pcb.run_motor_exp_sta[i] != Film_M_Run_Exp) continue;
      FM_PRINT((FM_PrintDBG_Buf, "%dth motor exception! [Film_Check_Motor_Limit]\n", i + 1));
    FILM_TRA_BLOCK_END

    if (film_pcb.ctrl_com_timing_num < chk_tim_num + 3) continue;
    
    FILM_TRA_MOTOR(i, film_pcb.lower_ele_cur_m_channel)
      Film_Finish_Rolling(i + 1);
      if (film_pcb.run_motor_exp_sta[i] == Film_M_Run_Exp) continue; 
      film_pcb.run_motor_exp_sta[i] = Film_M_OK;
      FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor check limit success... [Film_Check_Motor_Limit]\n", i + 1));
      if (act == FILM_RESET)
      {
        if (!film_pcb.second_stage_flag) continue;
        film_pcb.run_open_val[i] = FILM_ALL_OPEN;
        film_pcb.last_open_val[i] = FILM_ALL_OPEN;
        film_pcb.rt_open_val[i] = FILM_ALL_OPEN;
        Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_SET_MODE);
      }     
    FILM_TRA_BLOCK_END
    break;
  }
}
/*********************************************************************************************************************/

/*
 *重置行程参数配置。在重置行程前，挂载需要重置行程的电机
 */
film_err Film_Motor_Start_Task(film_u8 *ch_buf, film_u8 ch_num, film_m_act act)
{
  film_u8 i;

  if (ch_num > MOTOR_CHANNEL || !ch_num) return Film_CH_Err; /* 请求挂载的电机数目超过设置上限 */

  film_pcb.m_channel = 0;
  for (i = 0; i < ch_num; i++)
  {
    film_pcb.m_channel |= (1 << (ch_buf[i] - 1)); /* 挂载需要运动的电机 */
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor has been loaded...[Film_Motor_Start_Task]\n", ch_buf[i]));
    if (act != FILM_RESET) continue;
    /* 清除重置完成标志位 */
    if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, ch_buf[i], FILM_MEM_FLAG_RESET_MODE)) return Film_MEM_Exp; 
  }
  /* 开度卷膜需要设置的参数 */
  if (act == FILM_ROLL)
  {
  #if (USE_RESET_ROLL_CHECK)
    if (!Film_Check_Reset_Roll_Flag())/* 没有重置过行程，就退出卷膜，暂时如此，减少开发负担 */
    {
      FM_PRINT((FM_PrintDBG_Buf, "Motor has not reset route ! [Film_Motor_Start_Task]\n"));   
      return Film_No_Reset; 
    }  
  #endif
    /* 储存挂载电机的运行开度，并读取上一次开度值*/
    FILM_TRA_MOTOR(i, film_pcb.m_channel)
      if (Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1)) return Film_MEM_Exp;
      if (Film_MEM_Read_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.last_open_val[i], i + 1)) return Film_MEM_Exp;
      FM_PRINT((FM_PrintDBG_Buf, "The run open value of the %dth motor is %d%%\n", i+1 , film_pcb.run_open_val[i]));
      FM_PRINT((FM_PrintDBG_Buf, "The last open value of the %dth motor is %d%%\n", i+1 , film_pcb.last_open_val[i]));
    FILM_TRA_BLOCK_END
    /* 计算出挂载的电机需要的卷膜时长 */
    if (Film_Calculate_Roll_Time()) return Film_MEM_Exp;
  }

  if (act == FILM_F_ROLL)
  {
    FILM_TRA_MOTOR(i, film_pcb.m_channel)
      if (Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1)) return Film_MEM_Exp;
      FM_PRINT((FM_PrintDBG_Buf, "The force run open value of the %dth motor is %d%%\n", i+1 , film_pcb.run_open_val[i]));
    FILM_TRA_BLOCK_END    
  }

  /* 设置电机重置行程的方向 */
  Film_Set_Motor_Dir(act);
    /* 初始化电流机制 */
  Film_Motor_EleCurrent_Init();
    /* 挂载电机开始运动 */
  Film_Motor_Start_Run();

  film_pcb.ctrl_com_timing_num = 0;

  return Film_OK;
}

film_err Film_Motor_Monitor_Task(film_m_act act)
{
  film_err sta;

  while (1)
  { 
    /* 开度检测 */
    if (act == FILM_ROLL) Film_Detect_Opening();
    /* 限位检测 */
    Film_Detect_Limit(act);
    /* 超时检测 */
    sta = Film_Detect_Motor_OverTime();
    if (sta) return sta;

    Film_Detect_OverCur();

    Film_Listening_Task();

    if (act == FILM_RESET)
    {
      if (!film_pcb.second_stage_flag)
      {
        if (!Film_Monitor_Sig()) continue;
        /* 检查是否到达了限位，将电机异常的剔除 */
        Film_Verify_Motor_Limit(FILM_RESET);

        Film_Set_Motor_Dir(act); //换向
        Film_Motor_Start_Run();
        film_pcb.second_stage_flag = 1;
      }
      Film_Collect_Ele_Cur();
    }

    if (!Film_Monitor_Sig()) continue;

    /* 保存运动时长 */
    if (act == FILM_RESET)
      Film_Save_ResetRoll_Time();

    Film_Verify_Motor_Limit(FILM_RESET);
    break;
  }
  return Film_OK;
}

film_err Film_Motor_End_Task(film_m_act act)
{
  /* 保存采集的电流值 */
  if (act == FILM_RESET)
    Film_Cal_Save_Ele_Cur();
  /* 保存采集的电压值 */

  /* 保存三个开度 */
  Film_Save_Open_Val();

  film_pcb.second_stage_flag = 0;
}
/*********************************************************************************************/

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
