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
void Film_Motor_Start_Task(void);
void Film_Motor_Monitor_Task(void);
void Film_Motor_End_Task(void);
film_u8 Film_Motor_Channel_Sig(void);
void Film_Finish_Rolling_CH(film_u8 ch);
void Film_Channel_Reset(void);
void Film_Load_Motor_CH(film_u8 ch, film_m_act act);
void Film_Set_Motor_Status_CH(film_u8 ch, film_m_sta sta);

film_u32 Film_Select_Obj_Channel(film_m_act act);
void Film_Set_Obj_Channel(film_u32 src_channel, film_m_act act);

void Film_Store_Exp_Handler_CH(film_u8 ch, film_m_act act);

/* 一些数据处理函数声明 */
void Film_u8Tou16(film_u8 *src_buf, film_u16 *obj_buf, film_u32 src_len);
void Film_u16Tou8(film_u16 *src_buf, film_u8 *obj_buf, film_u32 src_len);

film_u8 ASSERT_CHANNEL(film_u8 ch)
{
  if (!ch || ch > MOTOR_CHANNEL) return 0;
  return 1;
}

void Film_Param_Init(void)
{
  film_u8 exp_open_flag;
  film_u8 i;
  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    Film_MEM_Get_Flag_CH(FILM_MEM_EXP_OPEN_FLAG_BASE_ADDR, &exp_open_flag, i + 1);
    (exp_open_flag == FILM_SET_FLAG) ? Film_MEM_Read_Param_CH(FILM_MEM_EXP_TEMP_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) :
                      Film_MEM_Read_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1);

    film_pcb.run_motor_exp_sta[i] = Film_M_IDIE;
    film_pcb.clt_ele_cur_num[i] = 0;
    film_pcb.lower_ele_cur_tim[i] = 0;
    film_pcb.ctrl_timing_num[i] = 0;
    film_pcb.second_stage_flag[i] = 0;
    film_pcb.reset_m_dir_flag[i] = 0;
    film_pcb.m_force_stop_flag[i] = 0;
    film_pcb.auto_open_roll_flag[i] = 0;
  }
  film_pcb.m_reset_channel = 0;
  film_pcb.m_open_channel = 0;
  film_pcb.m_f_open_channel = 0;
  film_pcb.ctrl_com_timing_num = 0;
  film_pcb.detect_m_overtime_flag = 0;
  film_pcb.new_task_request_flag = 0;
}

film_err Film_Set_Open_Value(film_u8 *ch_buf, film_u8 ch_num, film_u8 *open_buf)
{
  film_u8 i;
  for (i = 0; i < ch_num; i++)
  {
    /* 如果当前路数正在强制卷膜，不去覆盖强制卷膜的开度！ */
    if (film_pcb.m_f_open_channel & (1 << (ch_buf[i] - 1)))
      continue;

    if (open_buf[i] > FILM_ALL_OPEN) continue;
    film_pcb.run_open_val[ch_buf[i] - 1] = open_buf[i];
    if (Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[ch_buf[i] - 1], ch_buf[i]))
      return Film_MEM_Exp;
  }
  return Film_OK;
}

/**
 @brief   : 三种运动模式通道空闲时，初始化挂载电机通道。
 @param   : 1.要挂载的通道数缓存（1 ~ MOTOR_CHANNEL）
            2.要挂载的电机数量
            3.要进行的运动模式，参考 film_m_act
 @return  : 参考 film_err
 */
film_err Film_Load_Init(film_u8 *ch_buf, film_u8 ch_num, film_m_act act)
{
  film_u8 i;
  if (ch_num > MOTOR_CHANNEL || !ch_num) return Film_CH_Err; /* 请求挂载的电机数目超过设置上限 */

  Film_Channel_Reset();
  /* 根据挂载的运动类型，挂载到相应的通道上 */
  for (i = 0; i < ch_num; i++)
    Film_Load_Motor_CH(ch_buf[i], act);    
}

/**
 @brief   : 电机运行任务，适用于三种运动模式通道都是空闲的情况下的卷膜运动。
            主要进行了挂载电机到相应通道，根据电机的运动模式设置处理一些卷膜前的准备工作。
            进行卷膜监测，卷膜完成处理。
 @param   : 1.要挂载的通道数缓存（1 ~ MOTOR_CHANNEL）
            2.要挂载的电机数量
            3.要进行的运动模式，参考 film_m_act
 @return  : 参考 film_err
 */
film_err Film_Motor_Run_Task(film_u8 *ch_buf, film_u8 ch_num, film_m_act act)
{
  if (Film_Load_Init(&ch_buf[0], ch_num, act) == Film_CH_Err) return Film_CH_Err;
  Film_Motor_Start_Task();
  Film_Motor_Monitor_Task();
  Film_Motor_End_Task();
  return Film_OK;
}

/* 更新切换电机运行任务 */
void Film_Switch_Task(void)
{
  if (!film_pcb.new_task_request_flag) return;
  film_pcb.new_task_request_flag = FILM_RESET_FLAG;

  FM_PRINT((FM_PrintDBG_Buf, "the motor prepare to start new task...[Film_Switch_Task]\n"));

  Film_Motor_Start_Task();
  Film_Motor_Monitor_Task();
  Film_Motor_End_Task(); 
}

/* 开机后检测是否有电机在开度卷膜未完成时断过电，恢复这些未完成的电机运动 */
void Film_Check_Exp_Open_Task(void)
{
  film_u8 exp_ch_buf[MOTOR_CHANNEL];
  film_u8 ch_num = 0;
  film_u8 exp_open_val = FILM_TEMP_OPEN;
  film_u8 i;

  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    if (Film_MEM_Read_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) ||
      Film_MEM_Read_Param_CH(FILM_MEM_RT_OPEN_BASE_ADDR, &film_pcb.rt_open_val[i], i + 1))
      {
        #if (!FILM_LOWER_LEVEL_RESET_MODE)
        Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);
        #endif
        continue;
      }
        

    if (!(film_pcb.rt_open_val[i] - film_pcb.run_open_val[i])) continue;
    /* 如果有差值而不是相等 */
    FM_PRINT((FM_PrintDBG_Buf, "The last opening task of the %dth motor is not completed! [Film_Check_Exp_Open_Task]\n", i + 1));

    if (Film_MEM_Save_Param_CH(FILM_MEM_EXP_TEMP_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) ||
    Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &exp_open_val, i + 1) ||
    Film_MEM_Set_Flag_CH(FILM_MEM_EXP_OPEN_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_SET_MODE)) 
      continue;

    exp_ch_buf[ch_num] = i + 1;
    ch_num++;
  }
  if (!ch_num) goto clear_flag;

  Film_Motor_Run_Task(&exp_ch_buf[0], ch_num, FILM_F_ROLL);

  ch_num = 0;
  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    if (film_pcb.run_motor_exp_sta[i] != Film_M_F_Open_OK) continue;
    if (Film_MEM_Read_Param_CH(FILM_MEM_EXP_TEMP_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) ||
        Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1)) 
    {
      #if (!FILM_LOWER_LEVEL_RESET_MODE)
      Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);
      #endif
      continue;
    }

    exp_ch_buf[ch_num] = i + 1;
    ch_num++;
  }
  if (!ch_num) goto clear_flag;

  Film_Motor_Run_Task(&exp_ch_buf[0], ch_num, FILM_ROLL);
  goto clear_flag;
  
  clear_flag :
  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    Film_MEM_Set_Flag_CH(FILM_MEM_EXP_OPEN_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);
  }
}

void Film_Auto_Open_Task(void)
{
  film_u8 auto_open_ch_buf[MOTOR_CHANNEL];
  film_u8 ch_num = 0;
  film_u8 i;

  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    if (!film_pcb.auto_open_roll_flag[i]) continue;
    film_pcb.auto_open_roll_flag[i] = FILM_RESET_FLAG;

    if (Film_MEM_Read_Param_CH(FILM_MEM_AUTO_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) ||
        Film_MEM_Save_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1))
    {
      #if (!FILM_LOWER_LEVEL_RESET_MODE)
      Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);
      #endif
      continue;
    }

    FM_PRINT((FM_PrintDBG_Buf, "the %dth motor prepare auto-open task...[Film_Auto_Open_Task]\n", i + 1));
    /* 挂载电机 */
    auto_open_ch_buf[ch_num] = i + 1;
    ch_num++;
  }

  Film_Motor_Run_Task(&auto_open_ch_buf[0], ch_num, FILM_ROLL); 
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
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Start_Timming_CH]\n"));
    return;
  }
  film_pcb.ctrl_timing_en_sig |= (1 << (ch - 1));
  film_pcb.ctrl_timing_num[ch - 1] = 0;
}

/* 卷膜停止工作，停止控制心跳计数 */
void Film_Stop_Timming_CH(film_u8 ch)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Stop_Timming_CH]\n"));
    return;
  }

  film_pcb.ctrl_timing_en_sig &= ~(1 << (ch - 1));
  film_pcb.ctrl_timing_num[ch - 1] = 0;
}

/**********************************************与服务器下发参数设置有关函数****************************************************/
film_u8 Film_Read_RT_Opening_CH(film_u8 ch) 
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Read_RT_Opening_CH]\n"));
    return 0xFF;
  }
  return (film_pcb.rt_open_val[ch - 1]); 
}

film_u8 Film_Read_Last_Opening_CH(film_u8 ch) 
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Read_Last_Opening_CH]\n"));
    return Film_M_CH_Err;
  } 
  return (film_pcb.last_open_val[ch - 1]); 
}
film_u8 Film_Read_Run_Opening_CH(film_u8 ch) 
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Read_Run_Opening_CH]\n"));
    return Film_M_CH_Err;
  }
  return (film_pcb.run_open_val[ch - 1]); 
}

film_m_sta Film_Read_Motor_Status_CH(film_u8 ch)
{ 
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Read_Motor_Status_CH]\n"));
    return Film_M_CH_Err;
  }
  return (film_pcb.run_motor_exp_sta[ch - 1]); 
}

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

void Film_Set_Force_Stop(film_u8 *ch_buf, film_u8 ch_num)
{
  film_u8 i;

  if (ch_num > MOTOR_CHANNEL || !ch_num) return /* Film_CH_Err */;

  if (Film_Motor_Channel_Sig())
  {
    for (i = 0; i < ch_num; i++)
    {
      Film_Set_Motor_Status_CH(ch_buf[i], Film_M_F_Stop_OK);
      Film_Status_Msg_Delivery_Task(ch_buf[i], film_pcb.run_motor_exp_sta[ch_buf[i] - 1]);
    }
    return;
  } 

  for (i = 0; i < ch_num; i++)
    film_pcb.m_force_stop_flag[ch_buf[i] - 1] = FILM_SET_FLAG;
}

film_err Film_New_Task_Handler(film_u8 *ch_buf, film_u8 ch_num, film_m_act act)
{
  film_u32 obj_channel;
  film_u8 i, j;

  if (Film_Motor_Channel_Sig()) return Film_OK;

  FM_PRINT((FM_PrintDBG_Buf, "test1 [Film_New_Task_Handler]\n"));

  obj_channel = Film_Select_Obj_Channel(act);

  for (i = 0; i < ch_num; i++)
  {
    /* 如果该路电机正在强制卷膜，其他状态不能替换强制卷膜，拒绝切换 */
    if (film_pcb.m_f_open_channel & (1 << (ch_buf[i] - 1)))
    {
      FM_PRINT((FM_PrintDBG_Buf, "Force open channel busy! [Film_New_Task_Handler]\n"));
      continue;
    }

    if (act == FILM_F_ROLL)
    {
      film_pcb.m_reset_channel &= ~(1 << (ch_buf[i] - 1));
      film_pcb.m_open_channel &= ~(1 << (ch_buf[i] - 1));
    }
    /* 如果该路电机正在重置行程，当前切换任务是开度卷膜时，拒绝切换 */
    if (act == FILM_ROLL)
    {
      if (film_pcb.m_reset_channel & (1 << (ch_buf[i] - 1)))
        if (film_pcb.run_open_val[ch_buf[i] - 1] != FILM_ALL_CLOSE && film_pcb.run_open_val[ch_buf[i] - 1] != FILM_ALL_OPEN) continue;

      film_pcb.m_f_open_channel &= ~(1 << (ch_buf[i] - 1));
      film_pcb.m_reset_channel &= ~(1 << (ch_buf[i] - 1));      
    }
    else if (act == FILM_RESET)
    {
      if (film_pcb.m_reset_channel & (1 << (ch_buf[i] - 1)))
      {
        FM_PRINT((FM_PrintDBG_Buf, "reset open channel busy! [Film_New_Task_Handler]\n"));
        continue;
      }
      film_pcb.m_f_open_channel &= ~(1 << (ch_buf[i] - 1));
      film_pcb.m_open_channel &= ~(1 << (ch_buf[i] - 1));
    }

    obj_channel |= (1 << (ch_buf[i] - 1));
  }
  Film_Set_Obj_Channel(obj_channel, act);
 
  FILM_TRA_MOTOR_ACT(i, j, obj_channel)
    if (i == FILM_ROLL)
    {
      film_pcb.last_open_val[j] = film_pcb.rt_open_val[j];
      if (Film_MEM_Save_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.last_open_val[j], j + 1))
        Film_Store_Exp_Handler_CH(j + 1, FILM_ROLL);      
    }

    /* 写这段代码的原因是：
     * 举个例子
     * 1.已经重置完成过
     * 2.命令去强制卷膜，start_task里会判断等强制卷膜完成后需要恢复重置完成标志位，然后清除重置完成标志位
     * 3.如果本次强制卷膜顺利完成，就会恢复重置完成标志位
     * 4.但是，如果卷膜中途再次更新切换新任务，就会再次进入2步骤，但此时因为重置卷膜标志位已经清除了，所以该次卷膜会认为卷膜完成后不用恢复重置完成标志位
     * 会造成就算强制卷膜完成，下一次需要开度卷膜时却要重置行程。
     * 5.解决办法就是在切换新任务，也就是本函数，判断是否需要恢复重置行程标志位，如果需要，先在这里恢复，然后在2步骤就能再次正确判断等强制卷膜完成后需要恢复
     * 重置行程标志位了。
     */
    if (i == FILM_F_ROLL)
    {
      if (film_pcb.re_ok_before_f_roll_flag[j])
      {
        if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, j + 1, FILM_MEM_FLAG_SET_MODE))
          Film_Store_Exp_Handler_CH(j + 1, FILM_F_ROLL);
      }
    }

    film_pcb.auto_open_roll_flag[j] = FILM_RESET_FLAG;
    Film_Finish_Rolling_CH(j + 1);
  FILM_TRA_ACT_BLOCK_END

  film_pcb.new_task_request_flag = FILM_SET_FLAG;
  return Film_New_Task;
}

/***************************************************************************************************************************/

/****************************************与电机运动有关函数***************************************/

void Film_Set_Motor_Status_CH(film_u8 ch, film_m_sta sta)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Set_Motor_Status_CH]\n"));
    return;
  }
  film_pcb.run_motor_exp_sta[ch - 1] = sta;
}

/* 
 @brief   : 复位电机运行模式和清空挂起所有电机通道
 */
void Film_Channel_Reset(void)
{
  film_u8 i;
  for (i = 0; i < MOTOR_CHANNEL; i++)
    film_pcb.m_run_mode[i] = FILM_IDIE;

  film_pcb.m_reset_channel = 0;
  film_pcb.m_open_channel = 0;
  film_pcb.m_f_open_channel = 0;
}

/**
 @brief   : 挂载电机到相应的运动通道，并解除另外两个通道上该电机的路数。
 @param   : 1.路数（1 ~ MOTOR_CHANNEL）
            2.参考 film_m_act
 @return  : 无
 */
void Film_Load_Motor_CH(film_u8 ch, film_m_act act)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Load_Motor_CH]\n"));
    return;
  }
  /* 如果该路电机还在强制卷膜，禁止切换到重置行程或者强制卷膜 */
  if (film_pcb.m_run_mode[ch - 1] == FILM_F_ROLL) 
  {
    FM_PRINT((FM_PrintDBG_Buf, "The %dth of motor is running force-roll, reject load!\n", ch));
    return;
  } 

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

  /* 更新当前运动模式 */
  film_pcb.m_run_mode[ch - 1] = act;
  FM_PRINT((FM_PrintDBG_Buf, "%dth motor has been loaded...[Film_Motor_Run_Task]\n", ch));
}

/* 挂起电机和复位运动模式 */
void Film_Pending_Motor_CH(film_u8 ch, film_m_act act)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Pending_Motor_CH]\n"));
    return;
  }

  if (act == FILM_RESET)
    film_pcb.m_reset_channel &= ~(1 << (ch - 1));
  else if (act == FILM_ROLL)
    film_pcb.m_open_channel &= ~(1 << (ch - 1));
  else if (act == FILM_F_ROLL)
    film_pcb.m_f_open_channel &= ~(1 << (ch - 1));

  if (film_pcb.run_motor_exp_sta[ch - 1] != Film_M_Reset_OK)
    film_pcb.auto_open_roll_flag[ch - 1] = FILM_RESET_FLAG;

  film_pcb.m_run_mode[ch - 1] = FILM_IDIE; 
}

/**
 @brief   : 将一路电机从一个运动模式通道转移到另一个运动模式通道，当前只适用于源运动模式通道
            是开度卷膜的情况。
 @param   : 1.路数
            2.源运动模式通道
            3.目标运动模式通道
 @return  : 无
 */
void Film_Move_Load_Motor_CH(film_u8 ch, film_m_act src_act, film_m_act obj_act)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Move_Load_Motor_CH]\n"));
    return;
  }
  film_pcb.m_open_channel &= ~(1 << (ch - 1));
  if (obj_act == FILM_RESET) film_pcb.m_reset_channel |= (1 << (ch - 1));
  else if (obj_act == FILM_F_ROLL) film_pcb.m_f_open_channel |= (1 << (ch - 1));

  film_pcb.m_run_mode[ch - 1] = obj_act;
}

film_u32 Film_Select_Obj_Channel(film_m_act act)
{
  if (act == FILM_RESET) return film_pcb.m_reset_channel;
  else if (act == FILM_ROLL) return film_pcb.m_open_channel;
  else if (act == FILM_F_ROLL)return film_pcb.m_f_open_channel;
}

void Film_Set_Obj_Channel(film_u32 src_channel, film_m_act act)
{
  if (act == FILM_RESET)  film_pcb.m_reset_channel = src_channel;
  else if (act == FILM_ROLL)  film_pcb.m_open_channel = src_channel;
  else if (act == FILM_F_ROLL) film_pcb.m_f_open_channel = src_channel;  
}

/**
 @brief   : 根据电机运动模式设置相应的电机方向
 @param   : 运动模式
 @return  : 无
 */
void Film_Set_Motor_Dir(film_m_act act)
{
  film_u32 obj_channel;
  film_u8 i;

  obj_channel = Film_Select_Obj_Channel(act);

  FILM_TRA_MOTOR(i, obj_channel)
    switch (act)
    {
      case FILM_RESET : 
        film_pcb.motor_dir[i] = (!film_pcb.reset_m_dir_flag[i]) ? Film_Reversal : Film_Forward;
        film_pcb.reset_m_dir_flag[i] = ~film_pcb.reset_m_dir_flag[i];
        Film_Set_Motor_Status_CH(i + 1, Film_M_Reseting); 
      break;

      case FILM_ROLL : 
        if (film_pcb.rt_open_val[i] == FILM_UNKNOWN_OPEN)
          film_pcb.last_open_val[i] = (film_pcb.run_open_val[i] > 0) ? FILM_ALL_CLOSE : FILM_ALL_OPEN;
        Film_Set_Motor_Status_CH(i + 1, Film_M_Opening);

        if (film_pcb.last_open_val[i] < film_pcb.run_open_val[i])
          film_pcb.motor_dir[i] = Film_Forward;
        else if (film_pcb.last_open_val[i] > film_pcb.run_open_val[i])
          film_pcb.motor_dir[i] = Film_Reversal;
        else  /* 开度相同，挂起该电机，设置卷膜完成，投递消息 */
        {
          film_pcb.motor_dir[i] = Film_Stop; 
          Film_Pending_Motor_CH(i + 1, FILM_ROLL);
          Film_Set_Motor_Status_CH(i + 1, Film_M_Open_OK);
        }
      break;

      case FILM_F_ROLL :
        film_pcb.motor_dir[i] = (film_pcb.run_open_val[i]) ? Film_Forward : Film_Reversal;
        Film_Set_Motor_Status_CH(i + 1, Film_M_F_Opening);
      break; 
    }
    Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
  FILM_TRA_BLOCK_END
}

/* 取出被挂载的运动部件和他们的运行开度，上一次开度，计算出每个挂载的电机的方向 */
void Film_Motor_Start_Run(film_u32 obj_ch)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, obj_ch)
    film_pcb.m_force_stop_flag[i] = FILM_RESET_FLAG;
    Film_Ctrl_Motor_CH(i + 1, film_pcb.motor_dir[i]);
    if (film_pcb.motor_dir[i] == Film_Forward)
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Forward\n", i + 1));
    else
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Reversal\n", i + 1));

    Film_Start_Timming_CH(i + 1);
  FILM_TRA_BLOCK_END
}

void Film_Motor_Start_Run_CH(film_u8 ch)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Motor_Start_Run_CH]\n"));
    return;
  }
  film_pcb.m_force_stop_flag[ch - 1] = FILM_RESET_FLAG;
  Film_Ctrl_Motor_CH(ch, film_pcb.motor_dir[ch - 1]);
  if (film_pcb.motor_dir[ch - 1] == Film_Forward)
  FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Forward\n", ch));
  else
  FM_PRINT((FM_PrintDBG_Buf, "%dth motor start runing and direction is Reversal\n", ch));

  Film_Start_Timming_CH(ch);
}


void Film_Finish_Rolling_CH(film_u8 ch)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Finish_Rolling_CH_CH]\n"));
    return;
  }
  Film_DIR dir_tmp = Film_Stop;
  Film_Ctrl_Motor_CH(ch, dir_tmp);
  Film_Stop_Timming_CH(ch);
}
/***************************************************************************************************************/

/**************************************与电机流程控制有关函数*****************************************************/

/* 保存该路数的三个开度值到储存器 */
film_err Film_Save_Open_Val_CH(film_u8 ch)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Save_Open_Val_CH]\n"));
    return Film_CH_Err;
  }

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

  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Clear_All_Opening_CH]\n"));
    return Film_CH_Err;
  }

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
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Clear_Reset_OK_Flag_CH]\n"));
    return Film_CH_Err;
  }

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

/**
 @brief   : 根据实际要运动的开度计算出所需运动时间（S）
 @param   : 路数(1 ~ MOTOR_CHANNEL)
 @return  : 无 
 */
void Film_Calculate_Roll_Time_CH(film_u8 ch)
{
  film_u8 t_roll_time_tmp[2];  
  film_u8 abs_open_val;  /* 要运行的绝对开度值 */

  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Calculate_Roll_Time_CH]\n"));
    return;
  }
  
  /* 读取卷膜总时长 */
  if (Film_MEM_Read_Param_CH(FILM_MEM_ROLL_TIME_BASE_ADDR, &t_roll_time_tmp[0], ch))
  {
    Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, ch, FILM_MEM_FLAG_RESET_MODE);
    Film_Store_Exp_Handler_CH(ch, FILM_ROLL);
    return;
  }
  Film_u8Tou16(&t_roll_time_tmp[0], &film_pcb.total_roll_time[ch - 1], sizeof(t_roll_time_tmp));
  /* 得到绝对开度，然后用来计算卷膜时长 */
  abs_open_val = (film_pcb.motor_dir[ch - 1] == Film_Forward) ? (film_pcb.run_open_val[ch - 1] - film_pcb.last_open_val[ch - 1]) \
  : (film_pcb.last_open_val[ch - 1] - film_pcb.run_open_val[ch - 1]);
  film_pcb.run_roll_time[ch - 1] = (double)abs_open_val * 0.01 * (double)film_pcb.total_roll_time[ch - 1] + 0.5;
  FM_PRINT((FM_PrintDBG_Buf, "%dth motor run roll time is %dS\n", ch, film_pcb.run_roll_time[ch - 1]));
}

/**
 @brief   : 适用于开度卷膜前，检查所有挂载电机是否重置过行程。
            没有重置行程的根据开度值转移挂载到强制卷膜通道或者重置行程通道。
 @param   : 无
 @return  : 无
 */
void Film_Check_Reset_Roll_Flag(void)
{
  film_u8 reset_flag[MOTOR_CHANNEL];
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    Film_MEM_Get_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, &reset_flag[i], i + 1);
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
    if (!Film_MEM_Save_Param_CH(FILM_MEM_AUTO_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1))
      film_pcb.auto_open_roll_flag[i] = FILM_SET_FLAG;

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
  if (film_pcb.m_reset_channel || film_pcb.m_open_channel || film_pcb.m_f_open_channel) return 0;
  return 1;
}

void Film_Detect_Adjust_Open_Roll_CH(film_u8 ch)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Detect_Adjust_Open_Roll_CH]\n"));
    return;
  }

  film_pcb.last_open_val[ch - 1] = (film_pcb.motor_dir[ch - 1] == Film_Forward) ? FILM_ALL_CLOSE : FILM_ALL_OPEN;
  FM_PRINT((FM_PrintDBG_Buf, "The %dth motor opening error, need to adjust the opening [Film_Detect_Adjust_Open_Roll_CH]\n", ch));
  Film_Set_Motor_Status_CH(ch, Film_M_Opening);
  Film_Calculate_Roll_Time_CH(ch);
  Film_Motor_Start_Run_CH(ch);
}
/********************************************************************************************************************/


/******************************************************与电流检测机制有关函数***********************************************************/

void Film_Motor_Clear_EleCurrent(void)
{
  film_u8 i;
  for (i = 0; i < MOTOR_CHANNEL; i++)
    Film_MEM_Set_Flag_CH(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);   
}

/**
 @brief   : 根据保存的电流阈值，初始化计算卷膜允许的最大电流值，为后面的过流保护提供过流标准
 @param   : 无
 @return  : 无
 */
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
    /* 过流后的处理流程 */
    Film_Finish_Rolling_CH(j + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor over current and it is : %dmA [Film_Detect_OverCur]\n", j + 1, film_pcb.run_ele_cur[j]));
    Film_Set_Motor_Status_CH(j + 1, Film_M_OverEleCur);
    Film_Pending_Motor_CH(j + 1, (film_m_act)i);
    Film_Status_Msg_Delivery_Task(j + 1, film_pcb.run_motor_exp_sta[j]);
    /* 强制卷膜处理 */
    if (i == FILM_F_ROLL)
      #if (!FILM_LOWER_LEVEL_RESET_MODE)
      Film_Clear_Reset_OK_Flag_CH(j + 1);
      #endif
    /* 开度卷膜需要保存当前开度 */
    if (i != FILM_ROLL) continue;
    film_pcb.run_open_val[j] = film_pcb.rt_open_val[j];
    film_pcb.last_open_val[j] = film_pcb.rt_open_val[j];
    if (Film_Save_Open_Val_CH(j + 1))
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

    Film_Finish_Rolling_CH(i + 1);
    FM_PRINT((FM_PrintDBG_Buf, "%dth motor exception! [Film_Detect_LowerCur]\n", i + 1));
    film_pcb.run_open_val[i] = FILM_UNKNOWN_OPEN;
    film_pcb.rt_open_val[i] = FILM_UNKNOWN_OPEN; 
    film_pcb.last_open_val[i] = FILM_UNKNOWN_OPEN;
    #if (!FILM_LOWER_LEVEL_RESET_MODE)
    if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE) || Film_Save_Open_Val_CH(i + 1))
    {
      Film_Store_Exp_Handler_CH(i + 1, act);
      continue;
    }
    #else
    if (Film_Save_Open_Val_CH(i + 1))
    {
      Film_Store_Exp_Handler_CH(i + 1, act);
      continue;
    }
    #endif
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

  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Cal_Save_Ele_Cur_CH]\n"));
    return;
  }

  film_pcb.clt_ele_cur[ch - 1] /= film_pcb.clt_ele_cur_num[ch - 1];
  Film_u16Tou8(&film_pcb.clt_ele_cur[ch - 1], &cur_tmp[0], 1);
  Film_MEM_Save_Param_CH(FILM_MEM_ELE_CUR_BASE_ADDR, &cur_tmp[0], ch);
  Film_MEM_Set_Flag_CH(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, ch, FILM_MEM_FLAG_SET_MODE);
  FM_PRINT((FM_PrintDBG_Buf, "The %dth motor collect current is : %dmA [Film_Cal_Save_Ele_Cur_CH]\n", ch, film_pcb.clt_ele_cur[ch - 1]));
  film_pcb.clt_ele_cur_num[ch - 1] = 0;
  film_pcb.clt_ele_cur[ch - 1] = 0;
}
/***************************************************************************************************************/

/****************************************************异常检测相关函数********************************************/

/*储存异常处理，包含了挂起储存异常的电机，设置异常状态，投递状态 */
void Film_Store_Exp_Handler_CH(film_u8 ch, film_m_act act)
{
  if (!ASSERT_CHANNEL(ch))
  {
    FM_PRINT((FM_PrintDBG_Buf, "CH ERR! [Film_Cal_Save_Ele_Cur_CH]\n"));
    return;
  }
  Film_Finish_Rolling_CH(ch);
  Film_Pending_Motor_CH(ch, act);
  Film_Set_Motor_Status_CH(ch, Film_M_MEM_Exp);
  Film_Status_Msg_Delivery_Task(ch, film_pcb.run_motor_exp_sta[ch - 1]);
}

/* 检测是否某个或多个通道的电机被请求强制停止 */
void Film_Detect_Force_Stop(void)
{
  film_u32 obj_channel;
  film_u8 i, j;

  FILM_TRA_MOTOR_ACT(i, j, obj_channel)
    if (!film_pcb.m_force_stop_flag[j]) continue;
    film_pcb.m_force_stop_flag[j] = FILM_RESET_FLAG;

    Film_Finish_Rolling_CH(j + 1);
    Film_Pending_Motor_CH(j + 1, (film_m_act)i);
    Film_Set_Motor_Status_CH(j + 1, Film_M_F_Stop_OK);
    Film_Status_Msg_Delivery_Task(j + 1, film_pcb.run_motor_exp_sta[j]);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth motor has forced stop... [Film_Detect_Force_Stop]\n", j + 1));

    /* 保存开度、置位开度卷膜完成标志位 */
    if (i == FILM_ROLL)
    {
      film_pcb.last_open_val[j] = film_pcb.rt_open_val[j];
      film_pcb.run_open_val[j] = film_pcb.rt_open_val[j];
      if (Film_Save_Open_Val_CH(j + 1))
        Film_Store_Exp_Handler_CH(j + 1, FILM_ROLL);
    }
    else
    {
      film_pcb.last_open_val[j] = FILM_UNKNOWN_OPEN;
      film_pcb.rt_open_val[j] = FILM_UNKNOWN_OPEN;
      film_pcb.run_open_val[j] = FILM_UNKNOWN_OPEN;
      if (Film_Save_Open_Val_CH(j + 1))
      {
        (i == FILM_RESET) ? Film_Store_Exp_Handler_CH(j + 1, FILM_RESET) : Film_Store_Exp_Handler_CH(j + 1, FILM_F_ROLL);
      } 
    }

  FILM_TRA_ACT_BLOCK_END
}

/* 开度卷膜中，检测除了全关和全开之外的其他开度是否已经到达 */
void Film_Detect_Opening(void)
{
  film_u8 i;

  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)

    /* 实时开度计算 */
    if (film_pcb.motor_dir[i] == Film_Forward)
    {
      film_pcb.rt_open_val[i] = (((double)film_pcb.ctrl_timing_num[i] / (double)film_pcb.total_roll_time[i]) * 100) + film_pcb.last_open_val[i];
      if (film_pcb.rt_open_val[i] > film_pcb.run_open_val[i]) film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
    }
    else
    {
      film_pcb.rt_open_val[i] = film_pcb.last_open_val[i] - (((double)film_pcb.ctrl_timing_num[i] / (double)film_pcb.total_roll_time[i]) * 100);
      if (film_pcb.rt_open_val[i] < film_pcb.run_open_val[i]) film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
    }

    /* 开度到达验证 */
    if (film_pcb.run_open_val[i] == FILM_ALL_CLOSE || film_pcb.run_open_val[i] == FILM_ALL_OPEN) continue;
    if (film_pcb.ctrl_timing_num[i] < film_pcb.run_roll_time[i]) continue;
    Film_Finish_Rolling_CH(i + 1);
    film_pcb.last_open_val[i] = film_pcb.run_open_val[i];
    film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
    /* 完成开度卷膜后，保存三个开度、置位开度卷膜完成标志位 */
    if (Film_Save_Open_Val_CH(i + 1))
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
      if (film_pcb.motor_dir[j] == Film_Reversal)
        FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach down limit... [Film_Detect_Limit]\n", j + 1));
      else
        FM_PRINT((FM_PrintDBG_Buf, "The %dth motor reach up limit... [Film_Detect_Limit]\n", j + 1));        
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
    Film_Finish_Rolling_CH(j + 1);
    FM_PRINT((FM_PrintDBG_Buf, "The %dth of motor overtime! [Film_Detect_Motor_OverTime]\n", j + 1));
    #if (!FILM_LOWER_LEVEL_RESET_MODE)
    if (Film_Clear_All_Opening_CH(j + 1) || Film_Clear_Reset_OK_Flag_CH(j + 1))
    {
      Film_Store_Exp_Handler_CH(j + 1, (film_m_act)i);
      continue;
    }
    #else
    if (Film_Clear_All_Opening_CH(j + 1))
    {
      Film_Store_Exp_Handler_CH(j + 1, (film_m_act)i);
      continue;
    }
    #endif
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
  film_u8 recover_pos_flag[MOTOR_CHANNEL] = {0};
  film_u32 lower_ele_cur_tmp_channel = 0;
  film_u8 i;

  if (act == FILM_RESET) obj_channel = film_pcb.m_reset_channel;
  else if (act == FILM_ROLL) obj_channel = film_pcb.m_open_channel;
  else if (act == FILM_F_ROLL) obj_channel = film_pcb.m_f_open_channel;

  film_pcb.lower_ele_cur_m_channel = 0;

  FILM_TRA_MOTOR(i, obj_channel)
    if (film_pcb.run_motor_exp_sta[i] != Film_M_Wait_Chk) continue;
    FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor prepare check limit... [Film_Verify_Motor_Limit]\n", i + 1));
    Film_Finish_Rolling_CH(i + 1);
    film_pcb.lower_ele_cur_m_channel |= (1 << i);
    film_pcb.motor_dir[i] = (film_pcb.motor_dir[i] == Film_Forward) ? Film_Reversal : Film_Forward;
    film_pcb.lower_ele_cur_tim[i] = 0;
  FILM_TRA_BLOCK_END
  if (!film_pcb.lower_ele_cur_m_channel) return;
  
  lower_ele_cur_tmp_channel = film_pcb.lower_ele_cur_m_channel;
  Film_Motor_Start_Run(film_pcb.lower_ele_cur_m_channel);
  film_pcb.ctrl_com_timing_num = 0;

  while (1)
  {
    Film_Detect_LowerCur(act);
    if (film_pcb.ctrl_com_timing_num < (chk_tim_num + FILM_CHK_LIMIT_TIME)) continue;

    while (1)
    {
      FILM_TRA_MOTOR(i, lower_ele_cur_tmp_channel)
        if (recover_pos_flag[i] != FILM_SET_FLAG)
        {
          Film_Finish_Rolling_CH(i + 1);
          /* 然后再往相反方向走3S */
          film_pcb.motor_dir[i] = (film_pcb.motor_dir[i] == Film_Forward) ? Film_Reversal : Film_Forward;
          Film_Ctrl_Motor_CH(i + 1, film_pcb.motor_dir[i]);
          Film_Start_Timming_CH(i + 1);
          recover_pos_flag[i] = FILM_SET_FLAG;
        }

        if (film_pcb.ctrl_timing_num[i] < FILM_CHK_LIMIT_TIME) continue;
        Film_Finish_Rolling_CH(i + 1);
        lower_ele_cur_tmp_channel &= ~(1 << i);
      FILM_TRA_BLOCK_END

      if (!lower_ele_cur_tmp_channel) break;
    }

    FILM_TRA_MOTOR(i, film_pcb.lower_ele_cur_m_channel)
      film_pcb.motor_dir[i] = (film_pcb.motor_dir[i] == Film_Forward) ? Film_Reversal : Film_Forward;
      FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor check limit success... [Film_Verify_Motor_Limit]\n", i + 1));

      if (act == FILM_RESET)  film_pcb.run_open_val[i] = FILM_ALL_OPEN;
      film_pcb.rt_open_val[i] = film_pcb.run_open_val[i];
      film_pcb.last_open_val[i] = film_pcb.run_open_val[i];

      switch (act)
      {
        case FILM_ROLL :
          if (film_pcb.run_open_val[i] != FILM_ALL_CLOSE && film_pcb.run_open_val[i] != FILM_ALL_OPEN)
          {
            Film_Detect_Adjust_Open_Roll_CH(i + 1);
            continue;
          }
          Film_Set_Motor_Status_CH(i + 1, Film_M_Open_OK);
        break;

        case FILM_RESET : 
          if (!film_pcb.second_stage_flag[i]) Film_Set_Motor_Status_CH(i + 1, FILM_M_First_Reset_OK);
          else Film_Set_Motor_Status_CH(i + 1, Film_M_Reset_OK);

          if (!film_pcb.second_stage_flag[i]) continue;

          if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_SET_MODE))
          {
            Film_Store_Exp_Handler_CH(i + 1, FILM_RESET);
            FM_PRINT((FM_PrintDBG_Buf, "the %dth of motor save open or set flag exception!(RESET)  [Film_Verify_Motor_Limit]\n", i + 1));
            continue;
          }
          Film_Cal_Save_Ele_Cur_CH(i + 1);
        break;

        case FILM_F_ROLL :
          Film_Set_Motor_Status_CH(i + 1, Film_M_F_Open_OK);

          if (film_pcb.re_ok_before_f_roll_flag[i])
          {
            if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_SET_MODE))
              Film_Store_Exp_Handler_CH(i + 1, FILM_F_ROLL);
          }
        break;                            
      }
      if (Film_Save_Open_Val_CH(i + 1)) Film_Store_Exp_Handler_CH(i + 1, act);
      Film_Status_Msg_Delivery_Task(i + 1, film_pcb.run_motor_exp_sta[i]);
      Film_Pending_Motor_CH(i + 1, act);
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

/**
 @brief   : 三种运动模式通道上挂载的电机的卷膜前的参数处理。
 @param   : 无
 @return  : 无
 */
void Film_Motor_Start_Task(void)
{
  film_u8 flag;
  film_u8 i;

  /* 开度卷膜参数设置 */
  Film_Check_Reset_Roll_Flag();
  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    /* 读取服务器设置的目标开度、读取上一次卷膜开度 */
    if (Film_MEM_Read_Param_CH(FILM_MEM_LAST_OPEN_BASE_ADDR, &film_pcb.last_open_val[i], i + 1) ||
        Film_MEM_Read_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) )
    { 
      #if (!FILM_LOWER_LEVEL_RESET_MODE)
      Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);
      #endif
      Film_Store_Exp_Handler_CH(i + 1, FILM_ROLL);
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor open roll parameters exception!\n", i + 1));
      continue;
    }
    FM_PRINT((FM_PrintDBG_Buf, "The last open value of the %dth motor is %d%%\n", i+1 , film_pcb.last_open_val[i]));
    FM_PRINT((FM_PrintDBG_Buf, "The run open value of the %dth motor is %d%%\n", i+1 , film_pcb.run_open_val[i]));

    if (film_pcb.last_open_val[i] > FILM_ALL_OPEN || film_pcb.run_open_val[i] > FILM_ALL_OPEN)
    {
      FM_PRINT((FM_PrintDBG_Buf, "The %dth read open value exception!\n", i + 1));
      Film_Pending_Motor_CH(i + 1, FILM_ROLL);
    }
  FILM_TRA_BLOCK_END

  /* 强制卷膜参数设置 */
  FILM_TRA_MOTOR(i, film_pcb.m_f_open_channel)
    /* 判断强制卷膜前，是否早已重置行程完成。如果没有重置行程完成，那么等到强制卷膜完成后，是不应该置位重置行程完成标志位的 */
    #if (!FILM_LOWER_LEVEL_RESET_MODE)
    Film_MEM_Get_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, &flag, i + 1);
    (flag) ? film_pcb.re_ok_before_f_roll_flag[i] = FILM_SET_FLAG : film_pcb.re_ok_before_f_roll_flag[i] = FILM_RESET_FLAG;
    if (Film_MEM_Read_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1) ||
        Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE))
    {
      Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE);
      Film_Store_Exp_Handler_CH(i + 1, FILM_F_ROLL);
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor force open value exception!\n", i + 1));
      continue;      
    }
    #else
    Film_MEM_Get_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, &flag, i + 1);
    (flag) ? film_pcb.re_ok_before_f_roll_flag[i] = FILM_SET_FLAG : film_pcb.re_ok_before_f_roll_flag[i] = FILM_RESET_FLAG;
    if (Film_MEM_Read_Param_CH(FILM_MEM_RUN_OPEN_BASE_ADDR, &film_pcb.run_open_val[i], i + 1))
    {
      Film_Store_Exp_Handler_CH(i + 1, FILM_F_ROLL);
      FM_PRINT((FM_PrintDBG_Buf, "The %dth motor force open value exception!\n", i + 1));
      continue;      
    }
    #endif
    FM_PRINT((FM_PrintDBG_Buf, "The force run open value of the %dth motor is %d%%\n", i+1 , film_pcb.run_open_val[i]));
  FILM_TRA_BLOCK_END   

  /* 重置行程参数设置 */
  FILM_TRA_MOTOR(i, film_pcb.m_reset_channel)
    film_pcb.reset_m_dir_flag[i] = FILM_RESET_FLAG;
    film_pcb.run_time_save_flag[i] = FILM_RESET_FLAG;
    film_pcb.second_stage_flag[i] = FILM_RESET_FLAG;
    /* 清除重置完成标志位 */
    if (Film_MEM_Set_Flag_CH(FILM_MEM_RE_OK_FLAG_BASE_ADDR, i + 1, FILM_MEM_FLAG_RESET_MODE))
      Film_Store_Exp_Handler_CH(i + 1, FILM_RESET);
  FILM_TRA_BLOCK_END

  /* 设置电机的方向 */
  Film_Set_Motor_Dir(FILM_RESET);
  Film_Set_Motor_Dir(FILM_ROLL);
  Film_Set_Motor_Dir(FILM_F_ROLL);
  /* 初始化电流机制 */
  Film_Motor_EleCurrent_Init();
  /* 计算出挂载的电机需要的卷膜时长（开度卷膜） */
  FILM_TRA_MOTOR(i, film_pcb.m_open_channel)
    Film_Calculate_Roll_Time_CH(i + 1);
  FILM_TRA_BLOCK_END
  /* 挂载电机开始运动 */
  Film_Motor_Start_Run(film_pcb.m_reset_channel);
  Film_Motor_Start_Run(film_pcb.m_open_channel);
  Film_Motor_Start_Run(film_pcb.m_f_open_channel);
  film_pcb.ctrl_com_timing_num = 0;
  film_pcb.clt_ele_tim_num = 0; 
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
    //Film_Detect_OverCur();

    /* 重置行程基准点搜索*/
    Film_Seek_Reset_Origin();
    /* 重置行程中电流采集 */
    Film_Collect_Ele_Cur();
    /* 保存重置行程运动时长 */
    Film_Save_ResetRoll_Time();

    /* 信息监听 */
    Film_Listening_Task();

    /* 强制停止检测 */
    Film_Detect_Force_Stop();

    if (film_pcb.new_task_request_flag) return;

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
