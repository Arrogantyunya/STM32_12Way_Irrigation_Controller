#include "film_mem.h"

film_u8 Film_MEM_Get_DT_Size(film_u32 addr);
film_mem_err Film_MEM_Check_Idtc_Content(film_u32 saved_addr, film_u8 *new_buf, film_u32 len);
film_u8 Film_GetCrc8(film_u8 * data, film_u32 length);

/* 连续顺序写/读一串数据缓存 */
_film_(weak) film_mem_err Film_MEM_Write_Buffer(film_u32 base_addr, film_u8 *buf, film_u32 len) 
{ 
  FILM_DEBUGF(BT_CB_DBG, (FM_PrintDBG_Buf, "warning: [Film_MEM_Write_Buffer] function is not implemented!\n"));
  return FILM_MEM_W_ERR; 
}
_film_(weak) film_mem_err Film_MEM_Read_Buffer(film_u32 base_addr, film_u8 *buf, film_u32 len) 
{ 
  FILM_DEBUGF(BT_CB_DBG, (FM_PrintDBG_Buf, "warning: [Film_MEM_Read_Buffer] function is not implemented!\n"));
  return FILM_MEM_R_ERR; 
}

/* 写一段连续的数据到储存器*/
film_mem_err Film_MEM_Save_Param(film_u32 w_base_addr, film_u8 *w_buf)
{
  film_u32 w_len;
  film_u8 t_size;
  film_u8 crc_val[MOTOR_CHANNEL];
  film_u8 i;

  t_size = Film_MEM_Get_DT_Size(w_base_addr);
  w_len = MOTOR_CHANNEL * t_size;

  /* 写入的和已经保存的相同，不重复写入 */
  if (Film_MEM_Check_Idtc_Content(w_base_addr, &w_buf[0], w_len) == FILM_MEM_DIS_SAVE) return FILM_MEM_OK;

  for (i = 0; i < MOTOR_CHANNEL; i++)
    crc_val[i] = Film_GetCrc8(&w_buf[i * t_size], t_size);

  if (Film_MEM_Write_Buffer(w_base_addr, &w_buf[0], w_len) != FILM_MEM_OK) return FILM_MEM_W_ERR;
  if (Film_MEM_Write_Buffer((w_base_addr + MOTOR_CHANNEL * t_size), &crc_val[0], MOTOR_CHANNEL) != FILM_MEM_OK) return FILM_MEM_W_ERR;
 
  return FILM_MEM_OK;
}

/* 从储存器读一段连续的数据 */
film_mem_err Film_MEM_Read_Param(film_u32 r_base_addr, film_u8 *r_buf)
{
  film_u32 r_len;
  film_u8 crc[MOTOR_CHANNEL], crc_tmp;
  film_u8 t_size;
  film_u32 i;

  t_size = Film_MEM_Get_DT_Size(r_base_addr);
  r_len = MOTOR_CHANNEL * t_size;

  if (Film_MEM_Read_Buffer(r_base_addr, &r_buf[0], r_len) != FILM_MEM_OK) return FILM_MEM_R_ERR;

  for (i = 0; i < MOTOR_CHANNEL; i++)
  {
    crc[i] = Film_GetCrc8(&r_buf[i * t_size], t_size);
    Film_MEM_Read_Buffer((r_base_addr + r_len + i), &crc_tmp, sizeof(film_u8));
    if (crc[i] != crc_tmp)
      return FILM_MEM_CRC_ERR;
  } 
  return FILM_MEM_OK;
}

/* 按照指定的路数，写入该路的信息，电机路数不能为0或者超过设置的最大路数 */
film_mem_err Film_MEM_Save_Param_CH(film_u32 w_base_addr, film_u8 *w_buf, film_u8 ch)
{
  film_u32 w_addr;
  film_u32 crc_addr;
  film_u8 t_size;
  film_u8 crc_val;

  if ((ch > MOTOR_CHANNEL) || !ch) return FILM_MEM_CH_ERR;

  t_size = Film_MEM_Get_DT_Size(w_base_addr);       
  w_addr = w_base_addr + ((ch - 1) * t_size);  //计算出该路的写入地址
  crc_addr = w_base_addr + (MOTOR_CHANNEL * t_size) + (ch - 1);  //计算出该路的校验写入地址

  /* 写入的和已经保存的相同，不重复写入 */
  if (Film_MEM_Check_Idtc_Content(w_addr, &w_buf[0], t_size) == FILM_MEM_DIS_SAVE) return FILM_MEM_OK;

  crc_val = Film_GetCrc8(&w_buf[0], t_size);

  if (Film_MEM_Write_Buffer(w_addr, &w_buf[0], t_size) != FILM_MEM_OK) return FILM_MEM_W_ERR;
  if (Film_MEM_Write_Buffer(crc_addr, &crc_val, sizeof(film_u8)) != FILM_MEM_OK) return FILM_MEM_W_ERR;

  return FILM_MEM_OK;
}

/* 按照指定的路数，读出该路的信息，电机路数不能为0或者超过设置的最大路数 */
film_mem_err Film_MEM_Read_Param_CH(film_u32 r_base_addr, film_u8 *r_buf, film_u8 ch)
{
  film_u32 r_addr;
  film_u32 crc_addr;
  film_u8 t_size;
  film_u8 crc_val, crc_tmp;

  if ((ch > MOTOR_CHANNEL) || !ch) return FILM_MEM_CH_ERR;

  t_size = Film_MEM_Get_DT_Size(r_base_addr);
  r_addr = r_base_addr + ((ch - 1) * t_size);  //计算出该路的读取地址
  crc_addr = r_base_addr + (MOTOR_CHANNEL * t_size) + (ch - 1);  //计算出该路的校验读取地址

  if (Film_MEM_Read_Buffer(r_addr, &r_buf[0], t_size) != FILM_MEM_OK) return FILM_MEM_R_ERR;

  crc_val = Film_GetCrc8(&r_buf[0], t_size);
  Film_MEM_Read_Buffer(crc_addr, &crc_tmp, sizeof(film_u8));
  if (crc_val != crc_tmp)
    return FILM_MEM_CRC_ERR;

  return FILM_MEM_OK;
}

/* 写一段连续的标志位到储存器*/
film_mem_err Film_MEM_Set_Flag(film_u32 w_base_addr, film_flag_mode mode)
{
  film_u8 flag_buf[MOTOR_CHANNEL];
  film_u8 i;

  for (i = 0; i < MOTOR_CHANNEL; i++)
    flag_buf[i] = (mode == FILM_MEM_FLAG_SET_MODE) ? FILM_MEM_SET_FLAG : FILM_MEM_RESET_FLAG;

  /* 写入的和已经保存的相同，不重复写入 */
  if (Film_MEM_Check_Idtc_Content(w_base_addr, &flag_buf[0], MOTOR_CHANNEL) == FILM_MEM_DIS_SAVE) return FILM_MEM_OK;

  if (Film_MEM_Write_Buffer(w_base_addr, &flag_buf[0], MOTOR_CHANNEL) != FILM_MEM_OK) return FILM_MEM_W_ERR;
 
  return FILM_MEM_OK;
}

/* 从储存器读一段连续的标志位 */
film_mem_err Film_MEM_Get_Flag(film_u32 r_base_addr, film_u8 *flag_buf)
{
  if (Film_MEM_Read_Buffer(r_base_addr, &flag_buf[0], MOTOR_CHANNEL) != FILM_MEM_OK) return FILM_MEM_W_ERR;
  return FILM_MEM_OK;
}

/* 按照指定的路数和模式，设置该路的标志位 */
film_mem_err Film_MEM_Set_Flag_CH(film_u32 w_base_addr, film_u8 ch, film_flag_mode mode)
{
  film_u32 w_addr;
  film_u8 flag;

  if ((ch > MOTOR_CHANNEL) || !ch) return FILM_MEM_CH_ERR;

  flag = (mode == FILM_MEM_FLAG_SET_MODE) ? FILM_MEM_SET_FLAG : FILM_MEM_RESET_FLAG;
  w_addr = w_base_addr + (ch - 1);  //计算出该路的写入地址

  /* 写入的和已经保存的相同，不重复写入 */
  if (Film_MEM_Check_Idtc_Content(w_addr, &flag, sizeof(film_u8)) == FILM_MEM_DIS_SAVE) return FILM_MEM_OK;

  if (Film_MEM_Write_Buffer(w_addr, &flag, sizeof(film_u8)) != FILM_MEM_OK) return FILM_MEM_W_ERR;

  return FILM_MEM_OK;
}

/* 按照指定的路数，读出该路的标志位，电机路数不能为0或者超过设置的最大路数 */
film_mem_err Film_MEM_Get_Flag_CH(film_u32 r_base_addr, film_u8 *flag_buf, film_u8 ch)
{
  film_u32 r_addr;

  if ((ch > MOTOR_CHANNEL) || !ch) return FILM_MEM_CH_ERR;

  r_addr = r_base_addr + (ch - 1);  //计算出该路的读取地址
  if (Film_MEM_Read_Buffer(r_addr, &flag_buf[0], sizeof(film_u8)) != FILM_MEM_OK) return FILM_MEM_R_ERR;

  return FILM_MEM_OK;
}


/* 得到数据类型的大小 */
film_u8 Film_MEM_Get_DT_Size(film_u32 addr)
{
  /* 对于每一路电机的信息占用一个字节的方式 */
  if (addr <= FILM_MEM_ONE_BYTE_SIZE_END_ADDR)  return (sizeof(film_u8));
  /* 对于每一路电机的信息占用两个字节的方式 */
  else if (addr <= FILM_MEM_TWO_BYTE_SIZE_END_ADDR) return (sizeof(film_u16)); 
  /* 对于每一路电机的信息占用四个字节的方式 */
  else  return (sizeof(film_u32));
}

/* 比较要保存的和已经保存的是否相等。注意：相等返回1，不相等返回0 */
film_mem_err Film_MEM_Check_Idtc_Content(film_u32 saved_addr, film_u8 *new_buf, film_u32 len)
{
  film_u8 tmp;
	film_u32 i;

	for (i = 0; i < len; i++)
  {
    Film_MEM_Read_Buffer((saved_addr + i), &tmp, sizeof(film_u8));
    if (new_buf[i] != tmp) return FILM_MEM_EN_SAVE;
  }
	return FILM_MEM_DIS_SAVE;
}

film_u8 Film_GetCrc8(film_u8 * data, film_u32 length)
{ 
   unsigned char cFcs = 0;
   int i, j;
 
   for( i = 0; i < length; i ++ )
   {
      cFcs ^= data[i];
      for(j = 0; j < 8; j ++)
      {
         if(cFcs & 1)
            cFcs = (unsigned char)((cFcs >> 1) ^ AL2_FCS_COEF);
         else
            cFcs >>= 1;
      }
   } 
   return cFcs;
}
