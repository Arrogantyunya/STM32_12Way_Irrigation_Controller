#ifndef _FILM_MEM_H
#define _FILM_MEM_H

#include "film_config.h"

/* 卷膜库储存基地址，需要开发者来设置该基地址 */
#define FILM_MEM_BASE_ADDR                          150U

#define FILM_MEM_CHANNEL                            (MOTOR_CHANNEL - 1U)

#define FILM_MEM_BASE_ADDR_ALLOC(x)                 (x + 1U)
#define FILM_MEM_END_ADDR_ALLOC(x, m_size)          (x + (FILM_MEM_CHANNEL * m_size) + (m_size - 1))

#define FILM_MEM_ADDR_SIG(x)                        (x)

/* 运行开度储存地址 */
#define FILM_MEM_RUN_OPEN_BASE_ADDR                 FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_BASE_ADDR)
#define FILM_MEM_RUN_OPEN_END_ADDR                  FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RUN_OPEN_BASE_ADDR, sizeof(film_u8))
/* 运行开度校验储存地址 */
#define FILM_MEM_RUN_OPEN_CRC_BASE_ADDR             FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RUN_OPEN_END_ADDR)
#define FILM_MEM_RUN_OPEN_CRC_END_ADDR              FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RUN_OPEN_CRC_BASE_ADDR, sizeof(film_u8))

/* 上一次开度储存地址 */
#define FILM_MEM_LAST_OPEN_BASE_ADDR                FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RUN_OPEN_CRC_END_ADDR) 
#define FILM_MEM_LAST_OEPN_END_ADDR                 FILM_MEM_END_ADDR_ALLOC(FILM_MEM_LAST_OPEN_BASE_ADDR, sizeof(film_u8))
/* 上一次开度校验储存地址 */
#define FILM_MEM_LAST_OPEN_CRC_BASE_ADDR            FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_LAST_OEPN_END_ADDR)
#define FILM_MEM_LAST_OPEN_CRC_END_ADDR             FILM_MEM_END_ADDR_ALLOC(FILM_MEM_LAST_OPEN_CRC_BASE_ADDR, sizeof(film_u8))

/* 实时开度储存地址 */
#define FILM_MEM_RT_OPEN_BASE_ADDR                  FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_LAST_OPEN_CRC_END_ADDR)
#define FILM_MEM_RT_OPEN_END_ADDR                   FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RT_OPEN_BASE_ADDR, sizeof(film_u8))
/* 实时开度校验储存地址 */
#define FILM_MEM_RT_OPEN_CRC_BASE_ADDR              FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RT_OPEN_END_ADDR)
#define FILM_MEM_RT_OPEN_CRC_END_ADDR               FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RT_OPEN_CRC_BASE_ADDR, sizeof(film_u8))
/* 异常开度暂存储存地址（当卷膜中异常断电，这里保存运行开度,以便于恢复开度） */
#define FILM_MEM_EXP_TEMP_OPEN_BASE_ADDR            FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RT_OPEN_CRC_END_ADDR)
#define FILM_MEM_EXP_TEMP_OPEN_END_ADDR             FILM_MEM_END_ADDR_ALLOC(FILM_MEM_EXP_TEMP_OPEN_BASE_ADDR, sizeof(film_u8))
#define FILM_MEM_EXP_TEMP_OPEN_CRC_BASE_ADDR        FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_EXP_TEMP_OPEN_END_ADDR)
#define FILM_MEM_EXP_TEMP_OPEN_CRC_END_ADDR         FILM_MEM_END_ADDR_ALLOC(FILM_MEM_EXP_TEMP_OPEN_CRC_BASE_ADDR, sizeof(film_u8))
/* 得到新的开度任务，但某些路还未重置行程，暂存目标开度，待重置行程完成后，取出开度，卷膜到目标开度*/
#define FILM_MEM_AUTO_OPEN_BASE_ADDR                FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_EXP_TEMP_OPEN_CRC_END_ADDR)
#define FILM_MEM_AUTO_OPEN_END_ADDR                 FILM_MEM_END_ADDR_ALLOC(FILM_MEM_AUTO_OPEN_BASE_ADDR, sizeof(film_u8))
#define FILM_MEM_AUTO_OPEN_CRC_BASE_ADDR            FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_AUTO_OPEN_END_ADDR)
#define FILM_MEM_AUTO_OPEN_CRC_END_ADDR             FILM_MEM_END_ADDR_ALLOC(FILM_MEM_AUTO_OPEN_CRC_BASE_ADDR, sizeof(film_u8))

/* 电流阈值储存地址 */
#define FILM_MEM_ELE_CUR_LMT_BASE_ADDR              FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_AUTO_OPEN_CRC_END_ADDR)
#define FILM_MEM_ELE_CUR_LMT_END_ADDR               FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ELE_CUR_LMT_BASE_ADDR, sizeof(film_u8))
#define FILM_MEM_ELE_CUR_LMT_CRC_BASE_ADDR          FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ELE_CUR_LMT_END_ADDR)
#define FILM_MEM_ELE_CUR_LMT_CRC_END_ADDR           FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ELE_CUR_LMT_CRC_BASE_ADDR, sizeof(film_u8))

#define FILM_MEM_RT_STA_FRQ_BASE_ADDR               FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ELE_CUR_LMT_CRC_END_ADDR)
#define FILM_MEM_RT_STA_FRQ_END_ADDR                FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RT_STA_FRQ_BASE_ADDR, sizeof(film_u8))
#define FILM_MEM_RT_STA_FRQ_CRC_BASE_ADDR           FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RT_STA_FRQ_END_ADDR)
#define FILM_MEM_RT_STA_FRQ_CRC_END_ADDR            FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RT_STA_FRQ_CRC_BASE_ADDR, sizeof(film_u8))

/* 重置行程完成标志位 */
#define FILM_MEM_RE_OK_FLAG_BASE_ADDR               FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RT_STA_FRQ_CRC_END_ADDR)
#define FILM_MEM_RE_OK_FLAG_END_ADDR                FILM_MEM_END_ADDR_ALLOC(FILM_MEM_RE_OK_FLAG_BASE_ADDR, sizeof(film_u8))
/* 卷膜电流保存标志位 */
#define FILM_MEM_ELE_CUR_FLAG_BASE_ADDR             FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_RE_OK_FLAG_END_ADDR)
#define FILM_MEM_ELE_CUR_FLAG_END_ADDR              FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ELE_CUR_FLAG_BASE_ADDR, sizeof(film_u8))
/* 掉电开度异常标志位 */
#define FILM_MEM_EXP_OPEN_FLAG_BASE_ADDR            FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ELE_CUR_FLAG_END_ADDR)
#define FILM_MEM_EXP_OPEN_FLAG_END_ADDR             FILM_MEM_END_ADDR_ALLOC(FILM_MEM_EXP_OPEN_FLAG_BASE_ADDR, sizeof(film_u8))

/* 每个储存信息只占用一个字节空间的储存结束边界地址 */
#define FILM_MEM_ONE_BYTE_SIZE_END_ADDR             FILM_MEM_ADDR_SIG(FILM_MEM_EXP_OPEN_FLAG_END_ADDR)

/* 电机重置行程卷膜时间 */
#define FILM_MEM_ROLL_TIME_BASE_ADDR                FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ONE_BYTE_SIZE_END_ADDR)
#define FILM_MEM_ROLL_TIME_END_ADDR                 FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ROLL_TIME_BASE_ADDR, sizeof(film_u16))
#define FILM_MEM_ROLL_TIME_CRC_BASE_ADDR            FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ROLL_TIME_END_ADDR)
#define FILM_MEM_ROLL_TIME_CRC_END_ADDR             FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ROLL_TIME_CRC_BASE_ADDR, sizeof(film_u8))
/* 重置行程采集的电流值 */
#define FILM_MEM_ELE_CUR_BASE_ADDR                  FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ROLL_TIME_CRC_END_ADDR)
#define FILM_MEM_ELE_CUR_END_ADDR                   FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ELE_CUR_BASE_ADDR, sizeof(film_u16))
#define FILM_MEM_ELE_CUR_CRC_BASE_ADDR              FILM_MEM_BASE_ADDR_ALLOC(FILM_MEM_ELE_CUR_END_ADDR)
#define FILM_MEM_ELE_CUR_CRC_END_ADDR               FILM_MEM_END_ADDR_ALLOC(FILM_MEM_ELE_CUR_CRC_BASE_ADDR, sizeof(film_u8))     

/* 每个储存信息占用两个字节空间的储存结束边界地址 */
#define FILM_MEM_TWO_BYTE_SIZE_END_ADDR             FILM_MEM_ADDR_SIG(FILM_MEM_ELE_CUR_CRC_END_ADDR)

#define FILM_MEM_USE_SIZE                           (FILM_MEM_ADDR_SIG(FILM_MEM_TWO_BYTE_SIZE_END_ADDR) - FILM_MEM_BASE_ADDR)

#define FILM_MEM_SET_FLAG                           0x55U
#define FILM_MEM_RESET_FLAG                         0x00U

#define AL2_FCS_COEF        ((1 << 7) + (1 << 6) + (1 << 5))

typedef enum {
    FILM_MEM_OK = 0x00U,
    FILM_MEM_R_ERR = 0x01U,
    FILM_MEM_W_ERR = 0x02U,
    FILM_MEM_CRC_ERR = 0x03U,
    FILM_MEM_CH_ERR = 0x04U, //填入的电机路数不合法
    FILM_MEM_EN_SAVE = 0x05U,
    FILM_MEM_DIS_SAVE = 0x06U
}film_mem_err;

typedef enum{
    FILM_MEM_FLAG_SET_MODE = 0x00U,
    FILM_MEM_FLAG_RESET_MODE = 0x01U
}film_flag_mode;

film_mem_err Film_MEM_Save_Param(film_u32 w_base_addr, film_u8 *w_buf);
film_mem_err Film_MEM_Read_Param(film_u32 r_base_addr, film_u8 *r_buf);
film_mem_err Film_MEM_Save_Param_CH(film_u32 w_base_addr, film_u8 *w_buf, film_u8 ch);
film_mem_err Film_MEM_Read_Param_CH(film_u32 r_base_addr, film_u8 *r_buf, film_u8 ch);

film_mem_err Film_MEM_Set_Flag(film_u32 w_base_addr, film_flag_mode mode);
film_mem_err Film_MEM_Get_Flag(film_u32 r_base_addr, film_u8 *flag_buf);
film_mem_err Film_MEM_Set_Flag_CH(film_u32 w_base_addr, film_u8 ch, film_flag_mode mode);
film_mem_err Film_MEM_Get_Flag_CH(film_u32 r_base_addr, film_u8 *flag_buf, film_u8 ch);


#endif
