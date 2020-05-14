#ifndef	_APP_MEM_H_
#define _APP_MEM_H_

#include "../bt_inc/boot_config.h"

/* Զ������Э��涨����������ǰ50���ֽ�����Э���ռ����ֹ��������д�룡 */
#define APP_MEM_BASE_ADDR									 51

/*SN����ز��������ַ*/
#define SN_SAVE_FLAG_ADDR             (APP_MEM_BASE_ADDR + 0)
#define SN_BASE_ADDR                  (APP_MEM_BASE_ADDR + 1)
#define SN_END_ADDR                   (APP_MEM_BASE_ADDR + 9)
#define SN_CRC_ADDR                   (APP_MEM_BASE_ADDR + 10)
#define SN_ACCESS_NET_FLAG_ADDR       (APP_MEM_BASE_ADDR + 11)

/*�����������ز��������ַ*/
#define REGION_ADDR                   (APP_MEM_BASE_ADDR + 12)
#define REGION_CRC_ADDR               (APP_MEM_BASE_ADDR + 13)
#define REGION_SAVE_FLAG_ADDR         (APP_MEM_BASE_ADDR + 14)

/*��Ĥ�����������ز��������ַ*/
#define GROUP_NUM_BASE_ADDR           (APP_MEM_BASE_ADDR + 15)
#define GROUP_NUM_END_ADDR            (APP_MEM_BASE_ADDR + 19)
#define GROUP_NUM_CRC_ADDR            (APP_MEM_BASE_ADDR + 20)
#define GROUP_NUM_SAVE_FLAG_ADDR      (APP_MEM_BASE_ADDR + 21)


uint8_t app_mem_Is_Saved_Check(uint8_t flag_addr);

uint8_t app_mem_Save_SN_Code(uint8_t *sn_buf);
uint8_t app_mem_Read_SN_Code(uint8_t *sn_buf);

uint8_t app_mem_Save_Region_Code(uint8_t r_code);
uint8_t app_mem_Read_Region_Code(uint8_t *r_code);

uint8_t app_mem_Save_Group_Code(uint8_t *group_buf);
uint8_t app_mem_Read_Group_Code(uint8_t *group_buf);

#endif
