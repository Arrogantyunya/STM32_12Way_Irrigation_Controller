#ifndef _IAP_H

#include "../bt_inc/boot_config.h"

typedef void (*IAPFun)(void);

void Write_APP_Bin(unsigned int write_addr, unsigned char *app_buffer, unsigned int app_len);
void IAP_Load_APP(unsigned int obj_addr);

#endif
