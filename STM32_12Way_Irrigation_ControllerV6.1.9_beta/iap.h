#ifndef _IAP_H
#define _IAP_H

#include "boot_config.h"

typedef void (*IAPFun)(void);

void Jump_OBJ(unsigned int obj_addr);

#endif




