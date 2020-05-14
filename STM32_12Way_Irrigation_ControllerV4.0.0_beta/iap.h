#ifndef _IAP_H
#define _IAP_H

#include "boot_config.h"
#include "boot_mem.h"
#include "boot_protocol.h"

typedef void (*IAPFun)(void);

void Jump_OBJ(unsigned int obj_addr);

#endif




