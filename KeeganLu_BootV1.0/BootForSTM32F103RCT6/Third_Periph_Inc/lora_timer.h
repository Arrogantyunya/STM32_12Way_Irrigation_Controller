#ifndef _LORA_TIMER_H_
#define _LORA_TIMER_H_

#include "../bt_inc/boot_config.h"

#define LORA_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM6_CLK_ENABLE()
#define LORA_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM6_CLK_DISABLE()
#define LORA_TIM_IRQ                  TIM6_DAC_IRQn
#define LORA_TIM_Handler              TIM6_DAC_IRQHandler

void LoRa_Tim_Init(void);

void LoRa_Send_Data(uint8_t *pdata, uint16_t data_len);


extern uint8_t LoRa_Rcv_Buf[256];

extern uint8_t gRcv_FrameHead_Flag;

#endif
