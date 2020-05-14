#ifndef _DEBUG_UART_H_
#define	_DEBUG_UART_H_

#include "../bt_inc/boot_config.h"
#include <stdio.h>

#define DEBUG_USART							USART1
#define DEBUG_BaudRate					115200
#define DEBUG_USART_RCC_CLK_ENABLE()		__HAL_RCC_USART1_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()		__HAL_RCC_USART1_CLK_DISABLE()

#define DEBUG_USART_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_Tx_PIN							GPIO_PIN_9
#define DEBUG_USART_Tx_PORT							GPIOA
#define DEBUG_USART_Rx_PIN							GPIO_PIN_10
#define DEBUG_USART_Rx_PORT							GPIOA

void Debug_Print_Init(void);

#endif
