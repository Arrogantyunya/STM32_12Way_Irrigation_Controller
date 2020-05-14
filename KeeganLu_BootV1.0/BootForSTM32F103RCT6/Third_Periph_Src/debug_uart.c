#include "../Third_Periph_Inc/debug_uart.h"

UART_HandleTypeDef huart1;

void Debug_Print_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;
	
	DEBUG_USART_RCC_CLK_ENABLE();
	DEBUG_USART_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = DEBUG_USART_Tx_PIN;
	HAL_GPIO_Init(DEBUG_USART_Tx_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = DEBUG_USART_Rx_PIN;
	HAL_GPIO_Init(DEBUG_USART_Rx_PORT, &GPIO_InitStruct);
	
	huart1.Instance = DEBUG_USART;
	huart1.Init.BaudRate = DEBUG_BaudRate;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);
}

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (unsigned char *)&ch, 1, 0xFFFF);
	return ch;
}

int fgetc(FILE *f)
{
	unsigned char ch = 0;
	HAL_UART_Receive(&huart1, &ch, 1, 0xFFFF);
	return ch;
}
