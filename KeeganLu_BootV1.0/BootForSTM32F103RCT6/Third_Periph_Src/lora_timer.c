#include "../Third_Periph_Inc/lora_timer.h"
#include "../Third_Periph_Inc/lora.h"
#include "../bt_inc/boot_protocol.h"
#include "../Third_Periph_Inc/debug_uart.h"

TIM_HandleTypeDef hlora_tim;

uint32_t gLoRa_Rcv_IRQ_Num = 0;	//定时设置LoRa接收中断，确保能及时恢复因意外LoRa收不到数据


void LoRa_Tim_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	__HAL_RCC_TIM6_CLK_ENABLE();
	
  hlora_tim.Instance = TIM6;
  hlora_tim.Init.Prescaler = 83;	//分频范围0~65535，不要超过
  hlora_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  hlora_tim.Init.Period = 1000;	//1ms
  HAL_TIM_Base_Init(&hlora_tim);
	
  sMasterConfig.MasterOutputTrigger = TIM_IT_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&hlora_tim, &sMasterConfig);
	
   /* 外设中断配置 */
  HAL_NVIC_SetPriority(LORA_TIM_IRQ, 2, 0);
  HAL_NVIC_EnableIRQ(LORA_TIM_IRQ);
	
	HAL_TIM_Base_Start_IT(&hlora_tim);
}

void LoRa_Send_Data(uint8_t *pdata, uint16_t data_len)
{
	printf("send data len : %dBytes\n", data_len);
	printf("Send data frame : ");
	for (uint16_t i = 0; i < data_len; i++)
	{
		printf("%X ", pdata[i]);
	}
	printf("\n");
	HAL_UART_Transmit(&hLoRaSerial, &pdata[0], data_len, 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if (UartHandle->Instance == LoRaSerial)
	{
		HAL_UART_Receive_IT(&hLoRaSerial, &gLoRa_Rcv_Temp, 1);
		
		if (gLoRa_Rcv_Len >= 255) gLoRa_Rcv_Len = 0;
		gLoRa_Rcv_Buf[gLoRa_Rcv_Len++] = gLoRa_Rcv_Temp;
	}
	else
	{
		printf("不是LoRa串口接收！\n");
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(__HAL_TIM_GET_IT_SOURCE(&hlora_tim, TIM_IT_UPDATE)!=RESET)
  {
    __HAL_TIM_CLEAR_IT(&hlora_tim, TIM_IT_UPDATE);
		
		BT_Rcv_Tim_Num++;
		BT_Query_CycVal++;
		BT_Rand_Seed_Num++;
		
		gLoRa_Rcv_IRQ_Num++;
		if (gLoRa_Rcv_IRQ_Num > 100)
		{
			HAL_UART_Receive_IT(&hLoRaSerial, &gLoRa_Rcv_Temp, 1);
			gLoRa_Rcv_IRQ_Num = 0;
		}
  }
}

void LORA_TIM_Handler(void)
{
  HAL_TIM_IRQHandler(&hlora_tim);
}

void LoRa_Serial_IRQHandler(void)
{
	HAL_UART_IRQHandler(&hLoRaSerial);
}

void BT_Stop_Interrupt(void) 
{
	HAL_TIM_Base_Stop_IT(&hlora_tim);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
}
