/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../Third_Periph_Inc/lora.h"
#include "../Third_Periph_Inc/i2c.h"
#include "../Third_Periph_Inc/at24Cxx.h"
#include "../Third_Periph_Inc/debug_uart.h"
#include "../bt_inc/boot_protocol.h"
#include "../Third_Periph_Inc/lora_timer.h"
#include "../Third_Periph_Inc/app_mem.h"

#include <stdlib.h>

void SystemClock_Config(void);

uint8_t gSN_Code[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

enum BT_R_RESULT Rcv_Data_Save(uint8_t *d_buf)
{
	return BT_R_OK;
}

void BT_Fill_ReservedByte(uint8_t *re_buf)
{
	uint8_t i;
	
	if (app_mem_Is_Saved_Check(SN_SAVE_FLAG_ADDR) != 0x55)
		for (i = 0; i < 9; i++) re_buf[i] = 0;
	else
		app_mem_Read_SN_Code(re_buf);

	if (app_mem_Is_Saved_Check(REGION_SAVE_FLAG_ADDR) != 0x55)
		re_buf[9] = 0;
	else
		app_mem_Read_SN_Code(&re_buf[9]);
	
	re_buf[10] = 0; //组号
}

uint8_t Reserved_Check(uint16_t f_type)
{
	return BT_R_OK;
}

int rand_number;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* 外设和时钟初始化 */
  HAL_Init();
	
  SystemClock_Config();

	Debug_Print_Init();
	
	I2C_Init();
	EEPROM_WP_Init();

	LoRa_GPIO_Config();
	
	HAL_Delay(2000);
	
	LoRaUart_Baud_Config(gLoRa_BaudRate, 1);

	LoRa_Parameter_Init(0, 0);
	
	LoRa_Tim_Init();
	
	/********************/
	
	BT_Mount_Dev_And_Init(LoRa_Send_Data);
	BT_APP_Init();

  while (1)
  {
		BT_Cycle_Query_SW_Version();
		BT_Receive_Protocol_Msg(&gLoRa_Rcv_Buf[0], &gLoRa_Rcv_Len);		
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
