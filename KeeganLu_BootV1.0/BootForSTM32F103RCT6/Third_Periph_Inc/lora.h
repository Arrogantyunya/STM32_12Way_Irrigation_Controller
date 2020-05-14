#ifndef _LORA_H
#define _LORA_H

/*
 *该文件是基于ST公司最新的STM32 CubeMX软件生成的HAL库而编写的。
 */

#include "../bt_inc/boot_config.h"

#define USE_OLD_LORA_VERTION    0

#define USE_LORA_RESET  1

/*
 *Lora模块功能引脚定义
 */
#define LORA_RESET_RCC_CLK_ENABLE 	__HAL_RCC_GPIOB_CLK_ENABLE()
#define LORA_RESET_GPIO_PIN					GPIO_PIN_12     
#define LORA_RESET_GPIO_PORT				GPIOB

#define LORA_WAKEUP_RCC_CLK_ENABLE	__HAL_RCC_GPIOB_CLK_ENABLE()
#define LORA_WAKEUP_GPIO_PIN				GPIO_PIN_1
#define LORA_WAKEUP_GPIO_PORT				GPIOB  

#define LORA_ATCMD_RCC_CLK_ENABLE		__HAL_RCC_GPIOD_CLK_ENABLE()
#define LORA_ATCMD_GPIO_PIN					GPIO_PIN_0
#define LORA_ATCMD_GPIO_PORT				GPIOB

#define LORA_PWR_RCC_CLK_ENABLE			__HAL_RCC_GPIOA_CLK_ENABLE()
#define LORA_PWR_GPIO_PIN					GPIO_PIN_13
#define LORA_PWR_GPIO_PORT					GPIOB

#define LORA_PWR_ON     (HAL_GPIO_WritePin(LORA_PWR_GPIO_PORT, LORA_PWR_GPIO_PIN, GPIO_PIN_SET))
#define LORA_PWR_OFF    (HAL_GPIO_WritePin(LORA_PWR_GPIO_PORT, LORA_PWR_GPIO_PIN, GPIO_PIN_RESET))

/*
 *LoRa连接串口引脚定义
 */
#define LoRaSerial                            USART2
#define LoRaSerial_RCC_CLK_ENABLE()          __HAL_RCC_USART2_CLK_ENABLE()
#define LoRaSerial_RCC_CLK_DISABLE()         __HAL_RCC_USART2_CLK_DISABLE()

#define LoRaSerial_GPIO_ClK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define LoRaSerial_Tx_GPIO_PIN               GPIO_PIN_2
#define LoRaSerial_Tx_GPIO_PORT              GPIOA
#define LoRaSerial_Rx_GPIO_PIN               GPIO_PIN_3   
#define LoRaSerial_Rx_GPIO_PORT              GPIOA

#define USART2_AF7                             GPIO_AF7_USART2
#define LoRa_Serial_IRQHandler                 USART2_IRQHandler
#define LoRa_Serial_IRQn                       USART2_IRQn

/*LoRa AT Command*/
//Inquiry command
#define AT_													"AT"
#define AT_ADDR_                    "AT+ADDR?\r\n"  //return 4 byte
#define AT_MADDR_                   "AT+MADDR?\r\n" //return 4 byte
#define AT_ID_                      "AT+ID?\r\n"    //return 8 byte
#define AT_SYNC_                    "AT+SYNC?\r\n"  //return 1 byte
#define AT_POW_                     "AT+POW?\r\n"   //return 1 byte 
#define AT_BW_                      "AT+BW?\r\n"    //return 1 byte
#define AT_CR_                      "AT+CR?\r\n"    //return 1 byte
#define AT_CRC_                     "AT+CRC?\r\n"   //return 1 byte
#define AT_TFREQ_                   "AT+TFREQ?\r\n" //return 4 byte
#define AT_RFREQ_                   "AT+RFREQ?\r\n" //return 4 byte
#define AT_TSF_                     "AT+TSF?\r\n"   //return 1 byte
#define AT_RSF_                     "AT+RSF?\r\n"   //return 1 byte
#define AT_CSQ_                     "AT+CSQ?\r\n"   //return 2 byte
#define AT_CFG_                     "AT+CFG?\r\n"
#define AT_TIQ_                      "AT+TIQ?\r\n"
#define AT_RIQ_                     "AT+RIQ?\r\n"
#define AT_NET_                     "AT+NET?\r\n"
#define AT_SIP_                     "AT+SIP?\r\n"

#define AT_BRATE_										"AT+BRATE?\r\n"

#define AT_INQUIRE_PARA(at)         at"\r\n"   

//Set command
#define AT_NET                      "AT+NET="
#define AT_AK                       "AT+AK="
#define AT_ADDR                     "AT+ADDR="    
#define AT_MADDR                    "AT+MADDR="
#define AT_MODE                     "AT+MODE="
#define AT_PREM                     "AT+PREM="
#define AT_LDR                      "AT+LDR="
#define AT_SYNC                     "AT+SYNC="
#define AT_POW                      "AT+POW="
#define AT_BW                       "AT+BW="
#define AT_CR                       "AT+CR="
#define AT_CRC                      "AT+CRC="
#define AT_TFREQ                    "AT+TFREQ="
#define AT_RFREQ                    "AT+RFREQ="
#define AT_TSF                      "AT+TSF="
#define AT_RSF                      "AT+RSF="
#define AT_TIQ                      "AT+TIQ="
#define AT_RIQ                      "AT+RIQ="
#define AT_SIP                      "AT+SIP="
#define AT_ACK                      "AT+ACK="
#define AT_BRATE                    "AT+BRATE="
#define AT_EL                       "AT+EL="

#define AT_BRATE										"AT+BRATE="

#define AT_CONFIG_PARA(at, para)    at#para"\r\n"


#define LORA_NO_RESET   HAL_GPIO_WritePin(LORA_RESET_GPIO_PORT, LORA_RESET_GPIO_PIN, GPIO_PIN_SET)
#define LORA_RESET   		HAL_GPIO_WritePin(LORA_RESET_GPIO_PORT, LORA_RESET_GPIO_PIN, GPIO_PIN_RESET)

#define LORA_NO_WAKEUP 	HAL_GPIO_WritePin(LORA_WAKEUP_GPIO_PORT, LORA_WAKEUP_GPIO_PIN, GPIO_PIN_RESET)

#define LORA_DEFAULT_PASS_MODE HAL_GPIO_WritePin(LORA_ATCMD_GPIO_PORT, LORA_ATCMD_GPIO_PIN, GPIO_PIN_RESET)

enum LoRa_Mode{
    AT = 0, PASS_THROUGH_MODE
};

enum Receive_Type{
    LoRa_OK = 0, LoRa_ERROR, LoRa_Bytes, LoRa_Invalid
};

enum Error_Mark{
    Grammar_Err = 0, Para_Err, Execute_Failed, Channel_Busy, Length_Err, Save_Failed, Buffer_Full, OverTime, Set_Refuse, Unreadable,
    No_Err
};

enum Cmd{
    CMOMON = 0, CSQ
};

/* LoRa通信模式配置成网关还是节点 0xF0节点; 0xF1网关 */
#define LORA_COM_MODE_ADDR                      76
#define LORA_COM_MODE_FLAG_ADDR                 77
#define LORA_COM_MODE_VERIFY_ADDR               78

#define EP_LORA_ADDR_BASE_ADDR                  79
#define EP_LORA_ADDR_END_ADDR                   86
#define EP_LORA_ADDR_VERIFY_ADDR                87
#define EP_LORA_ADDR_SAVED_FLAG_ADDR            88


void LoRa_GPIO_Config(void);
void LoRa_UART_Init(uint32_t baudrate);

void LoRaUart_Baud_Config(uint32_t baudrate, uint8_t is_init);
void LoRa_Shutdown(void);
void LoRa_Restart(void);
//AT mode or pass-through mode
unsigned char LoRa_Mode(enum LoRa_Mode AT_status);
void LoRa_IsReset(unsigned char Is_reset);
unsigned char LoRa_AT(unsigned char *data_buffer, unsigned char is_query, const char *cmd, const char *para);

void LoRa_Parameter_Init(unsigned char only_net, unsigned char only_baud);

void LoRa_Send_Data(uint8_t *pdata, uint16_t data_len);

extern UART_HandleTypeDef hLoRaSerial;

extern uint32_t gLoRa_BaudRate;

extern uint8_t gLoRa_Rcv_Buf[256];
extern uint8_t gLoRa_Rcv_Temp;
extern uint16_t gLoRa_Rcv_Len;

#endif
