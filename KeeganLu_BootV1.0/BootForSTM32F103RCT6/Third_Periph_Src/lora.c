/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/3/18
 * 
 * 该文件仅仅是针对仁钰科技的MHL9LF型号的LoRa无线模块所编写，不一定适用于其他型号，或其他厂商
 * 的LoRa模块。该文件主要功能有配置MHL9LF型号LoRa模块的引脚模式、通过AT指令读取各类LoRa参数
 * 信息、通过AT指令配置各类LoRa参数等。对于该LoRa模块，需要注意的是，每一次发送的AT指令都要
 * 留意返回的数据是否是ERROR标志，如果是，建议做一些相应的异常处理。
 * 更多MHL9LF型号的LoRa无线模块信息，请参考仁钰公司的技术文档。
 * 头文件中提供了各个类的公共接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "../Third_Periph_Inc/lora.h"
#include "../Third_Periph_Inc/at24Cxx.h"
#include "../Third_Periph_Inc/crc8.h"
#include "../Third_Periph_Inc/debug_uart.h"

UART_HandleTypeDef hLoRaSerial;

uint32_t gLoRa_BaudRate = 9600;	//设置LoRa模块和设备串口之间的波特率
uint16_t gLoRa_Wait_Relay_Delay	=	220;

/* 软件方式切换AT模式和透传模式的指令 */
unsigned char SOFT_AT[3] = "+++";
unsigned char SOFT_PATH[5] = "ATT\r\n";

/* LoRa模块接收数据的相关缓存变量 */
unsigned char gLoRa_Rcv_Buf[256] = {0};
unsigned char gLoRa_Rcv_Temp = 0;
unsigned short gLoRa_Rcv_Len = 0;

void LoRa_Parse_Command(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer, unsigned char *data_len);
void LoRa_Get_CSQ(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer);

unsigned char EP_Verify_LoRa_Addr_Flag(void);
unsigned char EP_Save_LoRa_Com_Mode(unsigned char mode);
unsigned char EP_Read_LoRa_Com_Mode(void);

unsigned char EP_Save_LoRa_Addr(unsigned char *addr);
unsigned char EP_Read_LoRa_Addr(unsigned char *addr);

/*
 @brief     : 配置LoRa模块相关引脚
 @param     : 无
 @return    : 无
 */
void LoRa_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	LORA_RESET_RCC_CLK_ENABLE;
	LORA_WAKEUP_RCC_CLK_ENABLE;
	LORA_ATCMD_RCC_CLK_ENABLE;
	LORA_PWR_RCC_CLK_ENABLE;
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/* 配置LoRa供电控制引脚 */
	GPIO_InitStruct.Pin = LORA_PWR_GPIO_PIN;
	HAL_GPIO_Init(LORA_PWR_GPIO_PORT, &GPIO_InitStruct);
	LORA_PWR_ON;
	
	/* 配置LoRa AT指令控制引脚 */
	GPIO_InitStruct.Pin = LORA_ATCMD_GPIO_PIN;
	HAL_GPIO_Init(LORA_ATCMD_GPIO_PORT, &GPIO_InitStruct);
	LORA_DEFAULT_PASS_MODE;	//在引脚上默认拉低，透传模式
	
	/* 配置LoRa唤醒引脚 */
	GPIO_InitStruct.Pin = LORA_WAKEUP_GPIO_PIN;
	HAL_GPIO_Init(LORA_WAKEUP_GPIO_PORT, &GPIO_InitStruct);
	LORA_NO_WAKEUP;	//拉低，默认正常工作
	
	/* 配置LoRa复位引脚 */
	GPIO_InitStruct.Pin = LORA_RESET_GPIO_PIN;
	HAL_GPIO_Init(LORA_RESET_GPIO_PORT, &GPIO_InitStruct);	
	LORA_NO_RESET;	//拉高，默认不复位
}

/**
 @brief		: 配置LoRa模块的串口相关信息，如波特率，通信模式，中断。
 @param		: 波特率
 @return	: 无
 */
void LoRa_UART_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
  LoRaSerial_GPIO_ClK_ENABLE();
	LoRaSerial_RCC_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = LoRaSerial_Tx_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LoRaSerial_Tx_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LoRaSerial_Rx_GPIO_PIN;
	HAL_GPIO_Init(LoRaSerial_Rx_GPIO_PORT, &GPIO_InitStruct);
  
  hLoRaSerial.Instance = LoRaSerial;
	hLoRaSerial.Init.BaudRate = baudrate;
  hLoRaSerial.Init.WordLength = UART_WORDLENGTH_8B;
  hLoRaSerial.Init.StopBits = UART_STOPBITS_1;
  hLoRaSerial.Init.Parity = UART_PARITY_NONE;
  hLoRaSerial.Init.Mode = UART_MODE_TX_RX;
  hLoRaSerial.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hLoRaSerial.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&hLoRaSerial);
  
  HAL_NVIC_SetPriority(LoRa_Serial_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(LoRa_Serial_IRQn);
	
	HAL_UART_Receive_IT(&hLoRaSerial, &gLoRa_Rcv_Temp, 1);
}

/**
 @brief		: 解除LoRa串口中断，串口引脚等串口功能。
 @param		: 无
 @return 	: 无
 */
void LoRaSerial_End(void)
{
	LoRaSerial_RCC_CLK_DISABLE();
	HAL_NVIC_DisableIRQ(USART2_IRQn);

	HAL_GPIO_DeInit(LoRaSerial_Tx_GPIO_PORT, LoRaSerial_Tx_GPIO_PIN);
	HAL_GPIO_DeInit(LoRaSerial_Rx_GPIO_PORT, LoRaSerial_Rx_GPIO_PIN);
}

void Config_LoRa_Delay(uint32_t baud)
{
	switch (baud)
	{
		case 1200 : gLoRa_Wait_Relay_Delay = 220; break;
		case 2400 : gLoRa_Wait_Relay_Delay = 120; break;
		case 4800 : gLoRa_Wait_Relay_Delay = 100; break;
		case 9600 : gLoRa_Wait_Relay_Delay = 80; break;
		case 19200 : gLoRa_Wait_Relay_Delay = 40; break;
		case 38400 : gLoRa_Wait_Relay_Delay = 30; break;
		case 57600 : gLoRa_Wait_Relay_Delay = 20; break;
		case 115200 : gLoRa_Wait_Relay_Delay = 20; break;
	}
}

void LoRaUart_Baud_Config(uint32_t baudrate, uint8_t is_init)
{
	unsigned char temp_buf[20];
	unsigned int temp_baud;
	unsigned char i = 0;
	
	if (!is_init) 
	{
		LoRa_UART_Init(baudrate);
		return;
	}
	
	for (i = 0; i < 8; i++)
	{
		switch (i)
		{
			case 0 : temp_baud = 9600; 		break;
			case 1 : temp_baud = 115200; 	break;
			case 2 : temp_baud = 4800; 		break;
			case 3 : temp_baud = 2400; 		break;
			case 4 : temp_baud = 1200; 		break;
			case 5 : temp_baud = 19200; 	break;
			case 6 : temp_baud = 38400; 	break;
			case 7 : temp_baud = 57600; 	break;
		}
		LoRa_UART_Init(temp_baud);
		HAL_Delay(200);
		
		Config_LoRa_Delay(temp_baud);
		
		if (LoRa_AT(temp_buf, 1, AT_NET_, 0)) 
		{
			printf("Adapt LoRa module OK, baudrate : %d\n", temp_baud);
			if (temp_baud != gLoRa_BaudRate)
			{
				printf("Prepare to modify lora baudrate : %d\n", gLoRa_BaudRate);
				LoRa_Parameter_Init(0, 1);
			}
			else
				printf("There is no need to change the lora baudrate...\n");
			return;
		}
	}
	printf("Adapt LoRa module Err !!!\n");	
}

/**
 @brief     : 软件方式配置LoRa模式。
 @param     : AT_status : 高电平是AT模式，低电平是透传模式
 @return    : 无
 */
unsigned char LoRa_Mode(enum LoRa_Mode AT_status)
{
	gLoRa_Rcv_Len = 0;
	
	if (AT_status == AT)
		HAL_UART_Transmit(&hLoRaSerial, SOFT_AT, sizeof(SOFT_AT), 1000);
	else
		HAL_UART_Transmit(&hLoRaSerial, SOFT_PATH, sizeof(SOFT_PATH), 1000);

	HAL_Delay(gLoRa_Wait_Relay_Delay);

	if (gLoRa_Rcv_Buf[2] == 'O' && gLoRa_Rcv_Buf[3] == 'K')
		return 1;
	else
		return 0;
}

/**
 @brief     : 决定LoRa模块是否重启
 @param     : 无
 @return    : 无
 */
void LoRa_Reset(void)
{
	LORA_RESET;
	HAL_Delay(150); //150ms
	LORA_NO_RESET;
}

/* 如果可以软件切断LoRa供电 */
#if USE_LORA_RESET
/**
 @brief     : 将LoRa模块完全断电
 @param     : 无
 @return    : 无
 */
void LoRa_Shutdown(void)
{   
	LORA_PWR_OFF;
	LoRaSerial_End();
	
	HAL_GPIO_WritePin(LoRaSerial_Tx_GPIO_PORT, LoRaSerial_Tx_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LoRaSerial_Rx_GPIO_PORT, LoRaSerial_Rx_GPIO_PIN, GPIO_PIN_RESET);
	LORA_RESET; 
}

/**
 @brief     : 将完全断电的LoRa模块重启
 @param     : 无
 @return    : 无
 */
void LoRa_Restart(void)
{
    LoRa_GPIO_Config();
		LoRaUart_Baud_Config(gLoRa_BaudRate, 0);
    
    LoRa_Reset();
    LoRa_Mode(PASS_THROUGH_MODE);
}
#endif

/**
 @brief     : 检测是否收到错误回执
 @param     : 1.要检验的数据缓存
              2.接收信息缓存
 @return    : 错误或正确
 */
unsigned char LoRa_Check_Err_Msg(unsigned char *check_data)
{
	unsigned char err;
	if (check_data[0] == 'E' && check_data[1] == 'R')
	{
			err = (check_data[2] - '0') * 10 + (check_data[3] - '0');   //解析出错误代号，如：01, 02等
			printf("Lora Err Code = %d\n", err);  //打印出该错误代号
			return err;
	}
  return No_Err;
}

/**
 @brief     : 用于设置LoRa参数后，如果设置参数成功，接收到回执“OK”
 @param     : 1.要验证的数据缓存
              2.接收信息缓存   
 @return    : true or false                    
 */
unsigned char LoRa_Check_OK_Msg(unsigned char *check_data, unsigned char *data_buf)
{
    /*校验接收回执，是否成功设置参数*/
    if (check_data[0] == 'O' && check_data[1] == 'K')
    {
        data_buf[0] = check_data[0];
        data_buf[1] = check_data[1];
        return 1;
    }
	return 0;
}

/**
 @brief     : LoRa模块核心函数，用来发送AT指令给LoRa模块，同时也包含了判断是否接到查询数据、或设置OK，
              或设置失败等信息。该函数用来查询参数。
 @param     : 1.AT命令
              2.接收返回的信息
              3.接收信息长度
 @return    : received data type (bytes, error, OK)
 */
enum Receive_Type LoRa_AT_Query_Cmd(unsigned char *cmd, unsigned char *d_buf, unsigned char *d_len)
{
	unsigned char cmd_len = 0, copy_len = 0;
	unsigned char i;
	
	gLoRa_Rcv_Len = 0;
	
	/* 计算出本次查询指令的长度 */
	for (i = 0; cmd[i] != '\n'; i++) cmd_len++;
	cmd_len++;	

	/*发送指令给LoRa，同时接收返回的数据*/
	HAL_UART_Transmit(&hLoRaSerial, &cmd[0], cmd_len, 1000);
	HAL_Delay(gLoRa_Wait_Relay_Delay);
	
	/*判断接收的数据长度是否有效*/
	if (!gLoRa_Rcv_Len)
	{
		printf("No receive data !!!\n");
		return LoRa_Invalid;
	}

	/*验证帧头帧尾格式是否合法*/
	if ((gLoRa_Rcv_Buf[0] == '\r' && gLoRa_Rcv_Buf[1] == '\n') && 
		(gLoRa_Rcv_Buf[gLoRa_Rcv_Len - 1] == '\n' && gLoRa_Rcv_Buf[gLoRa_Rcv_Len - 2] == '\r'))
	{
			/*接收除了帧头帧尾的有效数据*/
		 for (i = 2; i < gLoRa_Rcv_Len - 2; i++)
				 gLoRa_Rcv_Buf[copy_len++] =  gLoRa_Rcv_Buf[i];

			/*判断接收的回执是否是ERROR*/
		 if (LoRa_Check_Err_Msg(&gLoRa_Rcv_Buf[0]) != No_Err) return LoRa_ERROR;

		 /*分析接收的回执*/
		 LoRa_Parse_Command(gLoRa_Rcv_Buf, copy_len, d_buf, d_len);	return LoRa_Bytes;
	}
	return LoRa_Invalid;
}

/*
 @brief     : LoRa模块核心函数，用来发送AT指令给LoRa模块，同时也包含了判断是否接到查询数据、或设置OK，
              或设置失败等信息。
              该函数用来设置参数
 @param     : 1.AT指令
              2.要设置的参数
              3.接收的数据缓存
 @return    : received data type (bytes, error, OK)
 */
enum Receive_Type LoRa_AT_Config_Cmd(const char *cmd, const char * para, unsigned char *d_buf)
{
    /*参考 AT_Query_Cmd 函数*/
    unsigned char copy_len = 0;
    unsigned char at_cmd[20] = {0};
    unsigned char i = 0, j = 0;
		
		/* 先复制指令名 */
    for (; cmd[i] != '\0'; i++)
        at_cmd[i] = cmd[i];
		/* 再复制要配置的参数 */
    for (; para[j] != '\0'; j++)
        at_cmd[i++] = para[j];
    /*最后加入AT指令帧尾格式*/
    at_cmd[i++] = '\r';
    at_cmd[i++] = '\n';

    /*发送指令给LoRa，同时接收返回的数据*/
		gLoRa_Rcv_Len = 0;
		HAL_UART_Transmit(&hLoRaSerial, &at_cmd[0], i, 1000);
		
		/* 如果指令是配置波特率，那么要同步设备本身的波特率设置 */
		if (at_cmd[3] == 'B' && at_cmd[4] == 'R')
		{
			LoRaUart_Baud_Config(gLoRa_BaudRate, 0);
			HAL_Delay(1000);
			return LoRa_OK;
		}
		
    HAL_Delay(gLoRa_Wait_Relay_Delay);

	 if (!gLoRa_Rcv_Len)
	 {
			printf("No receive data !!!\n");
			return LoRa_Invalid;
	 }

	if ((gLoRa_Rcv_Buf[0] == '\r' && gLoRa_Rcv_Buf[1] == '\n') && 
		(gLoRa_Rcv_Buf[gLoRa_Rcv_Len - 1] == '\n' && gLoRa_Rcv_Buf[gLoRa_Rcv_Len - 2] == '\r'))
	{
		for (i = 2; i < gLoRa_Rcv_Len - 2; i++)
			gLoRa_Rcv_Buf[copy_len++] =  gLoRa_Rcv_Buf[i];

		if (LoRa_Check_Err_Msg(&gLoRa_Rcv_Buf[0]) != No_Err) return LoRa_ERROR;
		if (LoRa_Check_OK_Msg(&gLoRa_Rcv_Buf[0], d_buf) == 1) 
				return LoRa_OK;
		else
				return LoRa_ERROR;
	}
  return LoRa_Invalid;
}

/**
 @brief     : 在AT指令查询参数后，用于判断返回的参数是何种参数，然后传递给对应的函数解析获取参数
 @param     : 1.接收的回执信息
              2.回执信息的长度
              3.返回分析的数据缓存
              4.返回分析的数据缓存长度
 @return    : 无
 */
void LoRa_Parse_Command(unsigned char *rcv_buf, unsigned char rcv_len, unsigned char *d_buf, unsigned char *d_len)
{
	unsigned char which_cmd;
	unsigned char i = 0, j = 0;

	/*判断是否命令是查询信号强度命令*/
	if (rcv_buf[1] == 'C' && rcv_buf[2] == 'S' && rcv_buf[3] == 'Q')
			which_cmd = CSQ;
	else
			which_cmd = CMOMON;

	/*回执的有效数据在 : 后面，所以这里截取 : 后面的数据*/
	while (rcv_buf[i] != ':')
			i++;
	i++;	//得到有效数据的起始位置
	
	/* 重新截取赋值数据，这个时候rcv_buf里装的就只剩除了\r\n和:外的有效数据了*/
	for (; i <= rcv_len; i++)
			rcv_buf[j++] = rcv_buf[i];

	if (which_cmd == CMOMON)	//除了信号质量外的回执，将信息从接收缓存里复制给调用者的缓存，给调用者进一步处理
	{
			for (i = 0; i < j; i++)
					d_buf[i] = rcv_buf[i];
			*d_len = --i;
	}
	else if (which_cmd == CSQ)
	{
			LoRa_Get_CSQ(rcv_buf, j - 1, d_buf);
	}
}

/*
 @brief     : 得到信噪比和接收信号强度参数。
 @param     : 1.接收的数据回执
        2.接收数据回执的长度
        3.返回分析的数据缓存
 @return    : 无
 */
void LoRa_Get_CSQ(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer)
{
  /*信噪比和接收强度两个值中间有逗号隔开，这里去除逗号，得到数值*/
	if (addr_temp[1] == ',' && len >= 3)
	{
		data_buffer[0] = addr_temp[0];
		data_buffer[1] = addr_temp[2];
	}
}

/*
 @brief     : LoRa返回的回执信息是ASCLL码，将ASCLL码转换成HEX。
 @param     : 1.ASCLL码
              2.ASCLL码长度
 @return    : true or false
 */
unsigned char String_to_Hex(unsigned char *str, unsigned char len)
{
    for (unsigned char i = 0; i < len; i++)
    {
        if (str[i] >= '0' && str[i] <= '9')
            str[i] -= '0';
        else if (str[i] >= 'A' && str[i] <= 'F')
            str[i] -= '7';
        else if (str[i] >= 'a' && str[i] <= 'f')
            str[i] -= 'W';
        else
            return 0;
    }
    return 1;
}

/*
 @brief     : 发送AT指令接口。可以通过该函数设置LoRa模块，或是查询设置信息。
 @param     : 1.接收返回的数据
              2.是查询指令还是设置指令
              3.命令名字
              4.命令参数（如果是查询指令，忽略命令参数）
 @return    : true or false
 */
unsigned char LoRa_AT(unsigned char *d_buf, unsigned char is_query, const char *cmd, const char *para)
{
	unsigned char rcv_len = 0;
	unsigned char return_type;
	unsigned char i;

  LoRa_Mode(AT);

	/*根据指令是查询指令还是设置参数指令，分别进入不同的指令函数*/
	if (is_query)
			return_type = LoRa_AT_Query_Cmd((unsigned char *)cmd, d_buf, &rcv_len);
	else
			return_type = LoRa_AT_Config_Cmd(cmd, para, d_buf);

	/*根据指令执行情况，返回对应状态。*/
	switch (return_type)
	{
			case LoRa_ERROR      : printf("Receipt ERROR...\n");  goto rcv_err; 
			case LoRa_Invalid    : printf("Involid!\n"); goto rcv_err;
			case LoRa_OK         : printf("Set param OK\n"); break;
			case LoRa_Bytes      : 	for (i = 0; i < rcv_len; i++)
																printf("%c", d_buf[i]);
															printf("\n");	
			break;
	}
	LoRa_Mode(PASS_THROUGH_MODE);
	return 1;
	
	rcv_err :
	{
		LoRa_Mode(PASS_THROUGH_MODE);
		return 0;
	}
}


/**
 @brief     : 由于对该LoRa模块产商出产配置的初始LoRa地址极其不信任，故该函数的功能就是
              将LoRa地址读取出来，再写入进去。确保“表里如一”。
 @param     : 无
 @return    : 无
 */
unsigned char Rewrite_ID(void)
{
	unsigned char RcvBuffer[8];
	char EP_Buffer[9] = {0};
	char WriteAddr[9];
	unsigned char i;
	unsigned char VerifyFlag = 1;

	/* 读取LoRa通信地址 */
	if (!LoRa_AT(RcvBuffer, 1, AT_ADDR_, 0))
	{
			printf("Read LoRa ADDR Err <Rewrite_ID>\n");
			return 0;
	}

	for (i = 0; i < 8; i++)
	{
			WriteAddr[i] = RcvBuffer[i];

			if (WriteAddr[i] <= 9)
					WriteAddr[i] += '0';
			else if ((WriteAddr[i] >= 10) && (WriteAddr[i] <= 15))
					WriteAddr[i] += '7';
	}
	WriteAddr[8] = '\0';

	if (EP_Verify_LoRa_Addr_Flag())
	{
		printf("Verify LoRa Addr <Rewrite_ID>\n");

		/* 如果读出来的LoRa地址正确 */
		if(EP_Read_LoRa_Addr((unsigned char *)EP_Buffer))
		{
			for (i = 0; i < 8; i++)
			{
				/* 如果查询的LoRa地址和EP储存的不同 */
				if (WriteAddr[i] != EP_Buffer[i])
				{
					VerifyFlag = 0;
					break;
				}
			}
			/* 重新写入EP储存的正确的地址 */
			if (!VerifyFlag)
			{
				printf("LoRa addr for AT Error!, write EP addr to LoRa! <Rewrite_ID>\n");
				if(!LoRa_AT(RcvBuffer, 0, AT_ADDR, EP_Buffer))
				return 0;
			}
			else
			{
				printf("LoRa addr for AT Correct, OK <Rewrite_ID>\n");
			}
		}
		/* EP储存的地址不幸出错，选择相信从AT指令读出来的，重新写入EP保存 */
		else
		{
			printf("EP saved err, rewrite! <Rewrite_ID>\n");
			if (!EP_Save_LoRa_Addr((unsigned char *)WriteAddr))
			return 0;
		}
	}
  /* 如果是设备是第一次运行 */
	else
	{
		/*写入读出来的地址*/
		if(!LoRa_AT(RcvBuffer, 0, AT_ADDR, WriteAddr))
				return 0;

		if (!EP_Save_LoRa_Addr((unsigned char *)WriteAddr))
				return 0;
	}
  return 1;
}

unsigned char LoRa_Param_Check(const char *cmd, const char *para, unsigned char only_set)
{
    unsigned char RcvBuffer[10] = {0};
    unsigned char copy_len = 0;
    unsigned char param_len = 0;
    unsigned char QueryOKFlag = 1;

    if (!only_set)
    {
        /* 查询该设置下的参数是否和预设的吻合 */
        if(!LoRa_AT(RcvBuffer, 1, cmd, para))
        {
            printf("Query parameters Faild! <Param_Check>\n");
            return 0;
        }

        for (unsigned char i = 0; para[i] != '\0'; i++)
            param_len++;

        char CopyBuffer[param_len];

        for (unsigned char i = 0; i < param_len; i++)
        {
            CopyBuffer[i] = RcvBuffer[i];
        }

        for (unsigned char i = 0; i < param_len; i++)
        {
            if (CopyBuffer[i] <= 9)
                CopyBuffer[i] += '0';
            else if (CopyBuffer[i] >= 10 && CopyBuffer[i] <= 15)
                CopyBuffer[i] += '7';
        }

        for (unsigned char i = 0; i < param_len; i++)
        {
            if (CopyBuffer[i] != para[i])    // 如果查询的参数和预设的不吻合，重新写入预设的参数。
            {
                printf("The parameters are different from what is expected. Reset the parameters <Param_Check>\n");
                QueryOKFlag = 0;
                break;
            }
        }
        if (QueryOKFlag)
        {
            printf("Read param correct...\n");
            return 1;
        }
    }

    if (QueryOKFlag == 0 || only_set == 1)
    {
        for (unsigned char i = 0; cmd[i] != '\n'; i++)
            copy_len++;
                
        char set_cmd[copy_len - 1];
        for (unsigned char i = 0; i < (copy_len - 1); i++)
            set_cmd[i] = cmd[i];

        set_cmd[copy_len - 1] = '\0';
        set_cmd[copy_len - 2] = '=';

        if(LoRa_AT(RcvBuffer, 0, set_cmd, para))
        {
            return 1;
        }
        else
        {
            printf("Parameters set Err! <Param_Check>\n");
            return 0;
        }
    }
		return 0;
}

void LoRa_Baud_Change_Cmd(char *cmd)
{
	cmd[0] = '0';
	cmd[2] = 0;
	
	switch (gLoRa_BaudRate)
	{
		case 1200 	: cmd[1] = '0'; break;
		case 2400 	: cmd[1] = '1'; break;
		case 4800 	: cmd[1] = '2'; break;
		case 9600 	: cmd[1] = '3'; break;
		case 19200 	: cmd[1] = '4'; break;
		case 38400 	: cmd[1] = '5'; break;
		case 57600 	: cmd[1] = '6'; break;
		case 115200 : cmd[1] = '7'; break;
	}
}

/*
 @brief     : 初始化LoRa配置。如果没有配置，无法与网关通信。
 @param     : 无
 @return    : 无
 */
void LoRa_Parameter_Init(unsigned char only_net, unsigned char only_baud)
{
    unsigned char StatusBuffer[25] = {0};
    unsigned char i, j;
    unsigned char AlarmLEDRunNum = 0;
    unsigned char SetStatusFlag;
		char baud_cmd[3];

    printf("Configurate LoRa parameters...\n");
		
		gLoRa_Rcv_Len = 0;
 
	do{
			i = 0;
			SetStatusFlag = 1;
			
			if (only_baud)
			{
				LoRa_Baud_Change_Cmd(&baud_cmd[0]);
				LoRa_Param_Check(AT_BRATE, &baud_cmd[0], 1);
				Config_LoRa_Delay(gLoRa_BaudRate);
				return;
			}
			else if (only_net)
			{
				#if USE_LORA_RESET
					 if (EP_Read_LoRa_Com_Mode() == 0xF0)
							StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "00", 1);    //配置成节点模式
					else
							StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "01", 1);    //配置成网关模式   
				#else
					StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "01", true);
				#endif
			}
			else if ((!only_net) && (!only_net))
			{
				StatusBuffer[i++] = Rewrite_ID();
				StatusBuffer[i++] = LoRa_Param_Check(AT_MADDR_, "71000000", 0);

				StatusBuffer[i++] = LoRa_Param_Check(AT_TFREQ_, "1C578DE0", 0);
				StatusBuffer[i++] = LoRa_Param_Check(AT_RFREQ_, "1C03AE80", 0);
				StatusBuffer[i++] = LoRa_Param_Check(AT_SYNC_, "34", 0);
				
				#if USE_LORA_RESET
				if (EP_Read_LoRa_Com_Mode() == 0xF0)
				{
						#if USE_OLD_LORA_VERTION
						StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "00", 1);    //配置成网关模式//配置成节点模式
						#else
						StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "00", 0);
						#endif
				}
				else
				{
						#if USE_OLD_LORA_VERTION
						StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "01", 1);    //配置成网关模式
						#else
						StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "01", 0);
						#endif
				}

				#else
					StatusBuffer[i++] = LoRa_Param_Check(AT_NET_, "01", true);    
				#endif

				StatusBuffer[i++] = LoRa_Param_Check(AT_TSF_, "09", 0); 
				StatusBuffer[i++] = LoRa_Param_Check(AT_RSF_, "09", 0); 

				#if USE_OLD_LORA_VERTION
				StatusBuffer[i++] = LoRa_Param_Check(AT_RIQ_, "00", 1);
				StatusBuffer[i++] = LoRa_Param_Check(AT_SIP_, "01", 1);
				StatusBuffer[i++] = LoRa_Param_Check(AT_TIQ_, "00", 1);
				#else
				StatusBuffer[i++] = LoRa_Param_Check(AT_RIQ_, "00", 0);
				StatusBuffer[i++] = LoRa_Param_Check(AT_SIP_, "01", 0);
				StatusBuffer[i++] = LoRa_Param_Check(AT_TIQ_, "00", 0); 
				#endif
				StatusBuffer[i++] = LoRa_Param_Check(AT_BW_, "07", 0); 
				StatusBuffer[i++] = LoRa_Param_Check(AT_POW_, "14", 0);
			}
			
			for (j = 0; j < i; j++)
			{
				if (!StatusBuffer[j])
				{
						SetStatusFlag = 0;
						printf("Param init Err, Try again...\n");

						#if USE_LORA_RESET
						AlarmLEDRunNum++;
						LoRa_Restart();
						LoRa_Shutdown();
						#endif

						break;
				}
			}
			if (SetStatusFlag)  return;	//如果配置自检所有LoRa参数成功，退出

			#if USE_LORA_RESET
			LoRa_Restart();
			HAL_Delay(2000);
			#endif
	}
	#if USE_LORA_RESET
			while (AlarmLEDRunNum < 10);
	#else
			while (AlarmLEDRunNum < 1);
	#endif


	//LED_SET_LORA_PARA_ERROR;
	printf("LoRa parameters set ERROR! <Parameter_Init>\n");
	HAL_Delay(3000);
	gLoRa_Rcv_Len = 0;
	//nvic_sys_reset();
}

void EP_Save_LoRa_Addr_Flag(void)
{
    if (AT24Cxx_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0X55)
        return;

    EEPROM_WP_En();
    AT24Cxx_WriteOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR, 0x55);
    EEPROM_WP_Dis();
}

void EP_Clear_LoRa_Addr_Flag(void)
{
    if (AT24Cxx_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0x00)
        return;

    EEPROM_WP_En();
    AT24Cxx_WriteOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR, 0x00);
    EEPROM_WP_Dis();
}

unsigned char EP_Verify_LoRa_Addr_Flag(void)
{
    if (AT24Cxx_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0x55)
        return 1;
    else
        return 0;
}

unsigned char EP_Save_LoRa_Addr(unsigned char *addr)
{
    unsigned char TempBuf[8];
    unsigned char AddrCrc8, AddrTempCrc8;

    EEPROM_WP_En();
    AddrCrc8 = GetCRC8(&addr[0], 8);
    for (unsigned char i = 0; i < 8; i++)
    {
        AT24Cxx_WriteOneByte(EP_LORA_ADDR_BASE_ADDR + i, addr[i]);
    }
    AT24Cxx_WriteOneByte(EP_LORA_ADDR_VERIFY_ADDR, AddrCrc8);

    for (unsigned char i = 0; i < 8; i++)
    {
        TempBuf[i] = AT24Cxx_ReadOneByte(EP_LORA_ADDR_BASE_ADDR + i);
    }
    AddrTempCrc8 = GetCRC8(&TempBuf[0], 8);

    if (AddrCrc8 != AddrTempCrc8)
    { 
        EP_Clear_LoRa_Addr_Flag();
        EEPROM_WP_Dis();
        return 0;
    }
    else
    {
        printf("Save LoRa Addr OK...\n");
        EP_Save_LoRa_Addr_Flag();
        return 1;
    }
}

unsigned char EP_Read_LoRa_Addr(unsigned char *addr)
{
    unsigned char TempBuf[8];
    unsigned char AddrCrc8, AddrTempCrc8;

    for (unsigned char i = 0; i < 8; i++)
    {
        TempBuf[i] = AT24Cxx_ReadOneByte(EP_LORA_ADDR_BASE_ADDR + i);
    }
    AddrCrc8 = GetCRC8(&TempBuf[0], 8);
    AddrTempCrc8 = AT24Cxx_ReadOneByte(EP_LORA_ADDR_VERIFY_ADDR);

    if (AddrCrc8 != AddrTempCrc8)
        return 0;
    
    for (unsigned char i = 0; i < 8; i++)
    {
        addr[i] = TempBuf[i];
    }
    printf("Read LoRa Addr OK...\n");
    return 1;
}

unsigned char EP_Save_LoRa_Com_Mode_Flag(void)
{
    if (AT24Cxx_ReadOneByte(LORA_COM_MODE_FLAG_ADDR) == 0x55)
        return 1;

    EEPROM_WP_En();
    AT24Cxx_WriteOneByte(LORA_COM_MODE_FLAG_ADDR, 0x55);
    EEPROM_WP_Dis();
    
    if (AT24Cxx_ReadOneByte(LORA_COM_MODE_FLAG_ADDR == 0x55))
        return 1;
    else
        return 0;
}

unsigned char EP_Clear_LoRa_Com_Mode_Flag(void)
{
    if (AT24Cxx_ReadOneByte(LORA_COM_MODE_FLAG_ADDR) == 0x00)
        return 1;

    EEPROM_WP_En();
    AT24Cxx_WriteOneByte(LORA_COM_MODE_FLAG_ADDR, 0x00);
    EEPROM_WP_Dis();
    
    if (AT24Cxx_ReadOneByte(LORA_COM_MODE_FLAG_ADDR == 0x00))
        return 1;
    else
        return 0;
}

unsigned char EP_Save_LoRa_Com_Mode(unsigned char mode)
{
    unsigned char ModeCrc8;

    if (mode == 0xF0 || mode == 0xF1)
    {
        if (AT24Cxx_ReadOneByte(LORA_COM_MODE_ADDR) == mode)
            return 1;
        
        EEPROM_WP_En();
        AT24Cxx_WriteOneByte(LORA_COM_MODE_ADDR, mode);
        ModeCrc8 = GetCRC8(&mode, 1);
        AT24Cxx_WriteOneByte(LORA_COM_MODE_VERIFY_ADDR, ModeCrc8);

        if (AT24Cxx_ReadOneByte(LORA_COM_MODE_ADDR) == mode)
        {
            EP_Save_LoRa_Com_Mode_Flag();
            EEPROM_WP_Dis();
            return 1;
        }
        else
        {
            EEPROM_WP_Dis();
            return 0;
        }
    }
    else
    {
        printf("Save LoRa communication mode Err <Save_LoRa_Com_Mode>\n");
        return 0;
    }
}

unsigned char EP_Read_LoRa_Com_Mode(void)
{   
    unsigned char ComTemp;
    unsigned char Comcrc8;
		
    ComTemp = AT24Cxx_ReadOneByte(LORA_COM_MODE_ADDR);
    Comcrc8 = GetCRC8(&ComTemp, 1);
		
    if (Comcrc8 == AT24Cxx_ReadOneByte(LORA_COM_MODE_VERIFY_ADDR))
    {
        return ComTemp;
    }
    else
    {
        EP_Save_LoRa_Com_Mode(0xF0);
        return 0xF0;
    }
}
