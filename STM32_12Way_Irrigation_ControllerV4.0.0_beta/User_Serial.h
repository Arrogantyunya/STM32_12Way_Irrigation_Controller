#ifndef _USER_SERIAL_H
#define _USER_SERIAL_H

#define USE_Serial_Upload   true

#define USE_LORA_Control    true
#define USE_RS485_Control   false


#if USE_Serial_Upload
    #if USE_LORA_Control
        #define Debug_Serial    Serial
        #define LoRa_Serial     Serial1  //USART2 --> USART1, when use serial upload
        #define RS485_Serial    Serial2
    #elif USE_RS485_Control
        #define Debug_Serial    Serial  //USART2 --> USART1, when use serial upload
        #define LoRa_Serial     Serial2
        #define RS485_Serial    Serial1
    #endif
#else
   #if USE_LORA_Control
        #define Debug_Serial    Serial1
        #define LoRa_Serial     Serial2  
        #define RS485_Serial    Serial3
    #elif USE_RS485_Control
        #define Debug_Serial    Serial1  //USART2 --> USART1, when use serial upload
        #define LoRa_Serial     Serial3
        #define RS485_Serial    Serial2
    #endif 
#endif

#endif