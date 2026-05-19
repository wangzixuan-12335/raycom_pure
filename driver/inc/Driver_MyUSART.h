#include "stm32f4xx.h"
#ifndef __MY_USART_H
#define __MY_USART_H
void MY_USART_Init();
void DBUS_Decode(uint8_t *data, uint16_t *data_decoded);
#endif
