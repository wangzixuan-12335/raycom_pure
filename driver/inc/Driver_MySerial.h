#ifndef __MY_SERIAL_H
#define __MY_SERIAL_H
#include "stm32f4xx.h"
#define MY_USARTx       USART2


void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);

#endif
