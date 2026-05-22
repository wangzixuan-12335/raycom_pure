#ifndef __MY_USART_H
#define __MY_USART_H
#include "stm32f4xx.h"
typedef struct
{
    int16_t r1x;
    int16_t r1y;
    int16_t r2x;
    int16_t r2y;
    int16_t s1;
    int16_t s2;
}usart1_data_decoded_type;


void MY_USART_Init();
void DBUS_Decode(uint8_t *data, usart1_data_decoded_type *data_decoded);
#endif
