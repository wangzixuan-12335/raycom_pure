#ifndef __MY_DELAY_H
#define __MY_DELAY_H

#include "stm32f4xx.h"

void My_delay_init(void);
void My_delay_us(uint32_t nus);
void My_delay_ms(uint32_t nms);

#endif