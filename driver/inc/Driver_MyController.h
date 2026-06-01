#ifndef __MY_CONTROLLER_H
#define __MY_CONTROLLER_H
#include "stm32f4xx.h"
#include "handle.h"
void MyController_Move(usart1_data_decoded_type *data);
void MyController_Stay(usart1_data_decoded_type *data);
#endif