#ifndef __MY_I2C_H
#define __MY_I2C_H
#include "stm32f4xx.h"

#define SCL_PORT        GPIOF
#define SCL_PIN         GPIO_Pin_1
#define SCL_RCC_CLK     RCC_AHB1Periph_GPIOF

#define SDA_PORT        GPIOF
#define SDA_PIN         GPIO_Pin_0
#define SDA_RCC_CLK     RCC_AHB1Periph_GPIOF

#define DELAY_TIME      10

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

#endif
