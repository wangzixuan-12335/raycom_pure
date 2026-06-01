#ifndef __MY_TOF_H
#define __MY_TOF_H
#include "stm32f4xx.h"
// 定义设备写地址（7位地址 0x29 左移一位得到 0x52）
#define TOF_DEV_ADDR            0x52
#define TOF_XSHUT_PORT          GPIOE
#define TOF_XSHUT_PIN           GPIO_Pin_4
#define TOF_XSHUT_RCC_CLK       RCC_AHB1Periph_GPIOE

#define TOF_VCC_3V3_PORT        GPIOE
#define TOF_VCC_3V3_PIN         GPIO_Pin_5
#define TOF_VCC_3V3_RCC_CLK     RCC_AHB1Periph_GPIOE

//传感器静差
#define TOF_OFFSET_MM           20

void TOF_Init(void);
void TOF_Loop_Read(uint16_t *distance);
#endif
