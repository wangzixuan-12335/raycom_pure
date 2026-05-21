/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-19 22:39:25
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-20 20:19:54
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\driver\src\Driver_MyUSART.c
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#include "stm32f4xx.h"
#include "handle.h"

void MY_USART_Init(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate=100000;
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode=USART_Mode_Rx;
    USART_InitStructure.USART_Parity=USART_Parity_No;
    USART_InitStructure.USART_StopBits=USART_StopBits_1;
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;
    
    USART_Init(USART1,&USART_InitStructure);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1,ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 选择中断优先级分组 (建议在 main 函数开头统一设置一次)
    // 如果你用了 FreeRTOS，大疆通常习惯用 Group 4 (全部 4 位用于抢占优先级)
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

    // 2. 配置 USART1 中断通道
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    
    // 3. 设置抢占优先级和子优先级
    // 注意：如果是接视觉或关键传感器，优先级调高点；如果是普通调试，设中等即可
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    
    // 4. 使能通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStructure);
}

void DBUS_Decode(uint8_t *data, usart1_data_decoded_type *data_decoded)
{
    // 通道 0: Byte 0 + Byte 1 的低 3 位
    data_decoded->r2x = (((uint16_t)data[0] | ((uint16_t)data[1] << 8)) & 0x07FF)-1024;

    // 通道 1: Byte 1 的高 5 位 + Byte 2 的低 6 位
    data_decoded->r2y = ((((uint16_t)data[1] >> 3) | ((uint16_t)data[2] << 5)) & 0x07FF)-1024;

    // 通道 2: Byte 2 的高 2 位 + Byte 3 的全部 + Byte 4 的低 1 位
    data_decoded->r1x = ((((uint16_t)data[2] >> 6) | ((uint16_t)data[3] << 2) | ((uint16_t)data[4] << 10)) & 0x07FF)-1024;

    // 通道 3: Byte 4 的高 7 位 + Byte 5 的低 4 位
    data_decoded->r1y = ((((uint16_t)data[4] >> 1) | ((uint16_t)data[5] << 7)) & 0x07FF)-1024;

    // S1 开关: Byte 5 的位 4 和位 5
    data_decoded->s2 = (data[5] >> 6) & 0x0003;

    // S2 开关: Byte 5 的位 6 和位 7
    data_decoded->s1 = (data[5] >> 4) & 0x0003;
    
    // 注意：data[6] 之后的数据（鼠标、键盘信息）在此函数中被跳过
}