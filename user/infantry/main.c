/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-19 21:48:45
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-21 01:43:29
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\user\infantry\main.c
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"



void main(void) {
    //遥控器USART+DMA转运
    MY_USART_Init();
    MyDMA_Init((uint32_t)&(USART1->DR),(uint32_t)usart1_raw_data,18);
    
    //初始化CAN通信
    BSP_CAN_Init();

    //初始化夹爪舵机结构体，PWM初始化
    BSP_PWM_Set_Port(&PWM_Holding_Jaw_Servo,PWM_PD12);
    BSP_PWM_Init(&PWM_Holding_Jaw_Servo,1800,1000,TIM_OCPolarity_High);
    
    //改变夹爪舵机角度
    //My_Servo_ChangeAngle(&PWM_Holding_Jaw_Servo,90);
    
    while (1)
    {
        /* code */
        //开环测试电机
        //Can_Send(CAN1,0x200,0,0,usart1_data_decoded[3],0);
    }
    
}
