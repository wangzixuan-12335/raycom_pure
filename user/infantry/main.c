/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-19 21:48:45
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-23 00:06:53
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
    //遥控器USART+DMA转运, 
    //USART在中断里面处理解码，解码数据在结构体usart1_data_decoded中
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    MY_USART_Init();
    MyDMA_Init((uint32_t)&(USART1->DR),(uint32_t)usart1_raw_data,18);
    
    //初始化CAN通信
    //CAN接收写在CAN1中断内，根据不同报文头将数据写到结构体(eg)Motor_3508_LF中
    //CAN发送写在TIM2的定时(1kHZ)的中断中
    BSP_CAN_Init();

    //初始化夹爪舵机结构体，PWM初始化
    //BSP_PWM_Init内TIM_OCMode_PWM2 修改为 TIM_OCMode_PWM1
    BSP_PWM_Set_Port(&PWM_Holding_Jaw_Servo,PWM_PD12);
    BSP_PWM_Init(&PWM_Holding_Jaw_Servo,1800,1000,TIM_OCPolarity_High);

    //初始化电机结构体
    Motor_3508_Init(&Motor_3508_LF,IsPositive_True,19);
    Motor_3508_Init(&Motor_3508_RF,IsPositive_False,19);
    Motor_3508_Init(&Motor_3508_LB,IsPositive_True,19);
    Motor_3508_Init(&Motor_3508_RB,IsPositive_False,19);
    //将PID_Type的变量指针赋值给电机结构体
    Motor_3508_LF.Motor_PID=&Motor_3508_LF_PID;
    Motor_3508_RF.Motor_PID=&Motor_3508_RF_PID;
    Motor_3508_LB.Motor_PID=&Motor_3508_LB_PID;
    Motor_3508_RB.Motor_PID=&Motor_3508_RB_PID;
    //初始化MOTOR3508的pid
    PID_Init(Motor_3508_LF.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_RF.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_LB.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_RB.Motor_PID,7.5,0.1,0.5,10000,3000);
    //电机结构体合集
    MyMotor_3508_Collection.LF=&Motor_3508_LF;
    MyMotor_3508_Collection.RF=&Motor_3508_RF;
    MyMotor_3508_Collection.LB=&Motor_3508_LB;
    MyMotor_3508_Collection.RB=&Motor_3508_RB;

    //初始化TIM2，生成一个1KHZ的中断
    //BSP_TIM2_Init经过修改，改了分频器，GPIOA初始化被注释
    BSP_TIM2_Init();

    //改变夹爪舵机角度
    //My_Servo_ChangeAngle(&PWM_Holding_Jaw_Servo,180);
    
    while (1)
    {
		
    }
    
}

