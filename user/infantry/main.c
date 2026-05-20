/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-19 21:48:45
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-20 11:06:53
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
    MY_USART_Init();
    MyDMA_Init((uint32_t)&(USART1->DR),(uint32_t)usart1_raw_data,18);
    while (1)
    {
        /* code */
    }
    
}
