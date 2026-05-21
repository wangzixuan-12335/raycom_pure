/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-21 00:49:39
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-21 01:46:03
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\device\src\Driver_MyServo.c
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#include "stm32f4xx.h"
#include "handle.h"


/**
 * @description: 改变舵机角度
 * @param {PWM_Type} *PWMx  PWM结构体
 * @param {uint8_t} angle   目标角度
 * @return {*}
 */
void My_Servo_ChangeAngle(PWM_Type *PWMx, uint8_t angle){
    if (angle<=180 && angle>=0)
    {
        PWM_Set_Compare(PWMx,(uint32_t)((angle/180.0f)*100+25));
    }
}

