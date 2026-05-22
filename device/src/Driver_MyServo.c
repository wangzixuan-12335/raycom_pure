/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-21 00:49:39
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-23 01:50:00
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
 * @param {uint8_t} angle   目标角度0-180
 * @return {*}
 */
void My_Servo_ChangeAngle(PWM_Type *PWMx, uint8_t angle){
    if (angle<=180 && angle>=0)
    {
        PWM_Set_Compare(PWMx,(uint32_t)((angle/180.0f)*100+25));
    }
}

/**
 * @description: 根据给定值控制舵机角度挡位
 * @param {int16_t} servo_status 
 * @return {*}
 */
void SetServoByController(int16_t servo_status){
        switch (servo_status)
        {
        case 1:
            My_Servo_ChangeAngle(&PWM_Holding_Jaw_Servo,0);
            break;
        case 3:
            My_Servo_ChangeAngle(&PWM_Holding_Jaw_Servo,90);
            break;
        case 2:
            My_Servo_ChangeAngle(&PWM_Holding_Jaw_Servo,180);
            break;
        default:
            break;
        }
}
