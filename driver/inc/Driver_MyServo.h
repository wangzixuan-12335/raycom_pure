/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-21 10:50:50
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-27 16:55:01
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\driver\inc\Driver_MyServo.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MY_SERVO_H
#define __MY_SERVO_H
#include "Driver_BSP.h";

#define SEEDLING_ANGLE 5
#define OPEN_ANGLE     120
#define STEM_ANGLE     15 

void My_Servo_ChangeAngle(PWM_Type *PWMx, uint8_t angle);
void SetServoByController(int16_t servo_status);
#endif
