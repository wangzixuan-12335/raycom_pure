/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-22 00:23:36
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-22 22:36:47
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\driver\inc\Driver_MyChassis.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MY_CHASSIS_H
#define __MY_CHASSIS_H
#include "stm32f4xx.h"
#define Wheel_radius 0.076f
#define Chassis_L 0.365f
#define Chassis_W 0.385f

typedef struct{
    MyMotor_3508_Type *LF;
    MyMotor_3508_Type *RF;
    MyMotor_3508_Type *LB;
    MyMotor_3508_Type *RB;
}MyMotor_3508_Type_Collection;

void SendChassis_ByRPM(MyMotor_3508_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);
void Chassis_Calculate(MyMotor_3508_Type_Collection *motor_collect,CAN_TypeDef *CANx,int16_t id,float Vx,float Vy,float AngularVelocity,float L,float W);
#endif
