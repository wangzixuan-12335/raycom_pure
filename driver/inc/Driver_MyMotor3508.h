/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-21 19:44:38
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-21 23:11:54
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\driver\inc\Driver_MyMotor3508.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MY_MOTOR_3508
#define __MY_MOTOR_3508

#include "stm32f4xx.h"
#include "Driver_PID.h"

typedef struct {
    uint16_t Mechanical_Angle;      //转子机械角度 0-8191
    int16_t Rotor_Speed;            //转子转速(rpm)
    int16_t Actual_Torque_Current;  //实际转矩电流
    int8_t Motor_Temperature;       //电机温度(摄氏度)
    PID_Type *Motor_PID;            //电机pid结构体
}MyMotor_3508_Type;

void Motor_decode_data(MyMotor_3508_Type *motor, uint8_t *rx_data);

#endif
