/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-21 19:44:38
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-22 21:42:13
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\driver\inc\Driver_MyMotor3508.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MY_MOTOR_3508
#define __MY_MOTOR_3508

#include "stm32f4xx.h"
#include "Driver_PID.h"
#define IsPositive_True 1
#define IsPositive_False -1

typedef struct {
    int8_t IsPositive;             //电机是否正转 IsPositive_True代表正转
    uint8_t ReductionRatio;         //电机减速比 3508自带19:1 
    uint16_t Mechanical_Angle;      //转子机械角度 0-8191
    int16_t Rotor_Speed;            //转子转速(rpm)
    int16_t Actual_Torque_Current;  //实际转矩电流
    int8_t Motor_Temperature;       //电机温度(摄氏度)
    PID_Type *Motor_PID;            //电机pid结构体
}MyMotor_3508_Type;

void Motor_decode_data(MyMotor_3508_Type *motor, uint8_t *rx_data);

void Motor_3508_Init(MyMotor_3508_Type *motor,int8_t IsPositive,uint8_t ReductionRatio);

#endif
