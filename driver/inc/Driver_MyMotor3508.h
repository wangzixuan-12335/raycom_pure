#include "stm32f4xx.h"
#include "handle.h"
#ifndef __MY_MOTOR_3508
#define __MY_MOTOR_3508


typedef struct
{
    uint16_t Mechanical_Angle;      //转子机械角度 0-8191
    int16_t Rotor_Speed;            //转子转速(rpm)
    int16_t Actual_Torque_Current;  //实际转矩电流
    int8_t Motor_Temperature;       //电机温度(摄氏度)
    PID_Type Motor_PID;             //电机pid结构体
}MyMotor_3508_Type;

void Motor_decode_data(MyMotor_3508_Type *motor, uint8_t *rx_data);

#endif
