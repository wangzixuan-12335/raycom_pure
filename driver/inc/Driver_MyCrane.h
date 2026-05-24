#ifndef __MY_CRANE_H
#define __MY_CRANE_H
#include "stm32f4xx.h"
#include "Driver_MyMotor3508.h"

typedef struct{
    MyMotor_3508_Type *CRANE_X1;
    MyMotor_3508_Type *CRANE_Y1;
    MyMotor_3508_Type *CRANE_Y2;
}MyMotor_3508_Crane_Type_Collection;

void SendCrane_ByRPM(MyMotor_3508_Crane_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203);

int16_t Crane_Control_Loop(MyMotor_3508_Type *motor, int16_t rc_val, 
    int64_t max_position, int64_t min_position, 
    float motor_speed_up, float motor_speed_down,
    float ramp_acc_step, float ramp_dec_step);

void Crane_Calculate(MyMotor_3508_Crane_Type_Collection *motor_collect,int16_t rc_val1,int16_t rc_val2,int16_t rc_val3);

#endif
