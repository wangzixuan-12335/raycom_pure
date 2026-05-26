#ifndef __MY_CRANE_H
#define __MY_CRANE_H
#include "stm32f4xx.h"
#include "Driver_MyMotor3508.h"

//X1电机
#define X1_MAX_POSITION         240000
#define X1_MIN_POSITION         0
#define X1_MOTOR_SPEED_UP       100.0f
#define X1_MOTOR_SPEED_DOWN     -100.0f
#define X1_RAMP_ACC_STEP        2.0f
#define X1_RAMP_DEC_STEP        4.0f

//Y1电机
#define Y1_MAX_POSITION         240000
#define Y1_MIN_POSITION         0
#define Y1_MOTOR_SPEED_UP       100.0f
#define Y1_MOTOR_SPEED_DOWN     -100.0f
#define Y1_RAMP_ACC_STEP        2.0f
#define Y1_RAMP_DEC_STEP        4.0f

//Y2电机
#define Y2_MAX_POSITION         240000
#define Y2_MIN_POSITION         0
#define Y2_MOTOR_SPEED_UP       100.0f
#define Y2_MOTOR_SPEED_DOWN     -100.0f
#define Y2_RAMP_ACC_STEP        2.0f
#define Y2_RAMP_DEC_STEP        4.0f

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
