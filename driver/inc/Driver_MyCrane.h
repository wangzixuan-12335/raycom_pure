#ifndef __MY_CRANE_H
#define __MY_CRANE_H
#include "stm32f4xx.h"
#include "Driver_MyMotor3508.h"

#define LIFT_MAX_POS     240000    // 软限位上限
#define LIFT_MIN_POS     0       // 软限位下限
#define MOTOR_SPEED_UP   100.0f    // 匀速上升设定值 (rpm)
#define MOTOR_SPEED_DOWN -100.0f   // 匀速下降设定值 (rpm)

// 斜坡加减速步长配置（根据实际龙门架重量调节）
// 假设控制循环是 1ms 运行一次，每次最多加/减 2.0 rpm 
// 从 0 启动到 600 rpm 只需要 300ms，既丝滑又不会感觉迟钝
#define RAMP_ACC_STEP    2.0f      // 加速度步长
#define RAMP_DEC_STEP    4.0f      // 减速度步长（减速稍微快一点，刹车更灵敏）

typedef struct{
    MyMotor_3508_Type *CRANE_X1;
    MyMotor_3508_Type *CRANE_Y1;
    MyMotor_3508_Type *CRANE_Y2;
}MyMotor_3508_Crane_Type_Collection;

void SendCrane_ByRPM(MyMotor_3508_Crane_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203);

void Crane_Control_Loop(MyMotor_3508_Crane_Type_Collection *motor_collect,int16_t rc_val);

#endif
