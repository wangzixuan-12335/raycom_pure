#include "stm32f4xx.h"
#include "handle.h"



void SendCrane_ByRPM(MyMotor_3508_Crane_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203){
    //计算考虑正反转和减速比
    float output_X1 = PID_Calculate(motor_collect->CRANE_X1->Motor_PID, 
        i_201*motor_collect->CRANE_X1->ReductionRatio*motor_collect->CRANE_X1->IsPositive, 
        (float)motor_collect->CRANE_X1->Rotor_Speed);
    float output_Y1 = PID_Calculate(motor_collect->CRANE_Y1->Motor_PID, 
        i_202*motor_collect->CRANE_Y1->ReductionRatio*motor_collect->CRANE_Y1->IsPositive, 
        (float)motor_collect->CRANE_Y1->Rotor_Speed);
    float output_Y2 = PID_Calculate(motor_collect->CRANE_Y2->Motor_PID, 
        i_203*motor_collect->CRANE_Y2->ReductionRatio*motor_collect->CRANE_Y2->IsPositive, 
        (float)motor_collect->CRANE_Y2->Rotor_Speed);

    Can_Send(CANx,id,(int16_t)output_X1,(int16_t)output_Y1,(int16_t)output_Y2,(int16_t)0);
}

// 静态全局变量或结构体成员，用于保存上一次的斜坡速度值
static float ramp_speed = 0.0f;

//测试！！！
void  Crane_Control_Loop(MyMotor_3508_Crane_Type_Collection *motor_collect, int16_t rc_val)
{
    float final_target_speed = 0.0f; // 遥控器最终想要的终点速度
    
    // 1. 基础解析：获取遥控器的原始期望目标
    if (rc_val > 100) {
        final_target_speed = MOTOR_SPEED_UP;   // 最终想要匀速上升
    } 
    else if (rc_val < -100) {
        final_target_speed = MOTOR_SPEED_DOWN; // 最终想要匀速下降
    } 
    else {
        final_target_speed = 0.0f;             // 最终想要悬停
    }
    
    // 2. 核心：无脑单向限位拦截（直接扼杀终点期望）
    if (motor_collect->CRANE_X1->total_ecd >= LIFT_MAX_POS && final_target_speed > 0) {
        final_target_speed = 0.0f; 
        motor_collect->CRANE_X1->Motor_PID->output_I = 0.0f; // 清空积分
    }
    if (motor_collect->CRANE_X1->total_ecd <= LIFT_MIN_POS && final_target_speed < 0) {
        final_target_speed = 0.0f; 
        motor_collect->CRANE_X1->Motor_PID->output_I = 0.0f; // 清空积分
    }
    
    // 3. 【核心进化】：斜坡函数加减速过渡
    // 如果当前斜坡速度还没达到遥控器的目标速度，则按照步长递增/递减
    if (ramp_speed < final_target_speed) 
    {
        ramp_speed += RAMP_ACC_STEP;
        if (ramp_speed > final_target_speed) {
            ramp_speed = final_target_speed; // 防止溢出越界
        }
    } 
    else if (ramp_speed > final_target_speed) 
    {
        ramp_speed -= RAMP_DEC_STEP;
        if (ramp_speed < final_target_speed) {
            ramp_speed = final_target_speed; // 防止越界
        }
    }

    // 4. 安全保护：如果龙门架已经在极限位置，且斜坡惯性还没降为0，为了绝对安全强制刹死
    if (motor_collect->CRANE_X1->total_ecd >= LIFT_MAX_POS && ramp_speed > 0) {
        ramp_speed = 0.0f;
    }
    if (motor_collect->CRANE_X1->total_ecd <= LIFT_MIN_POS && ramp_speed < 0) {
        ramp_speed = 0.0f;
    }
    
    // 5. 将通过斜坡规划、如丝般顺滑的 ramp_speed 送给底层执行函数
    SendCrane_ByRPM(motor_collect, CAN1, 0x1FF, ramp_speed, 0.0f, 0.0f);
}