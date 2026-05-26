#include "stm32f4xx.h"
#include "handle.h"



/**
 * @description: 
 * @param {MyMotor_3508_Crane_Type_Collection} *motor_collect
 * @param {CAN_TypeDef} *CANx
 * @param {int16_t} id
 * @param {int16_t} i_201   X1电机
 * @param {int16_t} i_202   Y1电机
 * @param {int16_t} i_203   Y2电机
 * @return {*}
 */
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


/**
 * @description: 混合位置环与速度斜坡环的龙门架电机控制
 * @param {MyMotor_3508_Type} *motor
 * @param {int16_t} rc_val          遥控器值
 * @param {int64_t} max_position    位置最大值
 * @param {int64_t} min_position    位置最小值
 * @param {float} motor_speed_up    匀速上升RPM
 * @param {float} motor_speed_down  匀速下降RPM
 * @param {float} ramp_acc_step     斜坡加速步长
 * @param {float} ramp_dec_step     斜坡减速步长
 * @return {int16_t} final_speed_cmd 最终速度RPM
 */
int16_t Crane_Control_Loop(MyMotor_3508_Type *motor, int16_t rc_val, 
    int64_t max_position, int64_t min_position, 
    float motor_speed_up, float motor_speed_down,
    float ramp_acc_step, float ramp_dec_step)
{
    float final_speed_cmd = 0.0f; // 最终给到 SendCrane_ByRPM 的速度期望
    int32_t current_ecd = motor->total_ecd; // 当前绝对位置

    // ================== 1. 状态与模式切换状态机 ==================
    
    // 【情况 A】：检测到越界（触顶或触底），强制进入位置抱死模式
    // 减去50防止反复纠偏
    if (current_ecd >= max_position-50 && rc_val > 100) 
    {
        motor->crane_mode = MODE_POSITION_HOLD;
        motor->target_pos = max_position; // 强制锁死在上限
    }
    // 加上50防止反复纠偏
    else if (current_ecd <= min_position+50 && rc_val < -100) 
    {
        motor->crane_mode = MODE_POSITION_HOLD;
        motor->target_pos = min_position; // 强制锁死在下限
    }
    // 【情况 B】：遥控器推出去，且未越界，进入正常的匀速赶路模式
    else if (rc_val > 100 || rc_val < -100) 
    {
        motor->crane_mode = MODE_SPEED_RAMP;
    }
    // 【情况 C】：遥控器回中（想要悬停）
    else 
    {
        // 关键：只有从赶路模式刚停下来的一瞬间，才捕捉一次位置，防止重复刷新
        if (motor->crane_mode == MODE_SPEED_RAMP) 
        {
            motor->target_pos = current_ecd; // 锁死当前这一帧的绝对位置
        }
        motor->crane_mode = MODE_POSITION_HOLD;
    }

    // ================== 2. 根据不同模式计算期望速度 ==================
    
    if (motor->crane_mode == MODE_SPEED_RAMP) 
    {
        // ------ 速度斜坡模式 ------
        float final_target_speed = (rc_val > 100) ? motor_speed_up : motor_speed_down;
        
        // 斜坡函数过渡（加速步长2，减速步长4）
        if (motor->ramp_speed < final_target_speed) {
            motor->ramp_speed += ramp_acc_step;
            if (motor->ramp_speed > final_target_speed) motor->ramp_speed = final_target_speed;
        } 
        else if (motor->ramp_speed > final_target_speed) {
            motor->ramp_speed -= ramp_dec_step;
            if (motor->ramp_speed < final_target_speed) motor->ramp_speed = final_target_speed;
        }
        
        final_speed_cmd = motor->ramp_speed;
    } 
    else 
    {
        // ------ 位置抱死模式（悬停/MIN/MAX） ------
        motor->ramp_speed = 0.0f; // 清空速度斜坡的历史值
        
        // 位置外环计算：输入目标位置和当前位置，输出“为了纠偏应该达到的目标速度”
        // 注意：位置外环 pid_position 的参数通常只需要 P，千万不要给 I
        final_speed_cmd = PID_Calculate(motor->Motor_Position_PID, (float)motor->target_pos, (float)current_ecd);
        
        // 限制位置外环输出的最大速度，防止纠偏时猛烈暴冲（限制在 100 rpm 左右即可）
        if (final_speed_cmd > 100.0f)  final_speed_cmd = 100.0f;
        if (final_speed_cmd < -100.0f) final_speed_cmd = -100.0f;

        // 增加死区：如果误差很小，直接停止纠偏，防止在目标点附近反复微调导致抽搐
        if (abs(motor->target_pos - current_ecd) < 50) {
            final_speed_cmd = 0.0f;
        }
    }

    // 3. 积分清零（消除抽搐特效药）：
    // 当处于位置抱死模式时，如果位置误差已经很小，或者在限位处，强制清零速度环积分
    // 避免积分项在临界点累积导致的高频抖动
    if (motor->crane_mode == MODE_POSITION_HOLD) {
        if (motor->Motor_PID != NULL) {
            motor->Motor_PID->output_I = 0.0f;
        }
    }

    return (int16_t)final_speed_cmd;
}

/**
 * @description: 
 * @param {MyMotor_3508_Crane_Type_Collection} *motor_collect
 * @param {int16_t} rc_val1 遥控器值1->x1
 * @param {int16_t} rc_val2 遥控器值2->y1
 * @param {int16_t} rc_val3 遥控器值3->y2
 * @return {*}
 */
void Crane_Calculate(MyMotor_3508_Crane_Type_Collection *motor_collect,int16_t rc_val1,int16_t rc_val2,int16_t rc_val3){
    int16_t output_X1=Crane_Control_Loop(motor_collect->CRANE_X1,rc_val1,X1_MAX_POSITION,X1_MIN_POSITION,X1_MOTOR_SPEED_UP,X1_MOTOR_SPEED_DOWN,X1_RAMP_ACC_STEP,X1_RAMP_DEC_STEP);
    int16_t output_Y1=Crane_Control_Loop(motor_collect->CRANE_Y1,rc_val2,Y1_MAX_POSITION,Y1_MIN_POSITION,Y1_MOTOR_SPEED_UP,Y1_MOTOR_SPEED_DOWN,Y1_RAMP_ACC_STEP,Y1_RAMP_DEC_STEP);
    int16_t output_Y2=Crane_Control_Loop(motor_collect->CRANE_Y2,rc_val3,Y2_MAX_POSITION,Y2_MIN_POSITION,Y2_MOTOR_SPEED_UP,Y2_MOTOR_SPEED_DOWN,Y2_RAMP_ACC_STEP,Y2_RAMP_DEC_STEP);
    SendCrane_ByRPM(motor_collect,CAN1,0x1FF,output_X1,output_Y1,output_Y2);
}