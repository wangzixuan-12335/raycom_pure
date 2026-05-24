#include "stm32f4xx.h"
#include "handle.h"

/**
 * @description: CAN解码
 * @param {MyMotor_3508_Type} *motor    //电机结构体
 * @param {uint8_t} *rx_data            //CAN接收的数据
 * @return {*}
 */
void Motor_decode_data(MyMotor_3508_Type *motor, uint8_t *rx_data){
    motor->Mechanical_Angle=(rx_data[0] << 8) | rx_data[1];
    motor->Rotor_Speed=(int16_t)((rx_data[2] << 8) | rx_data[3]);
    motor->Actual_Torque_Current=(int16_t)((rx_data[4] << 8) | rx_data[5]);
    motor->Motor_Temperature=(int8_t)rx_data[6];
    if(motor->IsActive==0){
        motor->Initial_Angle=motor->Mechanical_Angle;
        motor->Mechanical_Angle_last=motor->Mechanical_Angle;
        motor->IsActive=1;
    }
}

/**
 * @description: 初始化电机结构体
 * @param {MyMotor_3508_Type} *motor
 * @param {int8_t} IsPositive           是否正转
 * @param {uint8_t} ReductionRatio      减速比
 * @param {PID_Type} *speed_PID          速度环PID
 * @param {PID_Type} *position_PID       位置环环PID
 * @return {*}
 */
void Motor_3508_Init(MyMotor_3508_Type *motor,int8_t IsPositive,uint8_t ReductionRatio,PID_Type *speed_PID,PID_Type *position_PID){
    motor->IsPositive=IsPositive;
    motor->ReductionRatio=ReductionRatio;
    motor->IsActive=0;
    motor->Mechanical_Angle=0;
    motor->Initial_Angle=0;
    motor->Mechanical_Angle_last=0;
    motor->ECD_count=0;
    motor->total_ecd=0;
    motor->Rotor_Speed=0;
    motor->Actual_Torque_Current=0;
    motor->Motor_PID=speed_PID;
    motor->Motor_Position_PID=position_PID;
    motor->crane_mode=MODE_POSITION_HOLD;
    motor->target_pos=0;
    motor->ramp_speed=0.0f;
}

/**
 * @description: 计算连续角
 * @param {MyMotor_3508_Type} *motor 电机结构体
 * @return {*}
 */
void Update_3508_Continuous_Angle(MyMotor_3508_Type *motor){
    //获取转动差值
    int32_t diff=motor->Mechanical_Angle - motor->Mechanical_Angle_last;
    
    if (diff < -4096) {
        //说明正转过零（比如从 8100 变到了 100，diff = -8000
        if(motor->IsPositive==IsPositive_True){
            motor->ECD_count++;
        }else{
            motor->ECD_count--;
        }
        
    }else if (diff > 4096) {
        //说明反转过零（比如从 100 变到了 8100，diff = 8000）
        if(motor->IsPositive==IsPositive_True){
            motor->ECD_count--;
        }else{
            motor->ECD_count++;
        }
    }

    // 把这一次的新角度写到last中
    motor->Mechanical_Angle_last=motor->Mechanical_Angle;

    motor->total_ecd=motor->ECD_count*8192 + motor->IsPositive*(motor->Mechanical_Angle - motor->Initial_Angle);
}