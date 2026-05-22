#include "stm32f4xx.h"
#include "Driver_MyMotor3508.h"

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
}

/**
 * @description: 初始化电机结构体
 * @param {MyMotor_3508_Type} *motor
 * @param {int8_t} IsPositive           //是否正转
 * @param {uint8_t} ReductionRatio      //减速比
 * @return {*}
 */
void Motor_3508_Init(MyMotor_3508_Type *motor,int8_t IsPositive,uint8_t ReductionRatio){
    motor->IsPositive=IsPositive;
    motor->ReductionRatio=ReductionRatio;
}