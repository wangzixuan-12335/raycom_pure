#include "stm32f4xx.h"
#include "handle.h"

/**
 * @description: 
 * @param {CAN_TypeDef} *CANx 
 * @param {int16_t} id          //CAN报头
 * @param {int16_t} i_201       //左前 (Left Front) RPM
 * @param {int16_t} i_202       //右前 (Right Front) RPM
 * @param {int16_t} i_203       //左后 (Left Back) RPM
 * @param {int16_t} i_204       //右后 (Right Back) RPM
 * @return {*}
 */
void SendChassis_ByRPM(CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204){
    float output_LF = PID_Calculate(Motor_3508_LF.Motor_PID, i_201, (float)Motor_3508_LF.Rotor_Speed);
    float output_RF = PID_Calculate(Motor_3508_RF.Motor_PID, i_202, (float)Motor_3508_RF.Rotor_Speed);
    float output_LB = PID_Calculate(Motor_3508_LB.Motor_PID, i_203, (float)Motor_3508_LB.Rotor_Speed);
    float output_RB = PID_Calculate(Motor_3508_RB.Motor_PID, i_204, (float)Motor_3508_RB.Rotor_Speed);
    Can_Send(CANx,id,(int16_t)output_LF,(int16_t)output_RF,(int16_t)output_LB,(int16_t)output_RB);
}