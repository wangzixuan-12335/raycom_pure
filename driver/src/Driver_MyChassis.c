#include "stm32f4xx.h"
#include "handle.h"

/**
 * @description: 线速度转换角速度
 * @param {float} v 线速度
 * @param {float} r 轮子半径
 * @return {*}  角速度(rpm)
 */
int16_t Velocity2RPM(float v,float r){
    float rpm=(v*60.0f)/(2*PI*r);
    return (int16_t)rpm;
}

/**
 * @description: 
 * @param {MyMotor_3508_Type_Collection} *motor //电机集合结构体
 * @param {CAN_TypeDef} *CANx 
 * @param {int16_t} id          //CAN报头
 * @param {int16_t} i_201       //左前 (Left Front) RPM
 * @param {int16_t} i_202       //右前 (Right Front) RPM
 * @param {int16_t} i_203       //左后 (Left Back) RPM
 * @param {int16_t} i_204       //右后 (Right Back) RPM
 * @return {*}
 */
void SendChassis_ByRPM(MyMotor_3508_Type_Collection *motor_collect,CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204){
    //计算考虑正反转和减速比
    float output_LF = PID_Calculate(Motor_3508_LF.Motor_PID, i_201*motor_collect->LF->ReductionRatio*motor_collect->LF->IsPositive, (float)Motor_3508_LF.Rotor_Speed);
    float output_RF = PID_Calculate(Motor_3508_RF.Motor_PID, i_202*motor_collect->RF->ReductionRatio*motor_collect->RF->IsPositive, (float)Motor_3508_RF.Rotor_Speed);
    float output_LB = PID_Calculate(Motor_3508_LB.Motor_PID, i_203*motor_collect->LB->ReductionRatio*motor_collect->LB->IsPositive, (float)Motor_3508_LB.Rotor_Speed);
    float output_RB = PID_Calculate(Motor_3508_RB.Motor_PID, i_204*motor_collect->RB->ReductionRatio*motor_collect->RB->IsPositive, (float)Motor_3508_RB.Rotor_Speed);
    Can_Send(CANx,id,(int16_t)output_LF,(int16_t)output_RF,(int16_t)output_LB,(int16_t)output_RB);
}

/**
 * @description: 底盘正解算
 * @param {MyMotor_3508_Type_Collection} *motor_collect 电机集合结构体
 * @param {CAN_TypeDef} *CANx CANX
 * @param {int16_t} id  报文ID                              
 * @param {float} Vx    Vx速度(m/s)
 * @param {float} Vy    Vy速度(m/s)
 * @param {float} AngularVelocity  角速度rad/s 
 * @param {float} L     轴距(m)
 * @param {float} W     轮距(m)
 * @return {*}
 */
void Chassis_Calculate(MyMotor_3508_Type_Collection *motor_collect,CAN_TypeDef *CANx,int16_t id,float Vx,float Vy,float AngularVelocity,float L,float W){
    float K=(W+L)/2.0f;
    float V_LF,V_RF,V_LB,V_RB;
    V_LF=Vx+Vy+AngularVelocity*K;
    V_RF=Vx-Vy-AngularVelocity*K;
    V_LB=Vx-Vy+AngularVelocity*K;
    V_RB=Vx+Vy-AngularVelocity*K;
    
    SendChassis_ByRPM(motor_collect,CANx,id,Velocity2RPM(V_LF,Wheel_radius),Velocity2RPM(V_RF,Wheel_radius),Velocity2RPM(V_LB,Wheel_radius),Velocity2RPM(V_RB,Wheel_radius));
}