#include "stm32f4xx.h"
#include "Driver_MyMotor3508.h"

void Motor_decode_data(MyMotor_3508_Type *motor, uint8_t *rx_data){
    motor->Mechanical_Angle=(rx_data[0] << 8) | rx_data[1];
    motor->Rotor_Speed=(int16_t)((rx_data[2] << 8) | rx_data[3]);
    motor->Actual_Torque_Current=(int16_t)((rx_data[4] << 8) | rx_data[5]);
    motor->Motor_Temperature=(int8_t)rx_data[6];
}