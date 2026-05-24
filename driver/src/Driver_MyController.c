#include "stm32f4xx.h"
#include "handle.h"

void MyController(usart1_data_decoded_type *data){
    switch (data->s2)
    {
    case 1:
        //龙门架
        Crane_Calculate(&Motor_3508_Gantry_Crane_Collection,data->r2y,0,0);
        break;
    case 3:
        //底盘
        Chassis_Calculate(&MyMotor_3508_Collection,CAN1,0x200,(float)data->r1y/1000.0f,(float)data->r1x/1000.0f,(float)data->r2x/330.0f,Chassis_L,Chassis_W);
        //给一个0保持龙门架角度，防止切换掉PID
        Crane_Calculate(&Motor_3508_Gantry_Crane_Collection,0,0,0);
        break;
    default:
        break;
    }
    SetServoByController(data->s1);
}