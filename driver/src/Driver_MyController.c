#include "stm32f4xx.h"
#include "handle.h"

void MyController_Move(usart1_data_decoded_type *data){
    switch (data->s2)
    {
    case 1:
        //龙门架
        Crane_Calculate(&Motor_3508_Gantry_Crane_Collection,data->r2y,data->r1y,data->r1y);
        break;
    case 3:
        //底盘
        Chassis_Calculate(&MyMotor_3508_Collection,CAN1,0x200,(float)data->r1y/1000.0f,(float)data->r1x/1000.0f,(float)data->r2x/330.0f,Chassis_L,Chassis_W);
        break;
	case 2:
        //哪怕软件掉电也给值，防止3508掉CAN发疯
		Can_Send(CAN1,0x200,0,0,0,0);
        Can_Send(CAN1,0x1FF,0,0,0,0);
		break;
    default:
        break;
    }
    SetServoByController(data->s1);
}

void MyController_Stay(usart1_data_decoded_type *data){
    switch (data->s2)
    {
    case 1:
        //给一个0保持底盘不掉PID
        Chassis_Calculate(&MyMotor_3508_Collection,CAN1,0x200,0,0,0,Chassis_L,Chassis_W);
        break;
    case 3:
        //给一个0保持龙门架角度，防止切换掉PID
        Crane_Calculate(&Motor_3508_Gantry_Crane_Collection,0,0,0);
        break;
    default:
        break;
    }
}