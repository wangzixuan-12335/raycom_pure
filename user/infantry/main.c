#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"



void main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //初始化上下位机通信,初始化信息结构体
    BSP_USART2_Init(115200,0);
    Lower_Computer_Message[0]=0xAA;
    Lower_Computer_Message[7]=0x0D;
    Lower_Computer_Message[8]=0x0A;
    //初始化防止值一开始乱飞
    distance_x_mm=0;
    distance_y_mm=0;
    
    //初始化Delay和I2C
    My_delay_init();
    MyI2C_Init();

    //激光测距初始化 
    TOF_Init();
    
    //遥控器USART+DMA转运, 
    //USART在中断里面处理解码，解码数据在结构体usart1_data_decoded中
    MY_USART_Init();
    MyDMA_Init((uint32_t)&(USART1->DR),(uint32_t)usart1_raw_data,18);
	
    //初始化夹爪舵机结构体，PWM初始化
    //BSP_PWM_Init内TIM_OCMode_PWM2 修改为 TIM_OCMode_PWM1
    BSP_PWM_Set_Port(&PWM_Holding_Jaw_Servo,PWM_PD12);
    BSP_PWM_Init(&PWM_Holding_Jaw_Servo,1800,1000,TIM_OCPolarity_High);

    //初始化底盘电机结构体
    Motor_3508_Init(&Motor_3508_LF,IsPositive_True,19,&Motor_3508_LF_PID,&Motor_3508_LF_Position_PID);
    Motor_3508_Init(&Motor_3508_RF,IsPositive_False,19,&Motor_3508_RF_PID,&Motor_3508_RF_Position_PID);
    Motor_3508_Init(&Motor_3508_LB,IsPositive_True,19,&Motor_3508_LB_PID,&Motor_3508_LB_Position_PID);
    Motor_3508_Init(&Motor_3508_RB,IsPositive_False,19,&Motor_3508_RB_PID,&Motor_3508_RB_Position_PID);
    //初始化龙门架电机结构体
    Motor_3508_Init(&Motor_3508_Gantry_Crane_X1,IsPositive_False,19,&Motor_3508_Gantry_Crane_X1_PID,&Motor_3508_Gantry_Crane_X1_Position_PID);
    Motor_3508_Init(&Motor_3508_Gantry_Crane_Y1,IsPositive_True,19,&Motor_3508_Gantry_Crane_Y1_PID,&Motor_3508_Gantry_Crane_Y1_Position_PID);
    Motor_3508_Init(&Motor_3508_Gantry_Crane_Y2,IsPositive_False,19,&Motor_3508_Gantry_Crane_Y2_PID,&Motor_3508_Gantry_Crane_Y2_Position_PID);

    //初始化底盘MOTOR3508的pid
    PID_Init(Motor_3508_LF.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_RF.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_LB.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_RB.Motor_PID,7.5,0.1,0.5,10000,3000);
    //初始化龙门架MOTOR3508的pid
    PID_Init(Motor_3508_Gantry_Crane_X1.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_Gantry_Crane_Y1.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_Gantry_Crane_Y2.Motor_PID,7.5,0.1,0.5,10000,3000);
    PID_Init(Motor_3508_Gantry_Crane_X1.Motor_Position_PID,0.05,0,0,10000,3000);
    PID_Init(Motor_3508_Gantry_Crane_Y1.Motor_Position_PID,0.05,0,0,10000,3000);
    PID_Init(Motor_3508_Gantry_Crane_Y2.Motor_Position_PID,0.05,0,0,10000,3000);
    
    //底盘电机结构体合集
    MyMotor_3508_Collection.LF=&Motor_3508_LF;
    MyMotor_3508_Collection.RF=&Motor_3508_RF;
    MyMotor_3508_Collection.LB=&Motor_3508_LB;
    MyMotor_3508_Collection.RB=&Motor_3508_RB;
    //龙门架电机结构体合集
    Motor_3508_Gantry_Crane_Collection.CRANE_X1=&Motor_3508_Gantry_Crane_X1;
    Motor_3508_Gantry_Crane_Collection.CRANE_Y1=&Motor_3508_Gantry_Crane_Y1;
    Motor_3508_Gantry_Crane_Collection.CRANE_Y2=&Motor_3508_Gantry_Crane_Y2;

    //龙门架自动运行到软件最低限位处
    MyCrane_Make_Zero(&Motor_3508_Gantry_Crane_Collection);

    //初始化CAN通信
    //CAN接收写在CAN1中断内，根据不同报文头将数据写到结构体(eg)Motor_3508_LF中
    //CAN发送写在TIM2的定时(1kHZ)的中断中
    //can发送内有死循环，记得加超时退出(已修改)
    BSP_CAN_Init();

    //初始化TIM2，生成一个1KHZ的中断，主要业务写在这个中断中，要保证所有对象初始化完毕，故应最后开启
    //BSP_TIM2_Init经过修改，改了分频器，GPIOA初始化被注释
    BSP_TIM2_Init();

    //初始化TIM3，生成一个0.5KHZ的中断，保持电机和底盘角度写在这个中断中
    //若写在TIM2中会导致CAN发送不稳，导致电机过冲
    BSP_TIM3_Init();

    //改变夹爪舵机角度
    //My_Servo_ChangeAngle(&PWM_Holding_Jaw_Servo,180);

    while (1)
    {
        //不断读TOF激光测距值
        TOF_Loop_Read();

        distance_x_mm=Calculate_Crane_X_distance(&Motor_3508_Gantry_Crane_Collection);
        distance_y_mm=Calculate_Crane_Y_distance(&Motor_3508_Gantry_Crane_Collection);

        Lower_Computer_Message[1]=(uint8_t)(distance_chassis_mm>>8);
        Lower_Computer_Message[2]=(uint8_t)(distance_chassis_mm & 0xFF);

        Lower_Computer_Message[3]=(uint8_t)(distance_x_mm>>8);
        Lower_Computer_Message[4]=(uint8_t)(distance_x_mm & 0xFF);

        Lower_Computer_Message[5]=(uint8_t)(distance_y_mm>>8);
        Lower_Computer_Message[6]=(uint8_t)(distance_y_mm & 0xFF);
        
        Serial_SendArray(Lower_Computer_Message,9);
    }
    
}

