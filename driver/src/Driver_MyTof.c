#include "Driver_MyTof.h"
#include "stm32f4xx.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "Driver_MyI2C.h"
#include "Driver_MyDelay.h"
#include "handle.h"

uint16_t dev_address = TOF_DEV_ADDR;

void TOF_XSHUT_GPIO_Init(void)
{
    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(TOF_XSHUT_RCC_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(TOF_VCC_3V3_RCC_CLK, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = TOF_XSHUT_PIN; // 改成你板子实际连接的引脚！
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(TOF_XSHUT_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=TOF_VCC_3V3_PIN;
    GPIO_Init(TOF_VCC_3V3_PORT, &GPIO_InitStructure);
}

void TOF_Init(void){
    uint8_t byteData;
    uint8_t sensorState = 0;
    int8_t status = 0;
    distance_mm = 0; // 用来存放最终的测距结果（单位：毫米）
    // 【第一步】初始化底层的软件 I2C 引脚
    MyI2C_Init(); 

    TOF_XSHUT_GPIO_Init();
    GPIO_SetBits(TOF_VCC_3V3_PORT, TOF_VCC_3V3_PIN);

    GPIO_ResetBits(TOF_XSHUT_PORT, TOF_XSHUT_PIN); // 强行拉低 XSHUT，把传感器砸进休眠
    My_delay_ms(20);                   // 让它睡 20ms，彻底死透
    
    GPIO_SetBits(TOF_XSHUT_PORT, TOF_XSHUT_PIN);   // 重新拉高 XSHUT，唤醒传感器！
    My_delay_ms(20);                   // 给它 20ms 的时间“起床”睁眼

    // 【第二步】等待传感器硬件启动就绪
    // 传感器上电后内部数字核心初始化需要时间，必须循环读取其状态，直到 sensorState == 1
    while(sensorState == 0){
        status = VL53L1X_BootState(dev_address, &sensorState);
        if(status != 0) {
            // 如果这里进去了，说明 I2C 连传感器都没检测到，检查接线或上拉电阻！
            break; 
        }
        My_delay_ms(2);
    }

    // 【第三步】传感器核心初始化（加载固件默认调谐参数）
    status = VL53L1X_SensorInit(dev_address);

    // 【第四步】配置测距参数
    // 距离模式设置：1 = 短距离模式（最大~1.3米，抗环境光好）；2 = 长距离模式（最大~4米，RM赛场推荐）
    status = VL53L1X_SetDistanceMode(dev_address, 1); 
    
    // 设置定时预算（Timing Budget）：单次测距允许的时间（毫秒）。
    // 可选：15, 20, 33, 50, 100, 200ms。时间越长越精准，但刷新率变低。步兵壁障推荐 33ms 或 50ms。
    status = VL53L1X_SetTimingBudgetInMs(dev_address, 100); 
    
    // 设置两次测距之间的间隔周期。必须大于或等于上面的定时预算！
    status = VL53L1X_SetInterMeasurementInMs(dev_address, 110); 

    // 设置FOV角度
    //VL53L1X_SetROI(dev_address, 4, 4);

    // 【第五步】开启测距使能
    status = VL53L1X_StartRanging(dev_address);
}

void TOF_Loop_Read(void){
    uint8_t dataReady = 0;
    int8_t status;

    // 数据完成置标志位，不死等
    status = VL53L1X_CheckForDataReady(dev_address, &dataReady);

    // 2. 如果 dataReady == 1，说明最新一帧的激光数据已经完成
    if (status == 0 && dataReady == 1) 
    {
        // 读取具体的全标距离（单位：mm）
        status = VL53L1X_GetDistance(dev_address, &distance_mm);
        
        //读完数据清除标志位
        status = VL53L1X_ClearInterrupt(dev_address);
    }
}