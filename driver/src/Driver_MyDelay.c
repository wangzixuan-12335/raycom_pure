#include "Driver_MyDelay.h"
#include "stm32f4xx.h"

static uint32_t ticks_per_us = 0; // 每微秒需要的计数值
static uint32_t ticks_per_ms = 0; // 每毫秒需要的计数值

/**
 * @brief  初始化系统滴答定时器（SysTick）
 * @note   让定时器在后台永久运行，不占用常规硬件定时器
 */
void My_delay_init(void) {
    // 1. 选择内核时钟源（HCLK = 72MHz）
    // 计数器每秒会计数 72,000,000 次
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    
    // 2. 计算每微秒和每毫秒对应的 ticks 数量
    ticks_per_us = SystemCoreClock / 1000000; // 72MHz下 = 72
    ticks_per_ms = ticks_per_us * 1000;       // 72MHz下 = 72000
    
    // 3. 设置 SysTick 的自动重装载值为最大值 (24位寄存器最大值为 0xFFFFFF)
    SysTick->LOAD = 0xFFFFFF;
    
    // 4. 清空当前计数值
    SysTick->VAL = 0x00;
    
    // 5. 开启定时器（第0位ENABLE=1，且关闭SysTick中断，纯用作后台计数器）
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief  绝对安全的微秒阻塞延时（防中断打断）
 * @param  nus: 需要延时的微秒数
 */
void My_delay_us(uint32_t nus) {
    uint32_t total_ticks = nus * ticks_per_us; // 算出总共需要走过多少个 tick
    uint32_t start_val = SysTick->VAL;         // 记录刚进来时的当前计数值
    uint32_t current_val;
    uint32_t elapsed_ticks;

    while (1) {
        current_val = SysTick->VAL; // 实时获取当前的计数值
        
        // 由于 SysTick 是向下倒计数的（从大变小）
        if (current_val <= start_val) {
            // 情况一：没有发生自动重装（平稳运行）
            elapsed_ticks = start_val - current_val;
        } else {
            // 情况二：发生了解析度回绕（数到0后，自动回到了0xFFFFFF）
            elapsed_ticks = (SysTick->LOAD - current_val) + start_val;
        }
        
        // 如果走过的 ticks 已经达到了我们预设的目标，直接退出
        if (elapsed_ticks >= total_ticks) {
            break;
        }
    }
}

/**
 * @brief  绝对安全的毫秒阻塞延时
 * @param  nms: 需要延时的毫秒数
 */
void My_delay_ms(uint32_t nms) {
    // 毫秒延时同理，为了防止 nms 过大导致 ticks 溢出 24 位寄存器范围，分段进行
    while (nms--) {
        delay_us(1000);
    }
}