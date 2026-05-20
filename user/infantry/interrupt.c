/**
 * @brief  中断服务函数根据地
 */
#include "handle.h"

// EXTI9_5 陀螺仪中断
void EXTI9_5_IRQHandler(void) {
    uint8_t suc;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        xSemaphoreGiveFromISR(ImuDataReady, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DBus空闲中断(USART1)
void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        /**
         * 【核心：清除空闲中断标志位】
         * 必须先读 SR 寄存器，再读 DR 寄存器，这是 F427 硬件手册规定的清除序列
         */
        volatile uint32_t temp;
        temp = USART1->SR;
        temp = USART1->DR;

        // 1. 停止 DMA 数据流，准备处理缓存
        DMA_Cmd(DMA2_Stream2, DISABLE);

        // 2. 计算实际接收到的字节数
        // 这里的 18 必须对应你设置 DMA_BufferSize 的值
        uint16_t rx_len = 18 - DMA_GetCurrDataCounter(DMA2_Stream2);

        // 3. 长度校验：只有刚好收到 18 字节才认为是合法的 DBUS 数据包        
        DBUS_Decode(usart1_raw_data, usart1_data_decoded);

        // 4. 重置 DMA 计数器并重新开启
        // 必须先置 18，再使能，DMA 才能重新开始从 buf[0] 接收
        DMA_SetCurrDataCounter(DMA2_Stream2, 18);
        DMA_Cmd(DMA2_Stream2, ENABLE);
        USART_ClearITPendingBit(USART1,USART_IT_IDLE);
    }
}
/**
 * @brief USART3 串口中断
 * @note  视觉系统读取
 */
void USART3_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 3);
}

/**
 * @brief USART6 串口中断
 * @note  裁判系统读取
 */
void USART6_IRQHandler(void) {
        Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 6);
}

/**
 * @brief UART7 串口中断
 */
void UART7_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 7);
}

/**
 * @brief UART8 串口中断
 */
void UART8_IRQHandler(void) {
   Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 8);
}

// CAN1数据接收中断服务函数
void CAN1_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN1_BRIDGE);
}

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN2_BRIDGE);
}

// TIM2 高频计数器
extern volatile uint32_t ulHighFrequencyTimerTicks;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        ulHighFrequencyTimerTicks++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @return None
 */

void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @return None
 */
void hardfault_c(uint32_t *sp);
	
__ASM void HardFault_Handler(void)
{
		IMPORT hardfault_c;
	
    TST     LR, #4
    ITE     EQ
    MRSEQ   R0, MSP
    MRSNE   R0, PSP
    B       hardfault_c
}

void hardfault_c(uint32_t *sp)
{
    volatile uint32_t pc  = sp[6];
    volatile uint32_t lr  = sp[5];
    volatile uint32_t cfsr  = SCB->CFSR;
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t mmfar = SCB->MMFAR;
    volatile uint32_t bfar  = SCB->BFAR;

    __BKPT(0);
    while (1) {}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @return None
 */
void MemManage_Handler(void) {
     while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @return None
 */
void BusFault_Handler(void) {
   while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @return None
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @return None
 */
void DebugMon_Handler(void) {
    while (1) {
    }
}

// /**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @return None
//  */
// void SVC_Handler(void) {
//     //while(1){}
// }

// /**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @return None
//  */
// void PendSV_Handler(void) {
//    //while(1){}
// }

// /**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @return None
//  */
// void SysTick_Handler(void) {
//      //while(1){}
// }
