#include "stm32f4xx.h"

void MyDMA_Init(uint32_t addrA,uint32_t addrB,uint32_t buffersize){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_DeInit(DMA2_Stream2);
    DMA_InitTypeDef DMA_InitStructure;

    DMA_InitStructure.DMA_Channel = DMA_Channel_4;          // 具体通道根据实际外设调整
    DMA_InitStructure.DMA_PeripheralBaseAddr = addrA;       // 外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = addrB;           // 存储器地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  // 传输方向
    DMA_InitStructure.DMA_BufferSize = buffersize;           // 传输缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream2, &DMA_InitStructure);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);

}

