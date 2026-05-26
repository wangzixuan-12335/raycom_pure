#include "stm32f4xx.h"
#include "Driver_MySerial.h"

void Serial_SendByte(uint8_t Byte)
{
    uint32_t timeout = 0xFFFF;
	USART_SendData(MY_USARTx, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(MY_USARTx, USART_FLAG_TXE) == RESET && timeout > 0){
        timeout--;
    }	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
