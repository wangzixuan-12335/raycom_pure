/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "Driver_MyI2C.h"
#include "Driver_MyDelay.h"
#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	// 1. 发送 I2C 起始信号 (Start)
    MyI2C_Start();
    
    // 2. 发送设备地址（写模式）
    // dev 传入的是 7位地址左移一位后的值（即 0x52）。最低位为 0 代表写。
    MyI2C_SendByte(dev);
    if (MyI2C_ReceiveAck() != 0) { 
        MyI2C_Stop();
        return -1; // 器件没有响应，直接报错退出
    }
    
    // 3. 发送 16 位寄存器地址的高 8 位
    // 因为 VL53L1X 的寄存器是 16 位的，必须分两次发，高位在前（大端序）
    MyI2C_SendByte((index >> 8) & 0xFF);
    if (MyI2C_ReceiveAck() != 0) { 
        MyI2C_Stop(); 
        return -1; 
    }
    
    // 4. 发送 16 位寄存器地址的低 8 位
    MyI2C_SendByte(index & 0xFF);
    if (MyI2C_ReceiveAck() != 0) { 
        MyI2C_Stop(); 
        return -1; 
    }
    
    // 5. 循环发送 pdata 缓冲区里的所有数据
    for (uint32_t i = 0; i < count; i++) {
        MyI2C_SendByte(pdata[i]);
        if (MyI2C_ReceiveAck() != 0) { 
            MyI2C_Stop();
            return -1; 
        }
    }
    
    // 6. 所有数据发送完毕，发送 I2C 停止信号 (Stop)
    MyI2C_Stop();
    
    status=0;

	return status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	// ---- 阶段 1: 伪写寄存器地址 ----
    MyI2C_Start();
    MyI2C_SendByte(dev); // 默认发送写地址 0x52
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return -1; }
    
    MyI2C_SendByte((index >> 8) & 0xFF); // 发送寄存器高 8 位
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return -1; }
    
    MyI2C_SendByte(index & 0xFF);        // 发送寄存器低 8 位
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return -1; }
    
    // ---- 阶段 2: 重复起始，切换为读模式 ----
    MyI2C_Start();
    MyI2C_SendByte(dev | 0x01); // 切换为读地址 0x53
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return -1; }
    
    // ---- 阶段 3: 循环读取数据（精准适配你的接口） ----
    for (uint32_t i = 0; i < count; i++) {
        // 1. 先把字节读进来
        pdata[i] = MyI2C_ReceiveByte(); 
        
        // 2. 紧接着发送应答信号
        if (i == count - 1) {
            // 如果是最后一个字节，必须发 NACK (1) 告诉传感器：别发了，我要 Stop 了
            MyI2C_SendAck(1); 
        } else {
            // 如果不是最后一个字节，发 ACK (0) 告诉传感器：继续吐下一个数据
            MyI2C_SendAck(0); 
        }
    }
    
    // ---- 阶段 4: 结束通信 ----
    MyI2C_Stop();
    status = 0;
	return status;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return VL53L1_WriteMulti(dev,index,&data,1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	uint8_t buf[2];
	buf[0]=(uint8_t)(data>>8);
	buf[1]=(uint8_t)(data & 0xFF);
	return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	uint8_t buf[4];
	buf[0]=(uint8_t)(data>>24);
	buf[1]=(uint8_t)(data>>16);
	buf[2]=(uint8_t)(data>>8);
	buf[3]=(uint8_t)(data & 0xFF);
	return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return VL53L1_ReadMulti(dev,index,data,1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	uint8_t buf[2]; // 定义一个2字节的临时缓冲区
    
    // 连读2个字节存入 buf
    int8_t status = VL53L1_ReadMulti(dev, index, buf, 2);
    
    if (status == 0) {
        // 大端序转换：先收到的 buf[0] 是高 8 位，后收到的 buf[1] 是低 8 位
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    }
    
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
		uint8_t buf[4]; // 定义一个2字节的临时缓冲区
    
    // 连读2个字节存入 buf
    int8_t status = VL53L1_ReadMulti(dev, index, buf, 4);
    
	if (status == 0) {
        // 大端序转换：拼回 32 位无符号整数
        *data = ((uint32_t)buf[0] << 24) |
                ((uint32_t)buf[1] << 16) |
                ((uint32_t)buf[2] << 8)  |
                buf[3];
    }
    
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	if (wait_ms <= 0) {
        return 0;
    }
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	My_delay_ms((uint32_t)wait_ms);
	return 0;
}
