/*
 * @Author: Zixuan Wang tanng@163.com
 * @Date: 2026-05-22 00:23:36
 * @LastEditors: Zixuan Wang tanng@163.com
 * @LastEditTime: 2026-05-22 00:44:31
 * @FilePath: \mdkd:\Desktop\电控\raycom_pure\driver\inc\Driver_MyChassis.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __MY_CHASSIS_H
#define __MY_CHASSIS_H
#include "stm32f4xx.h"

void SendChassis_ByRPM(CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);
#endif
