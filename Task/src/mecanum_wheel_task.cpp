/**
 ******************************************************************************
    * @file    mecanum_wheel_task.cpp
    * @brief   麦轮解算
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "mecanum_wheel_task.hpp"
#include "drv_misc.h"
#include "projdefs.h"
#include "dvc_motor.hpp"
#include "dvc_remotecontrol.hpp"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

SimplePID::PIDParam param = {
    10.0f,  // Kp
    0.0f,   // Ki
    500.0f, // Kd
    10.0f,  // outputLimit
    0.0f    // intergralLimit
};
SimplePID myPID(SimplePID::PID_POSITION, param);
// Motor
// MotorDM4310 motor(1, 0, 3.1415926f, 40, 15, &myPID);
MotorM3508 motor1(6, &myPID);
MotorM3508 motor2(6, &myPID);
// RemoteControl

// 初始化遥控器
Dr16RemoteControl dr16;

extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length);

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

extern "C" void mecanum_wheel(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);        // 初始化CAN1
    while (1)
    {
        dr16.updateEvent();                 // 更新遥控器事件
        dr16.getLeftStickX();              // 获取左摇杆X轴数据
        dr16.getLeftStickY();              // 获取左摇杆Y轴数据
        dr16.getRightStickX();             // 获取右摇杆X轴数据
        dr16.getRightStickY();             // 获取右摇杆Y轴数据
        motor1.getCurrentAngularVelocity();
        motor2.getCurrentAngularVelocity();
        motor1.setTargetAngularVelocity(0.0f);
    }
    
}

extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length)
{
    dr16.receiveRxDataFromISR(Buffer);
}

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    motor1.decodeCanRxMessageFromISR(pRxMsg);
}

/**
 * @brief 发送电机控制数据
 */
inline void transmitMotorsControlData()
{
    const uint8_t *data = motor1.getMotorControlData();
    uint32_t send_mail_box;
    HAL_CAN_AddTxMessage(&hcan1, motor1.getMotorControlHeader(), data, &send_mail_box);
}