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
#include "cmsis_os.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

SimplePID::PIDParam param = {
    400.0f,  // Kp
    0.0f,   // Ki
    0.0f, // Kd
    1000.0f,  // outputLimit
    0.0f    // intergralLimit
};
SimplePID front_left_PID(SimplePID::PID_POSITION, param);
SimplePID front_right_PID(SimplePID::PID_POSITION, param);
SimplePID rear_left_PID(SimplePID::PID_POSITION, param);
SimplePID rear_right_PID(SimplePID::PID_POSITION, param);
// Motor
// MotorDM4310 motor(1, 0, 3.1415926f, 40, 15, &myPID);
MotorM3508 motor_front_left(1, &front_left_PID);
MotorM3508 motor_front_right(4, &front_right_PID);
MotorM3508 motor_rear_left(5, &rear_left_PID);
MotorM3508 motor_rear_right(6, &rear_right_PID);
Dr16RemoteControl dr16;

// RemoteControl
// 初始化遥控器

extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length);
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

//定义参数，调试时修正
#define MAX_FORWARD_SPEED 2.0f  //向前为正 m/s
#define MAX_RIGHT_SPEED  2.0f  //向右为正 m/s
#define MAX_ANGULAR_SPEED 10.0f  //逆时针为正 rad/s
typedef struct{ float vx;                     //整车目标前后速度 m/s   ；前正
                float vy;                     //整车目标左右速度 m/s   ；右正
                float vw;                     //整车目标角速度 rad/s   ；逆时针正
} MecanumWheelTargetSpeed;

//底盘参数，以实际值为准
#define L 0.2f  // 底盘横向轮距的一半 m
#define W 0.2f  // 底盘纵向轮距的一半 m
MecanumWheelTargetSpeed targetSpeed = {0.0f, 0.0f, 0.0f};

extern "C" void mecanum_wheel(void *argument)
{   
    
    CAN_Init(&hcan1, can1RxCallback);        // 初始化CAN1
    UART_Init(&huart3, dr16ITCallback, 18);   // 初始化USART1用于接收DR16遥控器数据
    
    printf("Mecanum Wheel Task Started\r\n");
    printf("CAN1 Initialized\r\n");
    
    while (1)
    {
        dr16.updateEvent();                 // 更新遥控器事件
        dr16.getLeftStickX();              // 获取左摇杆X轴数据
        dr16.getLeftStickY();              // 获取左摇杆Y轴数据
        dr16.getRightStickX();             // 获取右摇杆X轴数据
        dr16.getRightStickY();             // 获取右摇杆Y轴数据
        //将遥控器指令解算为整车目标转速

        targetSpeed.vx = dr16.getLeftStickY() * MAX_FORWARD_SPEED;  //前后速度 m/s
        targetSpeed.vy = dr16.getLeftStickX() * MAX_RIGHT_SPEED;    //左右速度 m/s
        targetSpeed.vw = dr16.getRightStickX() * MAX_ANGULAR_SPEED;  //角速度 rad/s

        //将整车目标转速解算为四轮目标转速
        float motor_front_left_speed  = 100*(targetSpeed.vx + targetSpeed.vy + (L + W) * targetSpeed.vw);
        float motor_front_right_speed = 100*(targetSpeed.vx - targetSpeed.vy - (L + W) * targetSpeed.vw);
        float motor_rear_left_speed   = 100*(targetSpeed.vx + targetSpeed.vy - (L + W) * targetSpeed.vw);
        float motor_rear_right_speed  = 100*(targetSpeed.vx - targetSpeed.vy + (L + W) * targetSpeed.vw);

        // 设置目标角速度并执行闭环控制
        motor_front_left.angularVelocityClosedloopControl(motor_front_left_speed);
        motor_front_right.angularVelocityClosedloopControl(motor_front_right_speed);
        motor_rear_left.angularVelocityClosedloopControl(motor_rear_left_speed);
        motor_rear_right.angularVelocityClosedloopControl(motor_rear_right_speed);

        transmitMotorsControlData();       // 发送电机控制数据
        
        // 添加任务延时，避免占用所有CPU时间
        osDelay(1);  // 延时1ms
    }
    
}

extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length)
{
    dr16.receiveRxDataFromISR(Buffer);
}

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    motor_front_left.decodeCanRxMessageFromISR(pRxMsg);
    motor_front_right.decodeCanRxMessageFromISR(pRxMsg);
    motor_rear_left.decodeCanRxMessageFromISR(pRxMsg);
    motor_rear_right.decodeCanRxMessageFromISR(pRxMsg);
}

/**
 * @brief 发送电机控制数据
 */
inline void transmitMotorsControlData()
{
    uint32_t send_mail_box;
    HAL_CAN_AddTxMessage(&hcan1, motor_front_left.getMotorControlHeader(), motor_front_left.getMotorControlData(), &send_mail_box);
    HAL_CAN_AddTxMessage(&hcan1, motor_front_right.getMotorControlHeader(), motor_front_right.getMotorControlData(), &send_mail_box);
    HAL_CAN_AddTxMessage(&hcan1, motor_rear_left.getMotorControlHeader(), motor_rear_left.getMotorControlData(), &send_mail_box);
    HAL_CAN_AddTxMessage(&hcan1, motor_rear_right.getMotorControlHeader(), motor_rear_right.getMotorControlData(), &send_mail_box);
}