/**
 * @file template.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "initial_task.h"

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "dr16.h"
#include "referee.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "vision_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t init_finished = 0;
uint32_t flag = 0;

Dr16 dr16;
Referee referee;


/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void device_can1_callback(CanRxBuffer *rx_buffer)
{
    switch (rx_buffer->header.StdId)
    {
    case (0x201):
    {
        chassis.wheel_motor_[0].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x202):
    {
        chassis.wheel_motor_[1].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x203):
    {
        chassis.wheel_motor_[2].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x204):
    {
        chassis.wheel_motor_[3].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x205):
    {
        gimbal.motor_yaw_.can_rx_callback(rx_buffer->data);

        break;
    }
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void device_can2_callback(CanRxBuffer *rx_buffer)
{
    switch (rx_buffer->header.StdId)
    {
    case (0x201):
    {
        shoot.trigger_.can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x202):
    {
        shoot.friction_left_.can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x203):
    {
        shoot.friction_right_.can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x205):
    {
        gimbal.motor_pitch_.can_rx_callback(rx_buffer->data);

        break;
    }
    }
}

/**
 * @brief UART3遥控器回调函数
 *
 * @param buffer UART3收到的消息
 * @param length 长度
 */
void dr16_uart3_callback(uint8_t *buffer, uint16_t length)
{
    dr16.uart_rx_callback(buffer, length);
}

/**
 * @brief UART6裁判系统回调函数
 *
 * @param buffer UART6收到的消息
 * @param length 长度
 */
void referee_uart6_callback(uint8_t *buffer, uint16_t length)
{
    referee.uart_rx_callback(buffer, length);
}

/**
 * @brief 外部中断回调函数
 *
 * @param gpio_pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == INT1_ACCEL_Pin || gpio_pin == INT1_GYRO_Pin)
    {
        ins.bmi088_.exti_read_callback(gpio_pin);

        if (gpio_pin == INT1_GYRO_Pin && insTaskHandle != NULL)
        {
            osThreadFlagsSet(insTaskHandle, INS_DATA_READY_FLAG);
        }
    }
}

/**
 * @brief USB视觉回调函数
 *
 * @param buf 接收的数据
 * @param len 数据长度
 */
void vision_usb_callback(uint8_t *buf, uint32_t len)
{
    vision.usb_rx_callback(buf, len);
}

/**
 * @brief TIM7任务回调函数
 *
 */
void task1ms_tim7_callback()
{
    static uint8_t alive_mod1000 = 0;
    if (alive_mod1000++ >= 1000)
    {
        alive_mod1000 = 0;
        referee.check_alive_1000ms();
    }

    static uint8_t alive_mod100 = 0;
    if (alive_mod100++ >= 100)
    {
        alive_mod100 = 0;

        dr16.check_alive_100ms();
        vision.check_alive_100ms();

        gimbal.motor_yaw_.check_alive_100ms();
        gimbal.motor_pitch_.check_alive_100ms();

        for (int i = 0; i < 4; i++)
        {
            chassis.wheel_motor_[i].check_alive_100ms();
        }

        shoot.trigger_.check_alive_100ms();
        shoot.friction_left_.check_alive_100ms();
        shoot.friction_right_.check_alive_100ms();
    }

    gimbal.motor_yaw_.calculate();
    gimbal.motor_pitch_.calculate();
    for (int i = 0; i < 4; i++)
    {
        chassis.wheel_motor_[i].calculate();
    }
    shoot.trigger_.calculate();
    shoot.friction_left_.calculate();
    shoot.friction_right_.calculate();
    can_data_send(can1_manage_obj.can_handle, 0x200, can1_0x200_tx_data, 8);
    can_data_send(can1_manage_obj.can_handle, 0x1fe, can1_0x1fe_tx_data, 8);
    can_data_send(can2_manage_obj.can_handle, 0x1fe, can2_0x1fe_tx_data, 8);
    can_data_send(can2_manage_obj.can_handle, 0x200, can2_0x200_tx_data, 8);

    flag++;
}

void task_init()
{
    dwt_init();

    dr16.init(&huart3);
    gimbal.dr16_ = &dr16;
    chassis.dr16_ = &dr16;
    shoot.dr16_ = &dr16;
    shoot.referee_ = &referee;
    referee.init(&huart6);

    can_init(&hcan1, device_can1_callback);
    can_init(&hcan2, device_can2_callback);
    uart_init(&huart3, dr16_uart3_callback, 18);
    uart_init(&huart6, referee_uart6_callback, 255);
    tim_init(&htim7, task1ms_tim7_callback);
    usb_init(vision_usb_callback);

    init_finished = 1;
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
