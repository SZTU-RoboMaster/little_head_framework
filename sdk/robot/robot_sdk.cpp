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
#include "robot_sdk.h"

#include "usb_device.h"

#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_imu_pwm.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "bsp_usb.h"

#include "dr16.h"
#include "referee.h"
#include "vt13.h"

#include "INS_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "vision_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t initialized = 0;
uint64_t flag = 0;

Dr16 dr16;
Vt13 vt13;
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
 * @brief UART1遥控器回调函数
 *
 * @param buffer UART1收到的消息
 * @param length 长度
 */
void vt13_uart1_callback(uint8_t *buffer, uint16_t length)
{
    vt13.uart_rx_callback(buffer, length);
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
    static uint16_t alive_mod1000 = 0;
    if (++alive_mod1000 >= 1000)
    {
        alive_mod1000 = 0;
        referee.check_alive_1000ms();
    }

    static uint8_t alive_mod100 = 0;
    if (++alive_mod100 >= 100)
    {
        alive_mod100 = 0;

        dr16.check_alive_100ms();
        vt13.check_alive_100ms();

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

void robot_sdk_init()
{
    dwt_init();
    imu_pwm_init();
    led_init();
    buzzer_init();

    dr16.init(&huart3);
    vt13.init(&huart1);
    referee.init(&huart6);

    aRGB_led_show(0xFFFF0000);
}

void robot_sdk_start()
{
    initialized = 1;

    usb_init(vision_usb_callback);
    uart_init(&huart1, vt13_uart1_callback, 21);
    uart_init(&huart3, dr16_uart3_callback, 18);
    uart_init(&huart6, referee_uart6_callback, 255);
    can_init(&hcan1, device_can1_callback);
    can_init(&hcan2, device_can2_callback);
    tim_init(&htim7, task1ms_tim7_callback);

    aRGB_led_show(0xFFFFFFFF);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
