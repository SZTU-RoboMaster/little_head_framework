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
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "chassis.h"
#include "dr16.h"
#include "gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t init_finished = false;
uint32_t flag = 0;

Chassis chassis;
Gimbal gimbal;

Dr16 dr16;

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
    case (0x202):
    {
        chassis.wheel_motor_[0].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x201):
    {
        chassis.wheel_motor_[1].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x204):
    {
        chassis.wheel_motor_[2].can_rx_callback(rx_buffer->data);

        break;
    }
    case (0x203):
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
    case (0x206):
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

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    gimbal.bmi088_.exti_read_callback(gpio_pin);
}

/**
 * @brief TIM7任务回调函数
 *
 */
void task1ms_tim7_callback()
{
    static uint8_t alive_mod100 = 0;
    if (alive_mod100++ >= 100)
    {
        alive_mod100 = 0;

        dr16.check_alive_100ms();

        gimbal.motor_yaw_.check_alive_100ms();
        gimbal.motor_pitch_.check_alive_100ms();

        for (int i = 0; i < 4; i++)
        {
            chassis.wheel_motor_[i].check_alive_100ms();
        }
    }

    gimbal.motor_yaw_.calculate();
    gimbal.motor_pitch_.calculate();
    for (int i = 0; i < 4; i++)
    {
        chassis.wheel_motor_[i].calculate();
    }
    can_data_send(can1_manage_obj.can_handle, 0x200, can1_0x200_tx_data, 8);
    can_data_send(can1_manage_obj.can_handle, 0x1fe, can1_0x1fe_tx_data, 8);
    can_data_send(can2_manage_obj.can_handle, 0x1fe, can2_0x1fe_tx_data, 8);

    flag++;
}

void task_init()
{
    dr16.init(&huart3);
    gimbal.dr16_ = &dr16;
    chassis.dr16_ = &dr16;

    gimbal.init();
    chassis.init();

    // HAL_Delay(300);

    can_init(&hcan1, device_can1_callback);
    can_init(&hcan2, device_can2_callback);
    uart_init(&huart3, dr16_uart3_callback, 18);
    tim_init(&htim7, task1ms_tim7_callback);

    init_finished = true;
}

void task_loop()
{
    if (gimbal.bmi088_.update_flag_)
    {
        gimbal.bmi088_.update_flag_ = 0;
        gimbal.bmi088_.gravity_kf_.update(
            gimbal.bmi088_.rx_data_.gyro[0], gimbal.bmi088_.rx_data_.gyro[1],
            gimbal.bmi088_.rx_data_.gyro[2], gimbal.bmi088_.rx_data_.accel[0],
            gimbal.bmi088_.rx_data_.accel[1], gimbal.bmi088_.rx_data_.accel[2], 0.001f);
        gimbal.bmi088_.quaternion_ekf_.update(
            gimbal.bmi088_.rx_data_.gyro[0], gimbal.bmi088_.rx_data_.gyro[1],
            gimbal.bmi088_.rx_data_.gyro[2], gimbal.bmi088_.gravity_kf_.gravity_vec_[0],
            gimbal.bmi088_.gravity_kf_.gravity_vec_[1], gimbal.bmi088_.gravity_kf_.gravity_vec_[2],
            0.001f);
        gimbal.ins_angle_[0] = gimbal.bmi088_.quaternion_ekf_.ins_.angle[0];
        gimbal.ins_angle_[1] = -gimbal.bmi088_.quaternion_ekf_.ins_.angle[1];
        gimbal.ins_angle_[2] = gimbal.bmi088_.quaternion_ekf_.ins_.angle[2];
    }

    static uint32_t pre_flag = 0;
    if (flag != pre_flag)
    {
        pre_flag = flag;

        gimbal.update_input();
        gimbal.update_feedback();
        gimbal.handle_safety();
        gimbal.set_mode();
        gimbal.update_control_state();
        gimbal.control();
        gimbal.output();

        chassis.set_gimbal_yaw(gimbal.get_yaw());
        chassis.update_input();
        chassis.update_feedback();
        chassis.handle_safety();
        chassis.set_mode();
        chassis.control();
        chassis.solve();
        chassis.output();
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
