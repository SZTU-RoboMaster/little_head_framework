/**
 * @file robot_sdk.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "robot_sdk.h"
#include "dr16.h"
#include "vt13.h"
#include "referee.h"
#include "INS_task.h"

#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_imu_pwm.h"
#include "bsp_led.h"
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "bsp_usb.h"

#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "vision_task.h"
#include "message_center.h"
#include "message_def.h"
#include "ins_event.h"
#include "main.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t initialized = 0;
uint64_t flag = 0;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 注册各模块共用的消息主题，消息中心由 robot_sdk 统一初始化
 */
void MessageCenter_Topics_Init(void)
{
    if (!Register_Topic(kInsTopicName, sizeof(Ins_Message_t)) ||
        !Register_Topic(kVisionTopicName, sizeof(Vision_Message_t)) ||
        !Register_Topic(kGimbalTopicName, sizeof(Gimbal_Message_t)) ||
        !Register_Topic(kDr16TopicName, sizeof(Dr16_Message_t)) ||
        !Register_Topic(kVt13TopicName, sizeof(Vt13_Message_t)) ||
        !Register_Topic(kCmdGimbalTopicName, sizeof(Gimbal_Cmd_Message_t)) ||
        !Register_Topic(kCmdChassisTopicName, sizeof(Chassis_Cmd_Message_t)) ||
        !Register_Topic(kCmdShootTopicName, sizeof(Shoot_Cmd_Message_t)) ||
        !Register_Topic(kChassisTopicName, sizeof(Chassis_Message_t)) ||
        !Register_Topic(kPowerControllerTopicName, sizeof(Power_Controller_Message_t)) ||
        !Register_Topic(kRefereeTopicName, sizeof(Referee_Message_t)))
    {
        Error_Handler();
    }
}

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
static void device_can1_callback(BspCan_RxBuffer_t *rx_buffer)
{
    switch (rx_buffer->header.Identifier)
    {
    case (0x201):
    {
        MotorDji_CanRxCallback(&chassis.wheel_motor[0], rx_buffer->data);

        break;
    }
    case (0x202):
    {
        MotorDji_CanRxCallback(&chassis.wheel_motor[1], rx_buffer->data);

        break;
    }
    case (0x203):
    {
        MotorDji_CanRxCallback(&chassis.wheel_motor[2], rx_buffer->data);

        break;
    }
    case (0x204):
    {
        MotorDji_CanRxCallback(&chassis.wheel_motor[3], rx_buffer->data);

        break;
    }
    case (0x205):
    {
        MotorDji_CanRxCallback(&gimbal.motor_yaw, rx_buffer->data);

        break;
    }
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
static void device_can2_callback(BspCan_RxBuffer_t *rx_buffer)
{
    switch (rx_buffer->header.Identifier)
    {
    case (0x201):
    {
        MotorDji_CanRxCallback(&shoot.trigger, rx_buffer->data);

        break;
    }
    case (0x202):
    {
        MotorDji_CanRxCallback(&shoot.friction_left, rx_buffer->data);

        break;
    }
    case (0x203):
    {
        MotorDji_CanRxCallback(&shoot.friction_right, rx_buffer->data);

        break;
    }
    case (0x205):
    {
        MotorDji_CanRxCallback(&gimbal.motor_pitch, rx_buffer->data);

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
static void vt13_uart1_callback(uint8_t *buffer, uint16_t length)
{
    Vt13_UartRxCallback(&vt13, buffer, length);
}

/**
 * @brief UART3遥控器回调函数
 *
 * @param buffer UART3收到的消息
 * @param length 长度
 */
static void dr16_uart3_callback(uint8_t *buffer, uint16_t length)
{
    Dr16_UartRxCallback(&dr16, buffer, length);
}

/**
 * @brief UART5裁判系统回调函数
 *
 * @param buffer UART5收到的消息
 * @param length 长度
 */
static void referee_uart5_callback(uint8_t *buffer, uint16_t length)
{
    Referee_UartRxCallback(&referee, buffer, length);
}

/**
 * @brief 外部中断回调函数
 *
 * @param gpio_pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (!initialized)
    {
        return;
    }

    if (gpio_pin == IMU_INT1_Pin)
    {
        if (Bmi270_ExtiReadCallback(&ins.bmi270, gpio_pin) && insTaskHandle != NULL)
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
static void vision_usb_callback(uint8_t *buf, uint32_t len)
{
    Vision_UsbRxCallback(buf, len);
}

/**
 * @brief TIM7任务回调函数
 *
 */
static void task1ms_tim7_callback(void)
{
    static uint16_t alive_mod1000 = 0;
    if (++alive_mod1000 >= 1000)
    {
        alive_mod1000 = 0;
        Referee_CheckAlive1000ms(&referee);
    }

    static uint8_t alive_mod100 = 0;
    if (++alive_mod100 >= 100)
    {
        alive_mod100 = 0;

        Dr16_CheckAlive100ms(&dr16);
        Vt13_CheckAlive100ms(&vt13);

        MotorDji_CheckAlive100ms(&gimbal.motor_yaw);
        MotorDji_CheckAlive100ms(&gimbal.motor_pitch);

        for (int i = 0; i < 4; i++)
        {
            MotorDji_CheckAlive100ms(&chassis.wheel_motor[i]);
        }

        MotorDji_CheckAlive100ms(&shoot.trigger);
        MotorDji_CheckAlive100ms(&shoot.friction_left);
        MotorDji_CheckAlive100ms(&shoot.friction_right);
    }

    MotorDji_Calculate(&gimbal.motor_yaw);
    MotorDji_Calculate(&gimbal.motor_pitch);
    for (int i = 0; i < 4; i++)
    {
        MotorDji_Calculate(&chassis.wheel_motor[i]);
    }
    MotorDji_Calculate(&shoot.trigger);
    MotorDji_Calculate(&shoot.friction_left);
    MotorDji_Calculate(&shoot.friction_right);
    BspCan_Send(bsp_can1.can_handle, 0x200, can1_0x200_tx_data, 8);
    BspCan_Send(bsp_can1.can_handle, 0x1fe, can1_0x1fe_tx_data, 8);
    BspCan_Send(bsp_can2.can_handle, 0x1fe, can2_0x1fe_tx_data, 8);
    BspCan_Send(bsp_can2.can_handle, 0x200, can2_0x200_tx_data, 8);

    flag++;
}

void RobotSdk_Init(void)
{
    MessageCenter_Init();
    MessageCenter_Topics_Init();

    BspDwt_Init();
    BspImuPwm_Init();
    BspLed_Init();
    BspBuzzer_Init();

    Dr16_Init(&dr16, &huart3);
    Vt13_Init(&vt13, &huart5);
    Referee_Init(&referee, &huart1);

    BspLed_ShowArgb(0xFFFF0000);
}

void RobotSdk_Start(void)
{
    initialized = 1;

    BspUsb_Init(vision_usb_callback);
    BspUart_Init(&huart1, vt13_uart1_callback, 21);
    BspUart_Init(&huart3, dr16_uart3_callback, 18);
    BspUart_Init(&huart5, referee_uart5_callback, 255);
    BspCan_Init(&hfdcan1, device_can1_callback);
    BspCan_Init(&hfdcan2, device_can2_callback);
    BspTim_Init(&htim7, task1ms_tim7_callback);

    BspLed_ShowArgb(0xFFFFFFFF);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
