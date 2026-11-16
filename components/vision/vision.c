/**
 * @file vision.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "vision.h"
#include "CRC8_CRC16.h"
#include "bsp_usb.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(FrameHeader) == 5, "Vision header layout changed");
_Static_assert(sizeof(VisionData) == 58, "Vision transmit layout changed");
_Static_assert(sizeof(RobotCtrlData) == 37, "Vision receive layout changed");

/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;
Vision_t vision;

/* Private function declarations ---------------------------------------------*/
static void Vision_Update_Rx(const uint8_t *buf, uint32_t len);
static void Vision_Update_Tx(void);

void Vision_Init(void)
{
    // C结构体的非零协议默认值在初始化函数中设置
    vision.tx_data.header.sof = HEADER_SOF;
    vision.tx_data.quaternion[0] = 1.0f;
    vision.tx_data.tail.end1 = END1_SOF;
    vision.tx_data.tail.end2 = END2_SOF;
    vision.rx_data.header.sof = HEADER_SOF;
    vision.rx_data.target_lock = 50; // 49: lock, 50: unlock
    vision.rx_data.tail.end1 = END1_SOF;
    vision.rx_data.tail.end2 = END2_SOF;

    // Topic 已由 robot_sdk 注册
    Create_Publisher(&vision.vision_publisher, kVisionTopicName, sizeof(Vision_Message_t));
    Create_Subscriber(&vision.ins_subscriber, kInsTopicName, sizeof(vision.ins_msg));
}

/**
 * @brief USB通信接收回调函数
 * @param buf 接收的数据
 * @param len 数据长度
 * @note 从USB中断调用，仅解析并保存有效帧；消息发布在视觉任务中完成
 */
void Vision_UsbRxCallback(uint8_t *buf, uint32_t len)
{
    Vision_Update_Rx(buf, len);
}

/**
 * @brief 在任务上下文中发布最新有效的视觉目标
 */
void Vision_Publish_New(void)
{
    RobotCtrlData snapshot;
    uint8_t has_new_frame = 0;

    // USB中断优先级为5；任务临界区保证帧和序号来自同一次接收
    taskENTER_CRITICAL();
    if (vision.rx_flag != vision.last_rx_flag)
    {
        snapshot = vision.rx_data;
        vision.last_rx_flag = vision.rx_flag;
        has_new_frame = 1;
    }
    taskEXIT_CRITICAL();

    if (has_new_frame)
    {
        Vision_Message_t msg = {
            .yaw = snapshot.yaw,
            .yaw_vel = snapshot.yaw_vel,
            .yaw_acc = snapshot.yaw_acc,
            .pitch = snapshot.pitch,
            .pitch_vel = snapshot.pitch_vel,
            .pitch_acc = snapshot.pitch_acc,
            .target_lock = snapshot.target_lock,
            .fire_command = snapshot.fire_command,
        };
        Message_Publish(&vision.vision_publisher, &msg, sizeof(msg));
    }
}

void Vision_Send(void)
{
    // 不修改USB仍在发送的缓冲区；就绪前也不发送
    if (!initialized || hUsbDeviceFS.pClassData == NULL)
        return;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0)
        return;

    Vision_Update_Tx();
    CDC_Transmit_FS((uint8_t *)&vision.tx_data, sizeof(vision.tx_data));
}

static void Vision_Update_Tx(void)
{
    Message_Read(&vision.ins_subscriber, &vision.ins_msg, sizeof(vision.ins_msg));

    vision.tx_data.header.sof = HEADER_SOF;
    vision.tx_data.header.data_length = sizeof(VisionData) - sizeof(FrameHeader) -
                                        sizeof(vision.tx_data.cmd_id) -
                                        sizeof(vision.tx_data.crc16) - sizeof(FrameTail);
    vision.tx_data.header.seq = vision.seq++;
    append_CRC8_check_sum((uint8_t *)&vision.tx_data, sizeof(FrameHeader));
    vision.tx_data.cmd_id = VISION_ID;
    vision.tx_data.id = 7;
    vision.tx_data.mode = 33; // auto aim
    vision.tx_data.yaw = vision.ins_msg.angle[2];
    vision.tx_data.yaw_vel = vision.ins_msg.gyro[2];
    vision.tx_data.pitch = vision.ins_msg.angle[1];
    vision.tx_data.pitch_vel = vision.ins_msg.gyro[1];
    vision.tx_data.roll = vision.ins_msg.angle[0];
    memcpy(vision.tx_data.quaternion, vision.ins_msg.quaternion,
           sizeof(vision.ins_msg.quaternion));
    vision.tx_data.shoot_speed = 23.0f;
    vision.tx_data.bullet_count = 100;
    vision.tx_data.game_progress = 1;
    append_CRC16_check_sum((uint8_t *)&vision.tx_data,
                           sizeof(VisionData) - sizeof(FrameTail));
    vision.tx_data.tail.end1 = END1_SOF;
    vision.tx_data.tail.end2 = END2_SOF;
}

/**
 * @brief 接收流式数据并提取完整帧
 */
static void Vision_Update_Rx(const uint8_t *buf, uint32_t len)
{
    if (len == 0 || buf == NULL)
        return;

    // 单次数据超过接收缓冲区时只保留末尾，避免复制越界
    if (len > sizeof(vision.rx_buffer))
    {
        buf += len - sizeof(vision.rx_buffer);
        len = sizeof(vision.rx_buffer);
        vision.rx_len = 0;
    }
    // Append new data to the buffer, discard buffered bytes if overflow
    if (vision.rx_len + len > sizeof(vision.rx_buffer))
        vision.rx_len = 0;
    memcpy(vision.rx_buffer + vision.rx_len, buf, len);
    vision.rx_len += (uint16_t)len;

    while (vision.rx_len >= 1)
    {
        if (vision.rx_buffer[0] == HEADER_SOF)
        {
            if (vision.rx_len < sizeof(RobotCtrlData))
                break; // Wait for more data
            if (verify_CRC16_check_sum(vision.rx_buffer,
                                       sizeof(RobotCtrlData) - sizeof(FrameTail)))
            {
                memcpy(&vision.rx_data, vision.rx_buffer, sizeof(RobotCtrlData));
                vision.rx_flag++;
                // Shift remaining data forward
                vision.rx_len -= sizeof(RobotCtrlData);
                if (vision.rx_len > 0)
                    memmove(vision.rx_buffer, vision.rx_buffer + sizeof(RobotCtrlData),
                            vision.rx_len);
            }
            else
            {
                // CRC failed, invalid frame, shift 1 byte forward to re-sync
                vision.rx_len--;
                if (vision.rx_len > 0)
                    memmove(vision.rx_buffer, vision.rx_buffer + 1, vision.rx_len);
            }
        }
        else
        {
            // Invalid head, shift 1 byte forward to find next HEADER_SOF
            vision.rx_len--;
            if (vision.rx_len > 0)
                memmove(vision.rx_buffer, vision.rx_buffer + 1, vision.rx_len);
        }
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
