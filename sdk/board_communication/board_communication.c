//
// Created by hjr on 2026/11/11.
//
#include "board_communication.h"

#include "bsp_can.h"
#include "message_center.h"
#include "message_def.h"
#include "robot_sdk.h"

#include <string.h>

#ifdef GIMBAL_BOARD
// 云台板：读取本地控制指令
Subscriber_t boardcomm_chassiscmd_subscriber;

// 云台板：发布从底盘板收到的数据
Publisher_t boardcomm_referee_publisher;

#endif


#ifdef CHASSIS_BOARD
// 底盘板：发布从云台板收到的指令
Publisher_t boardcomm_chassiscmd_publisher;

// 底盘板：读取本地反馈
Subscriber_t boardcomm_referee_subscriber;

#endif


/**
 * @brief 使用FDCAN帧进行板间通信
 * @note  增加数据时只需新增memcpy和used += sizeof()即可
 * @attention 注意不要超过64字节
 */

//云台给底盘
static void Boardcomm_Send_Chassis_cmd(void)
{
    // 1. 从消息中心读取数据

    // 定义一个局部结构体
    Chassis_Cmd_Message_t cmd;
    // 将数据复制进局部结构体
    if (!Message_Read(&boardcomm_chassiscmd_subscriber, &cmd, sizeof(cmd)))
    {
        return;

    }
    // 2. 数据打包
    uint8_t data[64] = {0};
    size_t used = 0;

    data[used++] = (uint8_t)cmd.chassis_mode;

    memcpy(&data[used], &cmd.rc_vx, sizeof(cmd.rc_vx));
    used += sizeof(cmd.rc_vx);

    memcpy(&data[used], &cmd.rc_vy, sizeof(cmd.rc_vy));
    used += sizeof(cmd.rc_vy);

    memcpy(&data[used], &cmd.rc_vw, sizeof(cmd.rc_vw));
    used += sizeof(cmd.rc_vw);

    // 3. 使用 CAN ID 0x301 发送
    BspFDCan_Send(&hfdcan3, 0x301, data, used);

}

static void Boardcomm_Send_Yaw_Angle(void)
{
    // 1. 从消息中心数据
    // 2. 数据打包
    // 3. 使用 CAN ID 0x302 发送
}

//底盘给云台
static void Boardcomm_Send_Referee(void)
{
    // 1. 从消息中心读取数据
    // 2. 数据打包
    // 3. 使用 CAN ID 0x311 发送
}

void Boardcomm_Gimbal_Send(void)
{
    Boardcomm_Send_Chassis_cmd();
    //Boardcomm_Send_Yaw_Angle();
}

void Boardcomm_Chassis_Send(void)
{
    Boardcomm_Send_Referee();
}


//底盘接收
void Boardcomm_Chssais_Receive(BspCan_RxBuffer_t *rx_buffer)
{
    switch (rx_buffer->header.Identifier)
    {
    case(0x301):
    {
        // 1. 解包
        Chassis_Cmd_Message_t cmd = {0};
        cmd.chassis_mode = (ChassisCmdMode_e)rx_buffer->data[0];
        memcpy(&cmd.rc_vx, &rx_buffer->data[1], sizeof(cmd.rc_vx));
        memcpy(&cmd.rc_vy, &rx_buffer->data[5], sizeof(cmd.rc_vy));
        memcpy(&cmd.rc_vw, &rx_buffer->data[9], sizeof(cmd.rc_vw));
        // 2. 发布到消息中心
        Message_PublishFromISR(&boardcomm_chassiscmd_publisher, &cmd, sizeof(cmd));
    }
    }
}

//云台接收
void Boardcomm_Gimbal_Receive(BspCan_RxBuffer_t *rx_buffer)
{
    switch (rx_buffer->header.Identifier)
    {
    case(0x311):
    {
        // 1. 解包
        // 2. 发布到消息中心
    }
    }
}