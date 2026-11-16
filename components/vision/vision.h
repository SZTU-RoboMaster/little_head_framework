/**
 * @file vision.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "message_center.h"
#include "message_def.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    // 接收缓冲区
    uint8_t rx_buffer[512];
    uint16_t rx_len;
    // 当前时刻的接收flag，由USB回调更新
    volatile uint32_t rx_flag;
    // 前一时刻的接收flag，由任务更新
    uint32_t last_rx_flag;
    // 发送序列号
    uint8_t seq;

    VisionData tx_data;
    RobotCtrlData rx_data;
    Subscriber_t ins_subscriber;
    Ins_Message_t ins_msg;
    Publisher_t vision_publisher;
} Vision_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Vision_t vision;

/* Exported function declarations ---------------------------------------------*/
void Vision_Init(void);
void Vision_UsbRxCallback(uint8_t *buf, uint32_t len);
void Vision_Publish_New(void);
void Vision_Send(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
