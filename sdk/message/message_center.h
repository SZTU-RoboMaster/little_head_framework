//
// Created by hjr on 2026/11/11.
//
#ifndef G4_LITTLE_HEAD_MESSAGE_CENTER_H
#define G4_LITTLE_HEAD_MESSAGE_CENTER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define MAX_TOPICS       24
#define MAX_NAME_LEN     31
#define MAX_DATA_SIZE    256

typedef struct
{
    char name[MAX_NAME_LEN + 1];

    uint8_t data[MAX_DATA_SIZE];
    size_t data_size;

    bool is_active;
    bool has_publisher;
    bool has_value;

    uint32_t update_count;      // 发布次数，也用作消息序号
    uint32_t last_update_time;  // 最近发布时间，单位为 FreeRTOS tick（本工程为 1 ms）

    uint16_t subscriber_count;
} Topic_t;

typedef struct
{
    Topic_t topics[MAX_TOPICS];
    uint16_t topic_count;
    uint32_t register_error_count; // 注册失败次数
} MessageCenter_t;


typedef struct
{
    Topic_t *topic;
    uint32_t publish_count;
} Publisher_t;

typedef struct
{
    Topic_t *topic;
    uint32_t read_count;
    uint32_t last_read_time;         // 最近一次成功读取时的 FreeRTOS tick
    uint32_t last_read_update_count; // 最近读到的 Topic 版本号（重复读取时也会记录）
    uint32_t last_message_time;      // 最近读到的消息的发布 tick
    bool has_read;                  // 是否至少成功读过一次，仅供调试观察
} Subscriber_t;

#ifdef __cplusplus
extern "C" {
#endif

extern MessageCenter_t g_message_center;

//消息中心初始化
void MessageCenter_Init(void);

//话题初始化
//由 Robot SDK 根据本机配置注册话题，在 MessageCenter_Init() 之后调用。
void MessageCenter_Topics_Init(void);

//注册话题
bool Register_Topic(const char *name, size_t data_size);

//创建发布者
bool Create_Publisher(Publisher_t *publisher, const char *topic_name, size_t data_size);

//创建订阅者
bool Create_Subscriber(Subscriber_t *subscriber, const char *topic_name, size_t data_size);

//以下三个接口在 FreeRTOS 任务中调用，ISR 中不可调用。
//发布最新消息（data_size 必须与注册 Topic 时的大小一致）
bool Message_Publish(Publisher_t *publisher, const void *data, size_t data_size);

//读取 Topic 中的最新消息（允许重复读取；无数据或长度不符返回 false）
bool Message_Read(Subscriber_t *subscriber, void *data, size_t data_size);

//判断 Topic 是否长期未发布消息（与 Subscriber 是否读取无关）
bool Message_IsTimeout(const Subscriber_t *subscriber, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // G4_LITTLE_HEAD_MESSAGE_CENTER_H
