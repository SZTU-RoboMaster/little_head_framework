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
#include "message_center.h"
#include "stm32f4xx_hal.h"
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

MessageCenter &MessageCenter::instance()
{
    static MessageCenter instance;
    return instance;
}

/**
 * @brief 注册Topic
 *
 * @param topic_name Topic名称
 * @param payload_size Topic数据大小
 * @param is_publisher 是否为发布者

 */
Topic *MessageCenter::register_topic(const char *topic_name, std::size_t payload_size,
                                     bool is_publisher)
{
    // 检查参数有效性
    if (topic_name == nullptr)
    {
        return nullptr;
    }

    const std::size_t name_length = std::strlen(topic_name);
    if (name_length > kMaxTopicNameLength || payload_size > kMaxPayloadSize)
    {
        return nullptr;
    }

    // 查找已注册的Topic或空闲的Topic槽
    Topic *empty_topic = nullptr;
    for (Topic &topic : topics_)
    {
        if (!topic.is_active_)
        {
            if (empty_topic == nullptr)
            {
                empty_topic = &topic;
            }
            continue;
        }
        if (std::strcmp(topic.name_, topic_name) == 0)
        {
            if (topic.payload_size_ != payload_size)
            {
                return nullptr;
            }
            if (is_publisher && topic.has_publisher_)
            {
                return nullptr;
            }
            topic.has_publisher_ |= is_publisher;
            return &topic;
        }
    }

    // 如果没有找到已注册的Topic，则使用空闲的Topic槽进行注册
    if (empty_topic != nullptr)
    {
        std::memcpy(empty_topic->name_, topic_name, name_length + 1);
        empty_topic->payload_size_ = payload_size;
        empty_topic->is_active_ = true;
        empty_topic->has_publisher_ = is_publisher;
        empty_topic->has_value_ = false;
        return empty_topic;
    }

    return nullptr;
}

bool Topic::publish_msg(const void *data)
{
    if (!is_active_ || !has_publisher_ || data == nullptr)
    {
        return false;
    }

    std::memcpy(data_, data, payload_size_);
    sequence_++;
    timestamp_ms_ = HAL_GetTick();
    has_value_ = true;
    return true;
}

bool Topic::update_msg(void *data, uint64_t &last_sequence, uint32_t &last_timestamp_ms,
                       bool &first_in)
{
    if (!is_active_ || !has_value_ || data == nullptr || last_sequence == sequence_)
    {
        return false;
    }

    std::memcpy(data, data_, payload_size_);
    last_sequence = sequence_;
    last_timestamp_ms = timestamp_ms_;
    first_in = false;
    return true;
}

bool Topic::is_fresh(uint32_t last_timestamp_ms, bool first_in, uint32_t timeout_ms) const
{
    if (first_in)
    {
        return false;
    }

    uint32_t elapsed_time = HAL_GetTick() - last_timestamp_ms;

    return elapsed_time <= timeout_ms;
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
