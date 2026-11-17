//
// Created by hjr on 2026/11/16.
//

#include "message_center.h"

//
// Created by hjr on 2026/11/11.
//
#include "message_center.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static Topic_t *Find_Topic(const char *name);

MessageCenter_t g_message_center;

void MessageCenter_Init(void)
{
    memset(&g_message_center, 0, sizeof(g_message_center));
}



bool Register_Topic(const char *name,size_t data_size)
{
    // 1. 检查参数
    if (name == NULL || name[0] == '\0' || strlen(name) > MAX_NAME_LEN || data_size == 0 || data_size > MAX_DATA_SIZE)
    {
        g_message_center.register_error_count++;
        return false;
    }

    // 2. 检查是否存在同名 Topic
    for (int i = 0; i < MAX_TOPICS; i++)
    {
        Topic_t *topic = &g_message_center.topics[i];

        if (topic->is_active && strcmp(topic->name, name) == 0)
        {
            // 不允许重复注册
            g_message_center.register_error_count++;
            return false;
        }
    }

    // 3. 寻找空闲 Topic
    for (int i = 0; i < MAX_TOPICS; i++)
    {
        Topic_t *topic = &g_message_center.topics[i];

        if (!topic->is_active)
        {
            // 保存话题名称
            strcpy(topic->name, name);

            // 保存消息大小
            topic->data_size = data_size;

            // 标记 Topic 已注册
            topic->is_active = true;

            // Topic 总数增加
            g_message_center.topic_count++;

            return true;
        }
    }

    // 4. Topic 数组已满
    g_message_center.register_error_count++;
    return false;
}

/**
 * @brief 根据名称查找已注册的 Topic
 * @return 成功返回 Topic 指针，失败返回 NULL
 */
static Topic_t *Find_Topic(const char *name)
{
    if (name == NULL)
    {
        return NULL;
    }

    // 遍历消息中心的所有 Topic
    for (int i = 0; i < MAX_TOPICS; i++)
    {
        Topic_t *topic = &g_message_center.topics[i];

        // 只检查已经注册的 Topic
        if (!topic->is_active)
        {
            continue;
        }

        // 比较话题名称
        if (strcmp(topic->name, name) == 0)
        {
            return topic;
        }
    }
    // 没有找到
    return NULL;
}

/**
 * @brief 创建发布者，绑定指定 Topic
 * @param publisher 发布者结构体指针
 * @param topic_name 话题名称
 * @param data_size  消息结构体大小
 * @return true 成功，false 失败
 */
bool Create_Publisher(Publisher_t *publisher,const char *topic_name,size_t data_size)
{
    // 1. 检查发布者是否有效、是否已经绑定
    if (publisher == NULL || publisher->topic != NULL)
    {
        return false;
    }

    // 2. 根据名称查找 Topic
    Topic_t *topic = Find_Topic(topic_name);

    if (topic == NULL)
    {
        return false;
    }

    // 3. 检查消息大小是否一致
    if (topic->data_size != data_size)
    {
        return false;
    }

    // 4. 一个 Topic 只允许一个 Publisher
    if (topic->has_publisher)
    {
        return false;
    }

    // 5. 将 Publisher 绑定到 Topic
    publisher->topic = topic;
    publisher->publish_count = 0;

    // 6. 标记 Topic 已有 Publisher
    topic->has_publisher = true;

    return true;
}

/**
 * @brief 创建订阅者，绑定指定 Topic
 */
bool Create_Subscriber(Subscriber_t *subscriber,const char *topic_name,size_t data_size)
{
    // 1. 检查订阅者是否有效、是否已经绑定
    if (subscriber == NULL || subscriber->topic != NULL)
    {
        return false;
    }

    // 2. 根据名称查找 Topic
    Topic_t *topic = Find_Topic(topic_name);

    if (topic == NULL)
    {
        return false;
    }

    // 3. 检查消息大小是否一致
    if (topic->data_size != data_size)
    {
        return false;
    }

    // 4. 检查订阅者计数是否已满
    if (topic->subscriber_count == UINT16_MAX)
    {
        return false;
    }

    // 5. 将 Subscriber 绑定到 Topic
    subscriber->topic = topic;
    subscriber->read_count = 0;
    subscriber->last_read_time = 0;
    subscriber->last_read_update_count = 0;
    subscriber->last_message_time = 0;
    subscriber->has_read = false;

    // 6. Topic 的订阅者数量加一
    topic->subscriber_count++;

    return true;
}

/**
 * @brief  将数据写入 Topic 最新值缓存
 * @note   仅在任务上下文中调用，data_size 必须与注册时一致
 */
bool Message_Publish(Publisher_t *publisher, const void *data, size_t data_size)
{
    // 检查发布者
    if (publisher == NULL || publisher->topic == NULL || data == NULL)
    {
        return false;
    }

    Topic_t *topic = publisher->topic;

    // 检查话题
    if (!topic->is_active || !topic->has_publisher || topic->data_size == 0 || topic->data_size > sizeof(topic->data) || data_size != topic->data_size)
    {
        return false;
    }

    // 数据拷贝、时间戳及版本号更新放在同一临界区，避免读到半条旧数据半条新数据。
    taskENTER_CRITICAL();

    memcpy(topic->data, data, data_size);
    topic->last_update_time = (uint32_t)xTaskGetTickCount();
    topic->update_count++;
    topic->has_value = true;
    publisher->publish_count++;

    taskEXIT_CRITICAL();
    return true;
}

/* UART interrupt callback publishes data. */
bool Message_PublishFromISR(Publisher_t *publisher, const void *data, size_t data_size)
{
    if (publisher == NULL || publisher->topic == NULL || data == NULL)
    {
        return false;
    }

    Topic_t *topic = publisher->topic;
    if (!topic->is_active || !topic->has_publisher || topic->data_size == 0 || topic->data_size > sizeof(topic->data) || data_size != topic->data_size)
    {
        return false;
    }

    UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
    memcpy(topic->data, data, data_size);
    topic->last_update_time = (uint32_t)xTaskGetTickCountFromISR();
    topic->update_count++;
    topic->has_value = true;
    publisher->publish_count++;
    taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
    return true;
}
/**
 * @brief  读取 Topic 缓存中的最新值
 * @note   只要 Topic 曾经发布过数据，就可以重复读取；不会清空数据。
 *         返回 false 表示没有数据、参数无效或消息长度不匹配。
 */
bool Message_Read(Subscriber_t *subscriber, void *data, size_t data_size)
{
    // 检查订阅者
    if (subscriber == NULL || subscriber->topic == NULL || data == NULL)
    {
        return false;
    }

    const Topic_t *topic = subscriber->topic;

    // 检查话题
    if (!topic->is_active || topic->data_size == 0 || topic->data_size > sizeof(topic->data) || data_size != topic->data_size)
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (!topic->has_value)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    // 每次 Read 都读取最新数据
    memcpy(data, topic->data, data_size);

    // 设计便于调试的变量
    subscriber->last_read_update_count = topic->update_count;
    subscriber->last_message_time = topic->last_update_time;
    subscriber->last_read_time = (uint32_t)xTaskGetTickCount();
    subscriber->read_count++;
    subscriber->has_read = true;

    taskEXIT_CRITICAL();
    return true;
}

/**
 * @brief  判断 Topic 是否超过 timeout_ms 毫秒未发布新消息
 * @return true: 从未发布或已超时；false: 数据仍新鲜
 * @note   本函数检查 Topic 发布，而不是 Subscriber 读取。
 */
bool Message_IsTimeout(const Subscriber_t *subscriber, uint32_t timeout_ms)
{
    if (subscriber == NULL || subscriber->topic == NULL)
    {
        return true;
    }

    const Topic_t *topic = subscriber->topic;
    bool has_value;
    TickType_t published_at;
    TickType_t now;

    taskENTER_CRITICAL();

    has_value = topic->is_active && topic->has_value;
    published_at = (TickType_t)topic->last_update_time;
    now = xTaskGetTickCount();

    taskEXIT_CRITICAL();

    if (!has_value)
    {
        return true;
    }

    // tick 为无符号整数，减法在一次计数器回绕内仍能正确计算间隔。
    TickType_t elapsed_ticks = (TickType_t)(now - published_at);

    // 比较前使用 64 位运算，避免毫秒换算造成精度损失。
    return (uint64_t)elapsed_ticks * 1000ULL > (uint64_t)timeout_ms * configTICK_RATE_HZ;
}
