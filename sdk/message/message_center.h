/**
 * @file template.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/
inline constexpr std::size_t kMaxTopics = 24;
inline constexpr std::size_t kMaxTopicNameLength = 31;
inline constexpr std::size_t kMaxPayloadSize = 256;

/* Exported types ------------------------------------------------------------*/

// forward declaration
template <typename T> class Publisher;
template <typename T> class Subscriber;
class MessageCenter;

/**
 * @brief Specialized Topic
 *
 */
class Topic
{
public:
private:
    friend class MessageCenter;
    template <typename T> friend class Publisher;
    template <typename T> friend class Subscriber;

    char name_[kMaxTopicNameLength + 1];
    uint8_t data_[kMaxPayloadSize];
    std::size_t payload_size_;

    uint32_t sequence_;
    uint32_t timestamp_ms_;

    bool is_active_;
    bool has_publisher_;
    bool has_value_;

private:
    bool publish_msg(const void *data);

    bool update_msg(void *data, uint32_t &last_sequence, uint32_t &last_timestamp_ms,
                    bool &received);

    bool is_fresh(uint32_t last_timestamp_ms, bool received, uint32_t timeout_ms) const;
};

/**
 * @brief Specialized Publisher
 *
 */
template <typename T> class Publisher
{
    static_assert(sizeof(T) <= kMaxPayloadSize);

public:
    Publisher() = default;

    bool publish(const T &data) const
    {
        return topic_ != nullptr && topic_->publish_msg(&data);
    }

private:
    friend class MessageCenter;

    explicit Publisher(Topic *topic) : topic_(topic)
    {
    }

    Topic *topic_ = nullptr;
};

/**
 * @brief Specialized Subscriber
 *
 */
template <typename T> class Subscriber
{
    static_assert(sizeof(T) <= kMaxPayloadSize);

public:
    Subscriber() = default;

    bool update(T &data)
    {
        return topic_ != nullptr &&
               topic_->update_msg(&data, last_sequence_, last_timestamp_ms_, received_);
    }

    bool is_fresh(uint32_t timeout_ms) const
    {
        return topic_ != nullptr && topic_->is_fresh(last_timestamp_ms_, received_, timeout_ms);
    }

private:
    friend class MessageCenter;

    explicit Subscriber(Topic *topic) : topic_(topic)
    {
    }

    Topic *topic_ = nullptr;
    uint32_t last_sequence_ = 0;
    uint32_t last_timestamp_ms_ = 0;
    bool received_ = false;
};

/**
 * @brief Specialized MessageCenter
 *
 */
class MessageCenter
{
public:
    static MessageCenter &instance();

    template <typename T> Publisher<T> advertise(const char *topic_name)
    {
        Topic *topic = register_topic(topic_name, sizeof(T), true);
        return Publisher<T>(topic);
    }

    template <typename T> Subscriber<T> subscribe(const char *topic_name)
    {
        Topic *topic = register_topic(topic_name, sizeof(T), false);
        return Subscriber<T>(topic);
    }

private:
    Topic *register_topic(const char *topic_name, std::size_t payload_size, bool is_publisher);

    Topic topics_[kMaxTopics];
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
