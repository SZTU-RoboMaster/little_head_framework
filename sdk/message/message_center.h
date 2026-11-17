//
// Created by hjr on 2026/11/16.
//

#ifndef LITTLE_HEAD_TEST_MESAGE_CENTER_H
#define LITTLE_HEAD_TEST_MESAGE_CENTER_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

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

    uint32_t update_count;
    uint32_t last_update_time;

    uint16_t subscriber_count;
} Topic_t;

typedef struct
{
    Topic_t topics[MAX_TOPICS];
    uint16_t topic_count;
    uint32_t register_error_count;
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
    uint32_t last_read_time;
    uint32_t last_read_update_count;
    uint32_t last_message_time;
    bool has_read;
} Subscriber_t;


extern MessageCenter_t g_message_center;


void MessageCenter_Init(void);


void MessageCenter_Topics_Init(void);


bool Register_Topic(const char *name, size_t data_size);


bool Create_Publisher(Publisher_t *publisher, const char *topic_name, size_t data_size);


bool Create_Subscriber(Subscriber_t *subscriber, const char *topic_name, size_t data_size);


bool Message_Publish(Publisher_t *publisher, const void *data, size_t data_size);

bool Message_PublishFromISR(Publisher_t *publisher, const void *data, size_t data_size);


bool Message_Read(Subscriber_t *subscriber, void *data, size_t data_size);


bool Message_IsTimeout(const Subscriber_t *subscriber, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // LITTLE_HEAD_TEST_MESAGE_CENTER_H
