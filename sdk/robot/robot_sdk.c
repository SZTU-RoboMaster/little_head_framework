//
// Created by hjr on 2026/11/11.
//

#include "robot_sdk.h"

#include "main.h"
#include "message_center.h"
#include "message_def.h"

void MessageCenter_Topics_Init(void)
{
    // 由 Robot SDK 统一列出本机的话题，长度从消息类型计算。
    static const struct
    {
        const char *name;
        size_t data_size;
    } topics[] = {
        {kInsTopicName, sizeof(InsMessage)},
        {kVisionTopicName, sizeof(VisionMessage)},
        {kGimbalTopicName, sizeof(GimbalMessage)},
        {kDr16TopicName, sizeof(Dr16Message)},
        {kVt13TopicName, sizeof(Vt13Message)},
        {kCmdGimbalTopicName, sizeof(GimbalCmdMessage)},
        {kCmdChassisTopicName, sizeof(ChassisCmdMessage)},
        {kCmdShootTopicName, sizeof(ShootCmdMessage)},
        {kChassisTopicName, sizeof(ChassisMessage)},
        {kPowerControllerTopicName, sizeof(PowerControllerMessage)},
        {kRefereeTopicName, sizeof(RefereeMessage)},
    };

    const size_t topic_count = sizeof(topics) / sizeof(topics[0]);
    for (size_t i = 0; i < topic_count; ++i)
    {
        if (!Register_Topic(topics[i].name, topics[i].data_size))
        {
            Error_Handler();
            return;
        }
    }
}

void robot_sdk_message_init(void)
{
    // 先清空消息中心，再注册话题。模块应在这之后绑定。
    MessageCenter_Init();
    MessageCenter_Topics_Init();
}
