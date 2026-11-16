# Message Center

每个 Topic 保存一份最新消息，发布者覆盖该消息，订阅者在任务周期中读取。实现位于 `message_center.c`，不为消息分配动态内存。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE sdk/message/message_center.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE sdk/message)
```

依赖 FreeRTOS 的任务/中断临界区和毫秒 Tick。

## 接入

`RobotSdk_Init()` 先执行 `MessageCenter_Init()`，再调用 `MessageCenter_Topics_Init()` 注册当前整机的 11 个 Topic。注册必须早于各模块创建 Publisher/Subscriber。

```c
Publisher_t publisher = {0};
Subscriber_t subscriber = {0};
Example_Message_t data = {0};

Create_Publisher(&publisher, kExampleTopicName, sizeof(data));
Create_Subscriber(&subscriber, kExampleTopicName, sizeof(data));

Message_Publish(&publisher, &data, sizeof(data));      // 任务上下文
Message_Read(&subscriber, &data, sizeof(data));         // 任务上下文
bool timeout = Message_IsTimeout(&subscriber, 100);    // 100 ms 内未发布
```

示例中的 Topic 名和消息结构体需先定义并用 `Register_Topic(name, sizeof(Example_Message_t))` 注册。`Message_Read()` 在 Topic 有值后可以反复读取相同数据；如需仅处理新消息，对比读取前后的 `subscriber.last_read_update_count`。

UART 接收回调运行在中断上下文，DR16、VT13 和裁判系统使用 `Message_PublishFromISR()`。它使用 FreeRTOS 的中断临界区和 `xTaskGetTickCountFromISR()`，只可在允许调用 FreeRTOS API 的中断优先级执行。当前 USART1/3/6 分别为 7/6/7，系统调用阈值为 5。普通应用任务和视觉任务继续使用 `Message_Publish()`。

## 限制

| 宏 | 默认值 | 作用 |
| --- | ---: | --- |
| `MAX_TOPICS` | 24 | 最多 Topic 数 |
| `MAX_NAME_LEN` | 31 | Topic 名最长字节数 |
| `MAX_DATA_SIZE` | 256 | 单条消息最多字节数 |

同一 Topic 只允许一个 Publisher，可有多个 Subscriber。`Create_Publisher()` 和 `Create_Subscriber()` 会检查已注册的消息大小。发布比读取快时，中间消息被覆盖；没有 FIFO。

当前 Topic 为 `/ins`、`/vision`、`/gimbal`、`/dr16`、`/vt13`、`/command/gimbal`、`/command/chassis`、`/command/shoot`、`/chassis`、`/powercontroller` 和 `/referee`，具体消息结构体见 `message_def.h`。
