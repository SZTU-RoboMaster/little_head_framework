# Message Center

面向嵌入式控制任务的轻量发布/订阅中间层。每个 Topic 保存一份最新消息，发布者调用 `publish()` 覆盖该消息，订阅者在自己的任务周期中调用 `update()` 获取更新，不使用回调和动态内存。

## 安装

将 `sdk/message` 目录加入工程，并在 `CMakeLists.txt` 中添加：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sdk/message/message_center.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sdk/message
)
```

模块使用 `inline constexpr`，工程需要启用 C++17。时间戳直接使用 STM32 HAL 的 `HAL_GetTick()`，因此还需要 HAL 头文件和毫秒时基。

## 发布消息

先在 `message_def.h` 中定义 Topic 名称和消息类型：

```cpp
static constexpr char kExampleTopicName[] = "/example";

struct ExampleMessage
{
    float value = 0.0f;
};
```

发布者在初始化时注册 Topic，在需要时发布：

```cpp
Publisher<ExampleMessage> publisher;

void init()
{
    publisher = MessageCenter::instance().advertise<ExampleMessage>(kExampleTopicName);
}

void publish_example()
{
    ExampleMessage msg{.value = 1.0f};
    publisher.publish(msg);
}
```

## 订阅消息

订阅者同样在初始化时绑定 Topic，并在自己的周期中主动更新：

```cpp
Subscriber<ExampleMessage> subscriber;
ExampleMessage msg;

void init()
{
    subscriber = MessageCenter::instance().subscribe<ExampleMessage>(kExampleTopicName);
}

void update()
{
    if (subscriber.update(msg))
    {
        // 本周期收到了发布者的新数据
    }

    if (!subscriber.is_fresh(100))
    {
        // 从未收到消息，或最后一次收到的消息已超过 100 ms
    }
}
```

`update()` 只在 Topic 序号发生变化时返回 `true`。同一条消息不会被同一个订阅者重复读取，但不同订阅者互不影响。

## 工作方式

- `MessageCenter::instance()` 返回全局单例。
- `advertise<T>()` 注册发布者，`subscribe<T>()` 注册订阅者。
- Topic 按名称匹配，同名 Topic 的消息大小必须一致。
- 每个 Topic 只允许一个发布者，可以有多个订阅者。
- `publish()` 复制消息、递增序号，并用 `HAL_GetTick()` 写入发布时间戳。
- `update()` 复制最新值，不保存历史消息，不阻塞任务。
- `is_fresh()` 根据订阅者最后一次成功更新的消息时间戳判断新鲜度。

## 静态资源限制

默认限制定义在 `message_center.h`：

| 常量 | 默认值 | 含义 |
| --- | ---: | --- |
| `kMaxTopics` | 24 | 最大 Topic 数量 |
| `kMaxTopicNameLength` | 31 | Topic 名称最大字符数，不含结尾 `\0` |
| `kMaxPayloadSize` | 256 | 单条消息最大字节数 |

所有 Topic 和消息缓存都由 `MessageCenter` 静态持有，不调用 `malloc`。超过上限、重复注册发布者、同名 Topic 消息大小不一致或 Topic 名称非法时，注册返回无效的 Publisher/Subscriber，后续 `publish()` 或 `update()` 返回 `false`。

## 当前整机 Topic

| Topic | 消息类型 | 发布者 | 订阅者 |
| --- | --- | --- | --- |
| `/ins` | `InsMessage` | INS | Gimbal、Vision |
| `/vision` | `VisionMessage` | Vision | Gimbal |
| `/gimbal` | `GimbalMessage` | Gimbal | Chassis |
| `/dr16` | `Dr16Message` | DR16 | Command |
| `/vt13` | `Vt13Message` | VT13 | Command |
| `/command/gimbal` | `GimbalCmdMessage` | Command | Gimbal |
| `/command/chassis` | `ChassisCmdMessage` | Command | Chassis |
| `/command/shoot` | `ShootCmdMessage` | Command | Shoot |
| `/chassis` | `ChassisMessage` | Chassis | Power Control |
| `/powercontroller` | `PowerControllerMessage` | Power Control | Chassis |
| `/referee` | `RefereeMessage` | Referee | Shoot、Power Control |

## 注意事项

- 消息类型应使用可直接按字节复制的数据结构，不要包含拥有动态资源的对象、虚函数或裸资源所有权。
- 当前实现latest-ready而不是fifo，发布速度高于订阅速度时，中间消息会被覆盖。
- 当前实现没有临界区保护。
- `is_fresh()` 使用 32 位毫秒 tick 的无符号减法，可正确处理一次 tick 回绕。

