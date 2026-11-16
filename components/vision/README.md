# Vision Protocol

机器人与视觉上位机之间的 USB CDC 二进制协议组件。模块从 `/ins` 获取姿态并发送，同时解析视觉端返回的目标状态并发布 `/vision`。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/vision/vision.c
    components/support/CRC8_CRC16.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/vision
    components/support
)
```

依赖 [`bsp/usb`](../../bsp/usb/README.md) 和 [`sdk/message`](../../sdk/message/README.md)；Topic 在 `RobotSdk_Init()` 中注册。

## 协议

结构体均使用 `packed` 布局和小端浮点数，不改动原有字段次序。编译时检查长度：`FrameHeader` 为 5 字节，`VisionData` 为 58 字节，`RobotCtrlData` 为 37 字节。

| 方向 | 结构体 | 内容 |
| --- | --- | --- |
| Robot → Vision | `VisionData` | 姿态、角速度、四元数、射速、弹丸数、CRC16 |
| Vision → Robot | `RobotCtrlData` | 目标角度、速度、加速度、锁定与开火指令、CRC16 |

帧头为 `0xA5`，帧尾为 `0x0D 0x0A`。`target_lock == 49` 表示锁定，`50` 表示未锁定。

## 初始化与接收

```c
Vision_Init();
usb_init(vision_usb_callback);

// SDK 的 vision_usb_callback 中：
Vision_UsbRxCallback(buf, len);
```

USB 回调在中断中解析分包、完整帧和 CRC16，只保存最新的有效帧。`Vision_Publish_New()` 在视觉任务中发布 `Vision_Message_t`，避免在中断里调用消息中心。单次接收超过 512 字节时保留末尾 512 字节，以免写出缓冲区。

## 周期发送

```c
Vision_Publish_New();
Vision_Send();
```

`Vision_Send()` 读取 INS、填充协议字段与 CRC，并通过 USB CDC 发送。上一帧仍在发送时本周期跳过，确保发送缓冲区不会在 USB 使用期间被改写。上层可根据实际 USB 吞吐量调整视觉任务周期。

## 消息接口

| 方向 | Topic | 消息类型 |
| --- | --- | --- |
| 订阅 | `/ins` | `Ins_Message_t` |
| 发布 | `/vision` | `Vision_Message_t` |

## 注意事项

- 当前接收解析沿用原逻辑：检查帧头第一个字节及 CRC16，尚未检查头部 CRC8、`cmd_id` 或帧尾。需要两端确认这些字段的约定后再补充校验。
- USB 中断中仍进行 CRC 与分包整理；收到大量错误字节时会逐字节重新同步，可能占用较长中断时间。
- 多个有效帧在同一任务周期到达时仅发布最后一帧；消息中心本身也是最新值缓存。
