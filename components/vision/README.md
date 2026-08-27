# Vision Protocol

机器人与视觉上位机之间的 USB CDC 二进制协议组件。模块从 `/ins` 获取姿态并周期发送，同时解析视觉端返回的目标状态并发布 `/vision`。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/vision/vision.cpp
    components/support/CRC8_CRC16.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/vision
    components/support
)
```

依赖 [`bsp/usb`](../../bsp/usb/README.md) 和 [`sdk/message`](../../sdk/message/README.md)。

## 协议

收发结构体均使用 `packed` 布局、小端浮点数和 CRC16：

| 方向 | 结构体 | 主要字段 |
| --- | --- | --- |
| Robot -> Vision | `RobotData` | `HJ` 帧头、模式、Yaw/Pitch、角速度、四元数、射速、弹丸数、CRC16 |
| Vision -> Robot | `VisionData` | `HJ` 帧头、目标角度/速度/加速度、锁定、开火命令、CRC16 |

`target_lock == 49` 表示锁定，`50` 表示未锁定。协议两端必须使用相同结构体布局和字节序。

## 初始化与接收

```cpp
Vision vision;

void vision_usb_callback(uint8_t *buffer, uint32_t length)
{
    vision.usb_rx_callback(buffer, length);
}

void init()
{
    vision.init();
    usb_init(vision_usb_callback);
}
```

接收解析器支持 USB 分包和一次收到多帧：它在 512 字节缓存中寻找 `HJ` 帧头，等待完整 `VisionData`，通过 CRC16 后更新数据，校验失败时逐字节重新同步。

有效数据以 `VisionMessage` 发布到 `/vision`。

## 周期发送

```cpp
void vision_loop_1ms()
{
    vision.update_tx(); // 从 /ins 更新姿态
    vision.send();      // 通过 USB CDC 发送 RobotData
}
```

`send()` 计算 CRC16 后调用 `CDC_Transmit_FS()`。当前接口不检查返回值，若发送周期高于 USB 实际吞吐，`USBD_BUSY` 时该帧会丢失；需要可靠发送时应增加发送完成状态或重试队列。

## 消息接口

| 方向 | Topic | 消息类型 |
| --- | --- | --- |
| 订阅 | `/ins` | `InsMessage` |
| 发布 | `/vision` | `VisionMessage` |

## 注意事项

- 当前结构体仅测试用，与队内不同。
- 修改协议结构体后必须同步修改视觉端，并重新确认 `sizeof()` 和 CRC 覆盖范围。
- 当前 `usb_rx_callback()` 在每次 USB 接收后都会发布缓存中的视觉数据；无效或半帧不会更新 payload，但仍可能刷新 Topic 时间戳。
- USB 回调与任务发送可能并发访问对象，扩展字段时应保持读写关系清晰。
- `shoot_speed` 和 `bullet_count` 当前没有由业务模块更新，默认发送零。

