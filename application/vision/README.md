# Vision Task

视觉通信的应用任务封装。任务初始化 `Vision` 组件，并以 1 kHz 从 `/ins` 更新姿态后通过 USB CDC 向视觉上位机发送机器人数据。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/vision/vision_task.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/vision
)
```

依赖 [`components/vision`](../../components/vision/README.md)、[`application/initial`](../initial/README.md) 和 CMSIS-RTOS2。

## 任务创建

```c
const osThreadAttr_t visionTask_attributes = {
    .name = "visionTask",
    .stack_size = 512 * 4,
    .priority = osPriorityAboveNormal,
};

visionTaskHandle = osThreadNew(vision_task, NULL, &visionTask_attributes);
```

`vision_task.cpp` 提供全局唯一实例 `Vision vision`，Robot SDK 的 USB 接收回调直接访问该对象。

## 任务流程

```cpp
extern "C" void vision_task(void *argument)
{
    vision.init();
    osThreadFlagsSet(defaultTaskHandle, TASK_READY_VISION);

    for (;;)
    {
        vision.update_tx();
        vision.send();
        osDelayUntil(next_tick);
    }
}
```

## 接收路径

视觉接收不经过该任务轮询：

```text
USB CDC Receive
    -> USB_Rx_Callback()
    -> vision_usb_callback()
    -> vision.usb_rx_callback()
    -> publish /vision
```

协议结构、CRC、分包处理和 Topic 定义见 [`components/vision`](../../components/vision/README.md)。

## 注意事项

- 当前发送频率为 1 kHz，但 `CDC_Transmit_FS()` 可能返回 `USBD_BUSY`；实际可用频率应按 USB 帧长和上位机处理能力确认。
- 如果不需要固定 1 kHz 发送，可降低任务频率或改用发送完成事件驱动。
- Robot SDK 启动 USB 回调前会等待本任务完成 `vision.init()`。

