# Vision Task

视觉通信的应用任务封装。任务初始化 `Vision_t`，以 1 kHz 从 `/ins` 更新姿态并通过 USB CDC 发送；收到有效视觉帧后由任务向 `/vision` 发布。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/vision/vision_task.c
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

`vision.c` 提供全局唯一实例 `Vision_t vision`，Robot SDK 的 USB 接收回调将数据交给该对象解析。

## 任务流程

```c
void vision_task(void *argument)
{
    Vision_Init();
    osThreadFlagsSet(defaultTaskHandle, TASK_READY_VISION);

    for (;;)
    {
        Vision_Publish_New();
        Vision_Send();
        osDelayUntil(next_tick);
    }
}
```

## 接收路径

USB回调只解析和保存有效帧，发布在任务中完成：

```text
USB CDC Receive
    -> USB_Rx_Callback()
    -> vision_usb_callback()
    -> Vision_UsbRxCallback()
    -> Vision_Publish_New() in vision_task
    -> publish /vision
```

协议结构、CRC、分包处理和 Topic 定义见 [`components/vision`](../../components/vision/README.md)。

## 注意事项

- 任务检查上一帧 USB 发送状态；忙时跳过本周期，不覆盖仍在发送的缓冲区。
- 如果不需要固定 1 kHz 发送，可降低任务频率或改用发送完成事件驱动。
- Robot SDK 启动 USB 回调前会等待本任务完成 `Vision_Init()`。

