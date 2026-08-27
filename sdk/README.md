# SDK Layer

SDK 层位于应用层与组件/BSP 层之间，提供跨模块通信和整机启动调度。应用模块通过 SDK 获取消息和设备服务，避免在各任务中重复编写底层回调分发代码。

## 模块

| 目录 | 作用 | 可移植性 |
| --- | --- | --- |
| [`message`](message/README.md) | 静态发布/订阅消息中心和整机消息定义 | 可独立复用 |
| [`robot`](robot/README.md) | 当前机器人的外设启动、回调路由和 1 ms 设备调度 | 需要按机器人修改 |

## 依赖方向

```text
application
    |
    v
sdk/message

sdk/robot ---> bsp + components + application 中的唯一设备实例
```

`message` 不依赖具体应用。`robot` 是整机交互层，允许访问云台、底盘、发射和 INS 的全局唯一实例，用于把 CAN、UART、USB、GPIO 和定时器回调路由到正确对象。

## 安装原则

- 只需要模块通信时，安装 `sdk/message`。
- 复用当前整机硬件连接和启动流程时，再安装 `sdk/robot`。
- 更换 CAN ID、UART、定时器或电机数量后，应修改 `robot_sdk.cpp` 中的回调映射。
- 新增消息时，在 `message_def.h` 中同时增加 Topic 名称和消息结构体。

