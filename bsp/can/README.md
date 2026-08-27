# CAN BSP

CAN1/CAN2 的 HAL 封装，负责启动 CAN、配置接收过滤器、发送标准帧，并把 FIFO0/FIFO1 接收中断转发给注册的上层回调。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/can/bsp_can.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/can
)
```

依赖 CubeMX 生成的 `can.h`、`can.c` 和 STM32 HAL CAN 驱动。

## CubeMX 配置

- 启用需要使用的 CAN1/CAN2 和对应 GPIO。
- 配置总线波特率；当前工程两路 CAN 均为 `1 Mbit/s`。
- 开启 `RX0` 和 `RX1` 中断；当前工程抢占优先级为 `5`。
- 使用 CAN2 时必须同时使能 CAN1 时钟，这是 STM32F4 双 CAN 的硬件要求。

## 使用方法

```cpp
#include "bsp_can.h"

void can1_rx_callback(CanRxBuffer *rx)
{
    if (rx->header.StdId == 0x201)
    {
        motor.can_rx_callback(rx->data);
    }
}

void device_init()
{
    can_init(&hcan1, can1_rx_callback);
}
```

发送一个标准数据帧：

```cpp
uint8_t data[8] = {};
can_data_send(&hcan1, 0x200, data, 8);
```

`can_data_send()` 返回 `HAL_CAN_AddTxMessage()` 的状态值。长度应为 `0-8` 字节。

## 当前行为

- `can_init()` 启动 CAN，并开启 FIFO0/FIFO1 消息挂起中断。
- CAN1 使用 Filter Bank 0/1，CAN2 使用 14/15。
- 默认过滤器接收全部标准数据帧，分别绑定 FIFO0 和 FIFO1。
- 发送接口只构造标准数据帧；当前接口不支持扩展帧和远程帧发送。
- 接收回调只有在全局 `initialized != 0` 后才会向上分发。

## 注意事项

- 该实现只管理 CAN1 和 CAN2，传入其他 CAN 实例不会注册管理对象。
- 上层回调运行在 CAN 中断上下文，不要阻塞或调用耗时逻辑。
- `bsp_can.cpp` 已实现 HAL 的 FIFO0/FIFO1 回调，工程中不要再定义同名强符号。
- 多个电机共用聚合发送帧时，先由电机模块填写缓存，再统一调用 `can_data_send()`。

