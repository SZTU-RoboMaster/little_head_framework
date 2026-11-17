# FDCAN BSP

FDCAN1/FDCAN2/FDCAN3 的 HAL 封装，负责启动 FDCAN、配置接收过滤器、发送经典 CAN 标准帧，并把 FIFO0 接收中断转发给注册的上层回调。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/can/bsp_can.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/can
)
```

依赖 CubeMX 生成的 `fdcan.h`、`fdcan.c` 和 STM32 HAL FDCAN 驱动。

## CubeMX 配置

- 启用需要使用的 FDCAN1/FDCAN2/FDCAN3 和对应 GPIO。
- 当前 G4 工程将三路 FDCAN 配置为经典 CAN 帧格式。
- 开启 FIFO0 新消息、Bus-Off 和协议错误通知。

## 使用方法

```c
#include "bsp_can.h"
#include "motor_dji.h"

extern MotorDji_t motor;

void can1_rx_callback(BspCan_RxBuffer_t *rx)
{
    if (rx->header.Identifier == 0x201)
    {
        MotorDji_CanRxCallback(&motor, rx->data);
    }
}

void device_init(void)
{
    BspCan_Init(&hfdcan1, can1_rx_callback);
}
```

发送一个标准数据帧：

```c
uint8_t data[8] = {0};
BspCan_Send(&hfdcan1, 0x200, data, 8);
```

`BspCan_Send()` 返回 `HAL_FDCAN_AddMessageToTxFifoQ()` 的状态值。其长度参数保持 G4 原实现的传递方式。

## 当前行为

- `BspCan_Init()` 配置索引 0 的全通标准帧过滤器、开启通知并启动 FDCAN。
- 默认过滤器把标准数据帧放入 FIFO0，其他帧由全局过滤器拒绝。
- 发送接口只构造标准数据帧；当前接口不支持扩展帧和远程帧发送。
- FIFO0 回调会持续读取直至队列清空；全局 `initialized == 0` 时只清空数据、不向上分发。
- Bus-Off 时按 G4 原实现清除控制器 INIT 位进行恢复。

## 注意事项

- 该实现管理 FDCAN1、FDCAN2 和 FDCAN3；当前 Robot SDK 使用前两路。
- 上层回调运行在 CAN 中断上下文，不要阻塞或调用耗时逻辑。
- `bsp_can.c` 已实现 HAL 的 FIFO0 和错误回调，工程中不要再定义同名强符号。
- 多个电机共用聚合发送帧时，先由电机模块填写缓存，再统一调用 `BspCan_Send()`。

