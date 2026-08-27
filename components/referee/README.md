# Referee

RoboMaster 裁判系统串口协议解析组件。模块使用流式状态机处理分包和粘包，校验帧头 CRC8 与整帧 CRC16，并保存各类裁判系统数据。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/referee/referee.cpp
    components/support/CRC8_CRC16.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/referee
    components/support
)
```

依赖 [`bsp/uart`](../../bsp/uart/README.md) 和 [`sdk/message`](../../sdk/message/README.md)。

## CubeMX 配置

裁判系统串口使用 `115200 8N1`。当前工程绑定 USART6 RX DMA，UART BSP 缓冲区长度为 `255` 字节。

## 使用方法

```cpp
Referee referee;

void referee_uart_callback(uint8_t *buffer, uint16_t length)
{
    referee.uart_rx_callback(buffer, length);
}

void init()
{
    referee.init(&huart6);
    uart_init(&huart6, referee_uart_callback, 255);
}

void alive_1s()
{
    referee.check_alive_1000ms();
}
```

每次 UART 数据到达后，状态机继续解析尚未完成的帧，因此上层不需要按完整协议帧切分 DMA 数据。

## 消息接口

模块发布 `/referee`，类型为 `RefereeMessage`，当前向其他模块公开：

- 17 mm 枪口热量。
- 枪口热量上限和冷却值。
- 底盘功率上限。
- 缓冲能量。

发射模块和底盘功控模块订阅该 Topic。完整协议数据结构保存在 `Referee` 对象内部，定义位于 `referee_protocol.h`。

## 离线处理

`check_alive_1000ms()` 检查最近 1 秒是否收到 UART 数据；离线时标记 `REFEREE_STATUS_DISABLE` 并重启 UART DMA。业务模块仍应通过 `/referee` 的消息时间戳设置自己的安全回退值。

## 注意事项

- 协议结构体使用 `packed` 布局，修改字段后必须对照当赛季官方协议确认长度和字节序。
- `REF_PROTOCOL_FRAME_MAX_SIZE` 当前为 `255` 字节，超过该长度的帧会被状态机丢弃。
- UART 回调运行在中断上下文，新增数据处理时避免耗时操作。
- 当前模块主要实现接收解析和摘要消息发布，UI/交互数据发送接口尚未对外封装。

