# Remote Control

遥控器协议组件，支持 DJI DR16 和 VT13。UART 数据在 BSP 回调中解析并通过 Message Center 发布，上层 Command 模块订阅语义化处理。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/rc/dr16.c
    components/rc/vt13.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/rc
)
```

依赖：

- [`bsp/uart`](../../bsp/uart/README.md)
- [`sdk/message`](../../sdk/message/README.md)
- VT13 额外依赖 [`components/support`](../support/README.md) 的 CRC16

只使用其中一种遥控器时，可以只加入对应 `.c`。

## DR16

CubeMX 典型配置为 UART `100000 baud`、`9-bit word length`、`even parity`、仅接收，实际数据位为 8E1。每帧固定 `18` 字节。

```c
void dr16_uart_callback(uint8_t *buffer, uint16_t length)
{
    Dr16_UartRxCallback(&dr16, buffer, length);
}

void init(void)
{
    Dr16_Init(&dr16, &huart3);
    BspUart_Init(&huart3, dr16_uart_callback, 18);
}
```

解析后发布 `/dr16`，消息类型为 `Dr16_Message_t`，包含四个摇杆通道、两个拨杆、鼠标、键盘和拨轮。

## VT13

当前工程通过 USART1 `921600 8N1` 接收，每帧固定 `21` 字节，帧头为 `0xA9 0x53`，末尾使用 CRC16。VT13 对象句柄仍按 G4 原逻辑绑定 UART5。

```c
void vt13_uart_callback(uint8_t *buffer, uint16_t length)
{
    Vt13_UartRxCallback(&vt13, buffer, length);
}

void init(void)
{
    Vt13_Init(&vt13, &huart5);
    BspUart_Init(&huart1, vt13_uart_callback, 21);
}
```

解析后发布 `/vt13`，消息类型为 `Vt13_Message_t`。

## 离线检测

两种遥控器都应每 `100 ms` 调用一次：

```c
Dr16_CheckAlive100ms(&dr16);
Vt13_CheckAlive100ms(&vt13);
```

该时间窗内没有收到完整帧时，模块标记离线并调用 `BspUart_Reinit()` 重启接收 DMA。上层通过 `Message_IsTimeout()` 判断消息超时。

## 注意事项

- 收到的长度与 UART BSP 配置长度不一致时，当前帧丢弃并重启 DMA。
- DR16 通道绝对值超过 `660` 时发布失能数据；VT13 还会检查帧头和 CRC16。
- 发布发生在 UART 回调上下文，使用 `Message_PublishFromISR()`；修改串口中断优先级时必须满足 FreeRTOS 的 ISR 系统调用要求。
- 当前 Command 逻辑优先使用 DR16；VT13 的语义映射尚未实现。

