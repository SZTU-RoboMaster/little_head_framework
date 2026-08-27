# Remote Control

遥控器协议组件，支持 DJI DR16 和 VT13。UART 数据在 BSP 回调中解析并通过 Message Center 发布，上层 Command 模块订阅语义化处理。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/rc/dr16.cpp
    components/rc/vt13.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/rc
)
```

依赖：

- [`bsp/uart`](../../bsp/uart/README.md)
- [`sdk/message`](../../sdk/message/README.md)
- VT13 额外依赖 [`components/support`](../support/README.md) 的 CRC16

只使用其中一种遥控器时，可以只加入对应 `.cpp`。

## DR16

CubeMX 典型配置为 UART `100000 baud`、`9-bit word length`、`even parity`、仅接收，实际数据位为 8E1。每帧固定 `18` 字节。

```cpp
Dr16 dr16;

void dr16_uart_callback(uint8_t *buffer, uint16_t length)
{
    dr16.uart_rx_callback(buffer, length);
}

void init()
{
    dr16.init(&huart3);
    uart_init(&huart3, dr16_uart_callback, 18);
}
```

解析成功后发布 `/dr16`，消息类型为 `Dr16Message`，包含四个摇杆通道、两个拨杆、鼠标、键盘和拨轮。

## VT13

当前工程使用 USART1 `921600 8N1`，每帧固定 `21` 字节，帧头为 `0xA9 0x53`，末尾使用 CRC16。

```cpp
Vt13 vt13;

void vt13_uart_callback(uint8_t *buffer, uint16_t length)
{
    vt13.uart_rx_callback(buffer, length);
}

void init()
{
    vt13.init(&huart1);
    uart_init(&huart1, vt13_uart_callback, 21);
}
```

解析成功后发布 `/vt13`，消息类型为 `Vt13Message`。

## 离线检测

两种遥控器都应每 `100 ms` 调用一次：

```cpp
dr16.check_alive_100ms();
vt13.check_alive_100ms();
```

该时间窗内没有收到完整帧时，模块标记离线并调用 `uart_reinit()` 重启接收 DMA。上层仍应使用消息的 `is_fresh(100)` 做控制失能，不能只依赖组件内部状态。

## 注意事项

- 收到的长度与 UART BSP 配置长度不一致时，当前帧丢弃并重启 DMA。
- DR16 通道绝对值超过 `660` 时发布失能数据；VT13 还会检查帧头和 CRC16。
- 发布发生在 UART 回调上下文，当前 Message Center 没有临界区保护，不要让同一 Topic 有其他发布者。
- 当前 Command 逻辑优先使用 DR16；VT13 的语义映射尚未实现。

