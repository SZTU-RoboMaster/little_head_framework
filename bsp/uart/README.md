# UART BSP

基于 HAL Receive-to-Idle DMA 的 UART 接收模块。空闲事件发生时将静态缓冲区和实际长度交给上层回调，然后重新启动接收。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/uart/bsp_uart.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/uart
)
```

依赖 CubeMX 生成的 `usart.h`、`usart.c`、DMA 初始化代码和 STM32 HAL UART/DMA 驱动。

## CubeMX 配置

- 启用 UART 异步模式和接收引脚。
- 为 UART RX 配置 DMA，并开启 UART 和 DMA 中断。
- 当前实现只为 USART1、USART3、UART5 提供管理对象。
- 当前工程配置：USART1 `921600 8N1`，USART3 `100000 8E1`，UART5 `115200 8N1`。

## 使用方法

```c
#include "bsp_uart.h"

void uart3_callback(uint8_t *buffer, uint16_t length)
{
    /* 在这里把接收到的数据交给协议解析模块 */
}

void device_start(void)
{
    BspUart_Init(&huart3, uart3_callback, 18);
}
```

第三个参数是单个接收缓冲区长度，最大不能超过 `UART_BUFFER_SIZE`，当前值为 `256` 字节。

接收帧错位或 UART 出错时可以重启 DMA：

```c
BspUart_Reinit(&huart3);
```

## 工作方式

- `BspUart_Init()` 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 启动单缓冲接收，并关闭 DMA 半传输中断。
- `HAL_UARTEx_RxEventCallback()` 向上层交付本次数据后重新启动接收。
- `HAL_UART_ErrorCallback()` 停止、清理并重新启动对应 UART 的接收 DMA。
- 回调只有在全局 `initialized != 0` 后才会分发。

## 注意事项

- 模块只封装接收；发送仍直接使用 HAL UART API 或另行封装。
- 上层回调运行在 UART/DMA 中断上下文，应只解析或转交数据，不能阻塞。
- 同一 UART 只能注册一个回调。
- 本文件实现 HAL UART 接收事件和错误回调，其他模块不要重复定义同名函数。

