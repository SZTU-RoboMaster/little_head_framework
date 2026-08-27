# BSP Layer

BSP 层对 STM32 HAL 和 CubeMX 生成的外设句柄做薄封装，负责启动外设、管理接收缓冲区并把 HAL 回调转发给上层。组件层不直接实现 HAL 中断分发，而是通过 BSP 注册自己的处理函数。

## 模块

| 目录 | 作用 |
| --- | --- |
| [`can`](can/README.md) | CAN1/CAN2 启动、过滤器、收发和回调分发 |
| [`uart`](uart/README.md) | USART1/3/6 Receive-to-Idle DMA 双缓冲接收 |
| [`usb`](usb/README.md) | USB Device CDC 接收回调转发 |
| [`tim`](tim/README.md) | TIM1-TIM14 更新中断回调注册 |
| [`dwt`](dwt/README.md) | Cortex-M DWT 初始化和微秒阻塞延时 |
| [`led`](led/README.md) | TIM5 三通道 RGB LED |
| [`buzzer`](buzzer/README.md) | TIM4 CH3 蜂鸣器 PWM |
| [`imu_pwm`](imu_pwm/README.md) | TIM10 CH1 IMU 加热 PWM |

## 安装原则

1. 先在 CubeMX 中开启模块需要的外设、GPIO、DMA 和中断。
2. 生成 HAL 初始化代码和句柄。
3. 将对应 BSP 的 `.cpp` 和 include 路径加入 CMake。
4. 在上层初始化时注册回调，再允许中断进入业务处理。

CAN、UART、USB 和 TIM 模块使用全局 `initialized` 作为启动门控。当前变量由 `sdk/robot/robot_sdk.cpp` 定义；单独移植 BSP 时，需要由工程提供该变量或改成自己的启动状态。

