# Robot SDK

当前机器人的整机交互层。它统一完成 BSP 和输入设备初始化，把硬件中断回调路由到应用层唯一实例，并在 TIM7 的 1 ms 中断中完成电机计算、CAN 发送和离线检测。

该模块与当前硬件连接、CAN ID 和应用对象强绑定，移植到其他机器人时必须检查并修改 `robot_sdk.cpp`。

## 安装

将 `sdk/robot` 目录加入工程，并在 `CMakeLists.txt` 中添加：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sdk/robot/robot_sdk.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sdk/robot
)
```

还需要安装并加入 include 路径：

- BSP：CAN、TIM、UART、USB、DWT、LED、Buzzer、IMU PWM。
- 组件：DR16、VT13、裁判系统。
- 应用：INS、Gimbal、Chassis、Shoot、Vision 的任务头文件和全局唯一实例。
- STM32CubeMX：USB Device CDC、CAN1/2、USART1/3/6、TIM7 及相关 DMA/中断。

## 启动接入

在 HAL 外设初始化完成、RTOS 内核启动之前调用 `robot_sdk_init()`：

```c
#include "robot_sdk.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    // 其余 CubeMX 外设初始化

    MX_USB_DEVICE_Init();
    robot_sdk_init();

    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();
}
```

`robot_sdk_init()` 初始化 DWT、IMU 加热 PWM、RGB LED、蜂鸣器，以及 DR16、VT13 和裁判系统对象。此时只建立对象与外设的绑定，不启动 CAN/UART/TIM 回调。

所有应用任务初始化完成后调用 `robot_sdk_start()`：

```cpp
extern "C" void StartDefaultTask(void *argument)
{
    osThreadFlagsWait(TASK_READY_ALL, osFlagsWaitAll, osWaitForever);
    robot_sdk_start();
    osThreadExit();
}
```

这样可以保证中断回调开始访问 `gimbal`、`chassis`、`shoot`、`ins` 和 `vision` 之前，这些对象已经完成初始化。

## 当前硬件映射

| 外设 | 设备/用途 | 回调目标 |
| --- | --- | --- |
| CAN1 `0x201`-`0x204` | 四个底盘电机 | `chassis.wheel_motor_[0..3]` |
| CAN1 `0x205` | Yaw 电机 | `gimbal.motor_yaw_` |
| CAN2 `0x201` | 拨盘电机 | `shoot.trigger_` |
| CAN2 `0x202`、`0x203` | 左右摩擦轮 | `shoot.friction_left/right_` |
| CAN2 `0x205` | Pitch 电机 | `gimbal.motor_pitch_` |
| USART1，21 字节 | VT13 | `vt13` |
| USART3，18 字节 | DR16 | `dr16` |
| USART6，255 字节 | 裁判系统 | `referee` |
| USB CDC | 视觉通信 | `vision` |
| BMI088 EXTI | IMU 采样 | `ins.bmi088_` 和 INS 线程标志 |
| TIM7，1 kHz | 电机调度与离线检测 | `task1ms_tim7_callback()` |

## 1 ms 调度

TIM7 每次更新中断执行：

1. 计算所有 DJI 电机控制输出。
2. 将 CAN1/CAN2 的 `0x200` 和 `0x1FE` 聚合帧发送出去。
3. 每 `100 ms` 检查遥控器和电机反馈/控制指令是否在线。
4. 每 `1000 ms` 检查裁判系统是否在线。

应用任务只更新电机 target；真正的 PID 计算、发送缓存填充和 CAN 发送由该调度器完成。

## `initialized` 启动门控

`robot_sdk.cpp` 定义全局变量：

```cpp
uint8_t initialized = 0;
```

`robot_sdk_start()` 将其置为 `1`。CAN、UART、USB 和 TIM BSP 的 HAL 回调会先检查该标志，防止系统尚未就绪时访问回调对象。单独移植这些 BSP 时，需要由自己的启动模块提供同名变量或改为项目自己的门控方式。

## 修改检查表

- 电机数量、CAN 总线、反馈 ID 或聚合发送 ID 改变时，修改 CAN 回调和 TIM7 发送列表。
- 遥控器或裁判系统串口改变时，同时修改对象 `init()`、`uart_init()` 和 CubeMX DMA 配置。
- 新增任务时，将任务就绪标志纳入启动屏障，再决定它是否需要由 SDK 路由中断。
- TIM7 回调运行在中断上下文，不要加入阻塞调用、动态内存或耗时业务逻辑。

