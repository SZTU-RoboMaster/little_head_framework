# Application Layer

Application 层实现机器人业务语义和控制流程。每个主模块拥有自己的唯一对象和 CMSIS-RTOS2 任务入口，通过 Message Center 交换数据；电机 target 由应用更新，实际电机计算和 CAN 发送由 Robot SDK 的 1 ms 调度完成。

## 模块

| 目录 | 作用 |
| --- | --- |
| [`initial`](initial/README.md) | 等待各任务初始化完成并启动 Robot SDK |
| [`INS`](INS/README.md) | IMU 采样、姿态融合、加热控制和 `/ins` 发布 |
| [`command`](command/README.md) | 遥控器仲裁和控制语义发布 |
| [`gimbal`](gimbal/README.md) | 云台模式、手动/视觉控制和双环 PID |
| [`chassis`](chassis/README.md) | 麦轮底盘、跟随/小陀螺和功率控制 |
| [`shoot`](shoot/README.md) | 摩擦轮、拨盘、单发/连发和堵转处理 |
| [`vision`](vision/README.md) | 视觉协议的周期发送任务 |

## 当前任务

| 任务 | 触发方式 | CMSIS-RTOS2 优先级 | Stack |
| --- | --- | --- | ---: |
| `StartDefaultTask` | 启动屏障，完成后退出 | `Realtime7` | 512 B |
| `INS_task` | BMI088 Gyro Data Ready 线程标志 | `Realtime` | 4096 B |
| `gimbal_task` | 1 ms 周期 | `High` | 2048 B |
| `chassis_task` | 1 ms 周期 | `AboveNormal` | 2048 B |
| `shoot_task` | 1 ms 周期 | `AboveNormal` | 2048 B |
| `vision_task` | 1 ms 周期 | `AboveNormal` | 2048 B |

任务属性和弱入口定义在 `Core/Src/freertos.c` 的 USER CODE 区；各 application 的 `.cpp` 使用 `extern "C"` 提供同名强定义。

## 新增应用模块

1. 在模块目录中实现对象和任务入口。
2. 将源文件和 include 路径加入 CMake。
3. 在 `freertos.c` USER CODE 区声明并通过 `osThreadNew()` 创建任务。
4. 若任务必须先于设备中断启动，在 `initial_task.h` 增加就绪标志，并在模块初始化后设置该标志。
5. 在 `message_def.h` 定义需要发布/订阅的 Topic 和消息类型。
6. 需要硬件中断路由时，在 `robot_sdk.cpp` 中绑定该模块的唯一实例。

应用任务不应直接发送同组 DJI CAN 聚合帧，否则会与 Robot SDK 的电机调度重复发送。

