# INS

惯性导航任务。模块读取 BMI088 数据，使用重力向量 KF 和四元数 EKF 计算姿态，通过 `/ins` 发布欧拉角、角速度、加速度和四元数，同时控制 IMU 加热 PWM。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/INS/INS_task.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/INS
)
```

依赖：

- [`components/bmi088`](../../components/bmi088/README.md)
- [`components/algorithm/pid`](../../components/algorithm/pid/README.md)
- [`bsp/imu_pwm`](../../bsp/imu_pwm/README.md)
- [`sdk/message`](../../sdk/message/README.md)
- CMSIS-RTOS2

## 任务创建

在 `freertos.c` USER CODE 区创建任务，并保留弱入口：

```c
const osThreadAttr_t insTask_attributes = {
    .name = "insTask",
    .stack_size = 1024 * 4,
    .priority = osPriorityRealtime,
};

insTaskHandle = osThreadNew(INS_task, NULL, &insTask_attributes);
```

`INS_task.cpp` 提供 `extern "C" void INS_task(void *)` 强定义。

## 数据就绪接入

Robot SDK 在 BMI088 EXTI 回调中读取传感器，并在 Gyro Data Ready 时唤醒 INS 任务：

```cpp
ins.bmi088_.exti_read_callback(gpio_pin);

if (gpio_pin == INT1_GYRO_Pin && insTaskHandle != NULL)
{
    osThreadFlagsSet(insTaskHandle, INS_DATA_READY_FLAG);
}
```

调用 RTOS From-ISR 能力的中断优先级不能高于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`。当前该值为 `5`，Gyro EXTI 优先级也为 `5`。

## 运行流程

```cpp
ins.init();
osThreadFlagsSet(defaultTaskHandle, TASK_READY_INS);

for (;;)
{
    osThreadFlagsWait(INS_DATA_READY_FLAG, osFlagsWaitAny, osWaitForever);
    ins.update();
    ins.publish();
    ins.temp_control();
}
```

`update()` 固定按 `dt = 0.001 s` 融合，因此需要稳定的 1 kHz Gyro Data Ready。

## 消息接口

模块发布 `/ins`，类型为 `InsMessage`：

| 字段 | 内容 |
| --- | --- |
| `angle[3]` | EKF 欧拉角 |
| `gyro[3]` | BMI088 角速度，rad/s |
| `acc[3]` | BMI088 加速度，m/s² |
| `quaternion[4]` | 姿态四元数 |

Gimbal 和 Vision 订阅该 Topic。

## 温控

启动阶段加热 PWM 固定为 `4999`，温度超过 `43 C` 后切换 PID，将目标温度控制在 `45 C`，输出限制为 `0-4999`。

首次移植必须根据加热电阻、MOS、电源和 IMU 安装重新确认 PID 与最大占空比。

