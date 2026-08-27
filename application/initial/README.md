# Initial Task

整机启动屏障。Default Task 等待 INS、Gimbal、Chassis、Shoot 和 Vision 五个任务完成各自初始化，然后调用 `robot_sdk_start()` 开启 UART、CAN、USB 回调和 TIM7 电机调度，最后退出自身线程。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/initial/initial_task.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/initial
)
```

依赖 [`sdk/robot`](../../sdk/robot/README.md) 和 CMSIS-RTOS2。

## FreeRTOS 接入

CubeMX 生成的 `freertos.c` 保留 Default Task，并提供弱定义：

```c
__weak void StartDefaultTask(void *argument)
{
    for (;;)
    {
        osDelay(1);
    }
}
```

`initial_task.cpp` 的 `extern "C"` 强定义会覆盖它：

```cpp
extern "C" void StartDefaultTask(void *argument)
{
    osThreadFlagsWait(TASK_READY_ALL, osFlagsWaitAll, osWaitForever);
    robot_sdk_start();
    osThreadExit();
}
```

Default Task 当前使用最高的 `osPriorityRealtime7`，但绝大部分时间阻塞在事件标志上，启动完成后立即退出。

## 子任务接入

每个子任务应在对象初始化成功后通知 Default Task：

```cpp
module.init();
osThreadFlagsSet(defaultTaskHandle, TASK_READY_MODULE);
```

新增或移除任务时同步修改 `initial_task.h` 中的单项标志和 `TASK_READY_ALL`。若某任务未置位，`robot_sdk_start()` 将永远不会执行。

## 启动顺序

```text
main: HAL/CubeMX init -> robot_sdk_init()
                         |
                         v
                  osKernelStart()
                         |
        +----------------+----------------+
        | 各 application task 初始化对象 |
        +----------------+----------------+
                         |
                  设置 TASK_READY_x
                         |
                         v
Default Task: robot_sdk_start() -> osThreadExit()
```

该顺序保证外设中断开始访问应用对象前，对象、PID、Publisher 和 Subscriber 均已初始化。

