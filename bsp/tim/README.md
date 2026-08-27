# TIM BSP

通用定时器更新中断回调注册模块。上层为指定 TIM 注册无参数函数，BSP 启动 Base Timer 中断，并在 `HAL_TIM_PeriodElapsedCallback()` 中转发。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/tim/bsp_tim.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/tim
)
```

依赖 CubeMX 生成的 `tim.h`、`tim.c` 和 STM32 HAL TIM 驱动。

## CubeMX 配置

将定时器配置为 Internal Clock/Base Timer，设置 Prescaler 和 Period，并开启 Update interrupt。当前工程使用 TIM7 产生 `1 kHz` 中断：

```text
Timer clock: 84 MHz
Prescaler:   83
Period:      999
Update rate: 1 kHz
IRQ priority: 5
```

## 使用方法

```cpp
#include "bsp_tim.h"

void task_1ms_callback()
{
    // 中断上下文中的短周期工作
}

void device_start()
{
    tim_init(&htim7, task_1ms_callback);
}
```

`tim_init()` 保存回调并调用 `HAL_TIM_Base_Start_IT()`。当前实现为 TIM1-TIM14 各保留一个回调槽位。

## HAL Tick

当前工程使用 TIM6 作为 HAL timebase。BSP 在 TIM6 更新时调用 `HAL_IncTick()`，但不会向用户回调分发 TIM6，因此不要用 `tim_init(&htim6, ...)` 注册业务回调。

## 注意事项

- 回调只有在全局 `initialized != 0` 后才会执行，TIM6 的 `HAL_IncTick()` 不受该门控影响。
- 同一 TIM 再次调用 `tim_init()` 会覆盖原回调。
- 回调运行在中断上下文，禁止阻塞、使用 `osDelay()` 或执行大段业务逻辑。
- 本文件实现了全局 `HAL_TIM_PeriodElapsedCallback()`，其他模块应通过 `tim_init()` 接入。

