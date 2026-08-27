# Buzzer BSP

使用 TIM4 CH3 PWM 驱动蜂鸣器，可在运行时设置 Prescaler 和比较值。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/buzzer/bsp_buzzer.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/buzzer
)
```

## CubeMX 配置

当前工程将 PD14 配置为 TIM4 CH3 PWM，TIM4 Period 为 `20999`。

## 使用方法

```cpp
#include "bsp_buzzer.h"

buzzer_init();
buzzer_on(839, 10500); // 设置分频和占空比
HAL_Delay(100);
buzzer_off();
```

`buzzer_init()` 启动 PWM；`buzzer_on(psc, pwm)` 更新预分频和比较值；`buzzer_off()` 将比较值清零。

## 注意事项

- `pwm` 不应超过定时器 ARR。
- 修改 Prescaler 后频率按下一个更新事件生效，具体行为取决于定时器预装载配置。
- RTOS 任务中不要用 `HAL_Delay()` 编排提示音，应使用任务状态机或非阻塞调度。

