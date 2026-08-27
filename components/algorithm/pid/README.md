# PID

离散 PID 控制器，支持前馈、输出限幅、积分限幅、死区、变速积分、积分分离和微分先行。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/algorithm/pid/pid.cpp
    components/algorithm/math_tools/math_tools.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/algorithm/pid
    components/algorithm/math_tools
    Drivers/CMSIS/DSP/Include
)
```

## 初始化

```cpp
#include "pid.h"

Pid speed_pid;

void init()
{
    speed_pid.init(
        40.0f,                 // kp
        0.0f,                  // ki
        0.0f,                  // kd
        0.0f,                  // kf
        0.0f,                  // integral output limit
        16384.0f,              // output limit
        0.001f,                // dt, s
        0.0f,                  // dead zone
        0.0f,                  // integral full error
        0.0f,                  // integral zero error
        0.0f,                  // integral separation threshold
        PID_D_FIRST_DISABLE);
}
```

后续参数均有默认值，简单 PI/PID 可以只传 `kp`、`ki`、`kd`。

## 周期调用

```cpp
speed_pid.set_target(target_omega);
speed_pid.set_feedback(feedback_omega);
speed_pid.calculate();

float output = speed_pid.get_output();
```

失能或模式切换时可清除积分：

```cpp
speed_pid.set_integral_error(0.0f);
```

## 参数说明

| 参数 | 说明 |
| --- | --- |
| `kp/ki/kd` | 比例、积分、微分增益 |
| `kf` | 目标变化量前馈增益 |
| `integral_output_limit` | 积分项输出限幅，`0` 表示不限幅 |
| `output_limit` | 总输出限幅，`0` 表示不限幅 |
| `dt` | 固定计算周期，单位秒 |
| `dead_zone` | 误差死区 |
| `integral_full_error` | 变速积分全速区上限 |
| `integral_zero_error` | 变速积分停止区下限 |
| `integral_separate_threshold` | 超过该误差时清除并停止积分 |
| `d_first` | 启用后只对反馈微分，减小目标突变造成的微分冲击 |

## 注意事项

- `calculate()` 必须按 `dt` 对应的固定周期调用，否则积分和微分尺度会错误。
- `dt` 必须大于零。
- 当 `integral_output_limit` 非零时，`ki` 也必须非零，否则积分限幅计算会除以零。
- 控制对象失能、离线或切换模式时应主动清除积分。

