# IMU Heater PWM

使用 TIM10 CH1 输出 IMU 加热 PWM，供 INS 温控 PID 调节 BMI088 加热功率。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/imu_pwm/bsp_imu_pwm.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/imu_pwm
)
```

## CubeMX 配置

当前工程将 PF6 配置为 TIM10 CH1 PWM，Period 为 `4999`。移植时应根据加热电路和目标 PWM 频率重新配置定时器。

## 使用方法

```cpp
#include "bsp_imu_pwm.h"

imu_pwm_init();
imu_pwm_set(2500); // 约 50% 占空比
imu_pwm_set(0);    // 关闭加热
```

`imu_pwm_set()` 直接写比较值，调用者负责把输出限制在 `0-ARR`。

## 注意事项

- 加热器属于高功率负载，首次调试必须限制最大 PWM 并监控 IMU 温度。
- 温度传感器失效或任务停止时，当前 BSP 不会自动关闭 PWM；安全策略应由 INS 应用层实现。
- 更换定时器通道后需要同步修改 `bsp_imu_pwm.cpp`。

