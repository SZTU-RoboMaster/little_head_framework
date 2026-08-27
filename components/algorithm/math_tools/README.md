# Math Tools

控制代码常用的数学辅助函数，包括单位换算、快速平方根倒数、整数/浮点映射、三次曲线映射和周期量归一化。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/algorithm/math_tools/math_tools.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/algorithm/math_tools
    Drivers/CMSIS/DSP/Include
)
```

模块依赖 STM32 HAL 和 CMSIS-DSP 的 `arm_math.h`。Cortex-M4 工程应定义 `ARM_MATH_CM4`。

## API

| 接口 | 作用 |
| --- | --- |
| `RPM_TO_RADPS` | RPM 转 rad/s 的系数 |
| `DEG_TO_RAD` | 角度转弧度的系数 |
| `inv_sqrt(x)` | 快速计算 `1 / sqrt(x)` |
| `uint_to_float(...)` | 将指定 bit 宽度的无符号整数映射到浮点区间 |
| `float_to_uint(...)` | 将浮点区间映射到指定 bit 宽度的无符号整数 |
| `cubic_map(x, expo)` | 在线性输入和三次输入之间插值 |
| `wrap_center(x, modulus)` | 将周期量归一化到 `[-modulus/2, modulus/2)` |

## 使用示例

```cpp
#include "math_tools.h"

float omega = motor_rpm * RPM_TO_RADPS;
float angle = wrap_center(total_angle, 2.0f * PI);
float command = cubic_map(joystick, 0.5f);

uint16_t packed = float_to_uint(torque, -10.0f, 10.0f, 12);
float unpacked = uint_to_float(packed, -10.0f, 10.0f, 12);
```

## 注意事项

- `float_to_uint()` 不做输入限幅，调用前应先将输入限制在目标区间。
- 映射函数的 `bits` 必须与协议位宽一致，并避免传入会导致整数移位溢出的值。
- `inv_sqrt()` 使用单次迭代，适合控制计算，不适合要求高精度的离线数值计算。
- `wrap_center()` 的 `modulus` 必须为正数。

