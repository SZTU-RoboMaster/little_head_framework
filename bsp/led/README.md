# RGB LED BSP

使用 TIM5 三路 PWM 控制板载 RGB LED，输入格式为 `0xAARRGGBB`。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/led/bsp_led.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/led
)
```

## CubeMX 配置

当前硬件映射：

| 颜色 | 定时器通道 | GPIO |
| --- | --- | --- |
| Blue | TIM5 CH1 | PH10 |
| Green | TIM5 CH2 | PH11 |
| Red | TIM5 CH3 | PH12 |

TIM5 三个通道配置为 PWM Generation，当前 Period 为 `65535`。

## 使用方法

```cpp
#include "bsp_led.h"

led_init();
aRGB_led_show(0xFFFF0000); // 不透明红色
aRGB_led_show(0xFF00FF00); // 不透明绿色
aRGB_led_show(0x800000FF); // 半亮蓝色
```

`led_init()` 启动三个 PWM 通道。Alpha 和每个颜色分量均为 `0-255`，实际比较值为 `color * alpha`。

## 注意事项

- 更换定时器、通道或 GPIO 后，需要同步修改 `bsp_led.cpp`。
- 该实现假设 PWM 比较值越大 LED 越亮；使用反相驱动时需要调整输出逻辑。

