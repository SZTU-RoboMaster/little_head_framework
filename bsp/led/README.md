# RGB LED BSP

使用 TIM1 三路 PWM 控制板载 RGB LED，输入格式为 `0xAARRGGBB`。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/led/bsp_led.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/led
)
```

## CubeMX 配置

当前硬件映射：

| 颜色 | 定时器通道 | GPIO |
| --- | --- | --- |
| Blue | TIM1 CH4 | PC3 |
| Green | TIM1 CH2 | PC1 |
| Red | TIM1 CH3 | PC2 |

TIM1 三个通道配置为 PWM Generation，当前 Period 为 `42499`。

## 使用方法

```c
#include "bsp_led.h"

BspLed_Init();
BspLed_ShowArgb(0xFFFF0000); // 不透明红色
BspLed_ShowArgb(0xFF00FF00); // 不透明绿色
BspLed_ShowArgb(0x800000FF); // 半亮蓝色
```

`BspLed_Init()` 启动三个 PWM 通道。Alpha 和每个颜色分量均为 `0-255`，实际比较值为 `color * alpha`。

## 注意事项

- 更换定时器、通道或 GPIO 后，需要同步修改 `bsp_led.c`。
- 该实现假设 PWM 比较值越大 LED 越亮；使用反相驱动时需要调整输出逻辑。

