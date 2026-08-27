# DWT

基于 Cortex-M DWT 周期计数器的微秒阻塞延时模块，主要用于 BMI088 和 IST8310 等器件初始化时的短延时。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/dwt/bsp_dwt.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/dwt
)
```

该模块直接访问 CMSIS Core 的 `CoreDebug` 和 `DWT` 寄存器，不需要配置额外定时器。

## 使用方法

```cpp
#include "bsp_dwt.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    dwt_init();

    delay_us(10);
}
```

`dwt_init()` 开启 trace 并启动 `DWT_CYCCNT`。`delay_us()` 轮询周期计数器直到达到目标时间。

## 时钟配置

当前实现按 `168 MHz` 主频计算：

```cpp
const uint32_t ticks = us * 168;
```

移植到其他主频时必须修改该系数，或改为根据 `SystemCoreClock` 计算。

## 注意事项

- `delay_us()` 是忙等待，会占用 CPU，不适合毫秒级延时。
- 中断抢占会延长实际等待时间。
- 计数差使用无符号减法，可以处理一次 32 位计数器回绕。
- 部分低功耗或调试配置会停止 DWT；当前工程不进入此类低功耗状态。

