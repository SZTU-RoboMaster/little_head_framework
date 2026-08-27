# Power Control

底盘功率控制模块。模块根据底盘四个电机的目标电流、目标转速和反馈转速预测功率，并在预测功率超过限制时重新分配功率，最终输出四个电机允许使用的目标电流。

该模块通过 Message Center 与底盘和裁判系统通信，不直接访问硬件，无需额外配置 CubeMX 外设。

## 安装

将 `application/chassis/power_control` 目录加入工程，并在工程的 `CMakeLists.txt` 中添加：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/chassis/power_control/power_controller.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/chassis/power_control
)
```

## 依赖

编译依赖：

- `sdk/message`：提供 Message Center、消息类型和 Topic 定义。
- `components/algorithm/math_tools`：提供数学工具及相关头文件依赖。

需要保证下列依赖也已加入 CMake：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sdk/message/message_center.cpp
    components/algorithm/math_tools/math_tools.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    sdk/message
    components/algorithm/math_tools
)
```

运行时还需要其他模块发布底盘和裁判系统消息。裁判系统消息可以暂时缺失，此时功率上限回退到 `50 W`；底盘消息不可缺失，否则本模块不会计算和发布输出。

## 消息接口

| 方向 | Topic | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/chassis` | `ChassisMessage` | 获取四个电机的目标电流、目标转速和反馈转速 |
| 订阅 | `/referee` | `RefereeMessage` | 获取底盘功率限制和缓冲能量 |
| 发布 | `/powercontroller` | `PowerControllerMessage` | 发布功率限制后的四路目标电流 |

消息类型和 Topic 名称定义在 `sdk/message/message_def.h`。安装到其他工程时，需要同时保留以下定义：

- `kChassisTopicName`、`ChassisMessage`
- `kRefereeTopicName`、`RefereeMessage`
- `kPowerControllerTopicName`、`PowerControllerMessage`

## 使用方法

### 初始化

```cpp
#include "power_controller.h"

PowerController power_controller;

void chassis_init()
{
    power_controller.init(POWER_CONTROL_ENABLE);
}
```

`init()` 支持两种模式：

| 模式 | 行为 |
| --- | --- |
| `POWER_CONTROL_ENABLE` | 功率超限时重新分配功率并限制电机电流 |
| `POWER_CONTROL_DISABLE` | 不进行功率限制，直接转发底盘请求的电流 |

### 周期调用

在底盘发布 `/chassis` 后调用 `update()`：

```cpp
void chassis_control_loop()
{
    chassis.update_input();
    chassis.update_feedback();
    chassis.handle_safety();
    chassis.set_mode();
    chassis.control();
    chassis.solve();
    chassis.output();       // 发布 /chassis

    power_controller.update(); // 计算并发布 /powercontroller
}
```

推荐与底盘控制任务使用相同周期。当前工程在 `chassis_task` 中以 `1 kHz` 调用，底盘会在下一控制周期读取本次发布的限流结果。

`update()` 只有收到新的 `/chassis` 消息时才会计算并发布，因此无需在外部额外判断消息是否更新。

## 工作流程

1. 读取裁判系统的底盘功率限制和缓冲能量。
2. 根据四个电机的目标电流和反馈转速预测总功率。
3. 未超过功率上限时，直接输出原目标电流。
4. 超过功率上限时，根据轮速误差和各电机请求功率分配可用功率。
5. 通过电机功率模型逆解每个电机允许使用的目标电流。
6. 将结果发布到 `/powercontroller`，由底盘模块订阅并发送给电机。

裁判系统超过 `1000 ms` 没有新消息时，模块将最大功率设为 `50 W`：

```cpp
max_power = 50.0f;
```

裁判系统在线时使用：

```text
max_power = chassis_power_limit + 0.2 * (buffer_energy - 50)
```

## 参数标定

电机功率模型参数位于 `power_controller.h`：

```cpp
static constexpr float k1_ = 3.24956187e-4f;
static constexpr float k2_ = 2.96678362e-1f;
static constexpr float k3_ = 1.77641220e-7f;
static constexpr float a_  = 9.50608963e-1f;
```

预测模型为：

```text
power = k1 * current * omega
      + k2 * abs(omega)
      + k3 * current^2
      + a
```

这些参数与电机型号、减速箱、底盘机械结构和采样数据有关。模块移植到其他机器人后，应重新采集数据并标定，不能直接把当前参数视为通用参数。

功率超限时，模块会在“轮速误差权重”和“请求功率权重”之间插值。插值阈值由以下参数控制：

```cpp
static constexpr float omega_err_lower = 8.0f;
static constexpr float omega_err_upper = 16.0f;
```

## 注意事项

- `ChassisMessage` 中的数组顺序必须与底盘四个电机的顺序保持一致。
- `cmd_current` 使用 DJI 电机控制指令的数值尺度，不是以安培为单位的实际电流。
- `feedback_omega` 和 `cmd_omega` 的单位必须一致。
- 功控关闭只会旁路限流算法，不会使能或失能底盘电机。
- 初次运行应架空底盘并监视裁判系统功率、预测功率和输出电流，确认模型与电机方向正确后再落地测试。
