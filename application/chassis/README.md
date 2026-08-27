# Chassis

四麦克纳姆轮底盘应用，支持失能、底盘独立、跟随云台和小陀螺模式，包含运动学解算、轮速 PID、轮速缩放、有限加速度和可选功率控制。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/chassis/chassis.cpp
    application/chassis/chassis_task.cpp
    application/chassis/power_control/power_controller.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/chassis
    application/chassis/power_control
)
```

依赖 DJI [`motor`](../../components/motor/README.md)、PID、Math Tools、[`sdk/message`](../../sdk/message/README.md)、CMSIS-RTOS2 和 [`power_control`](power_control/README.md)。不需要功控时可移除 PowerController，并让底盘直接使用 `control_output_` 电流。

## 硬件映射

四个轮电机均位于 CAN1，聚合发送 ID 为 `0x200`：

| 电机索引 | 反馈 ID | 反向 |
| ---: | --- | --- |
| 0 | `0x201` | 是 |
| 1 | `0x202` | 否 |
| 2 | `0x203` | 否 |
| 3 | `0x204` | 是 |

减速比为 `3591/187`。Robot SDK 必须路由四个反馈 ID，并在 1 ms 调度中统一发送 CAN1 `0x200`。

## 消息接口

| 方向 | Topic | 消息类型 |
| --- | --- | --- |
| 订阅 | `/command/chassis` | `ChassisCmdMessage` |
| 订阅 | `/gimbal` | `GimbalMessage` |
| 订阅 | `/powercontroller` | `PowerControllerMessage` |
| 发布 | `/chassis` | `ChassisMessage` |

控制命令超过 `100 ms` 未更新时自动清零并进入 Relax。

## 任务调用顺序

```cpp
chassis.update_input();
chassis.update_feedback();
chassis.handle_safety();
chassis.set_mode();
chassis.control();
chassis.solve();
chassis.output();

power_controller.update();
```

任务周期为 `1 ms`。`chassis.output()` 先发布本周期期望电流，PowerController 随后计算并发布限流结果，底盘在下一周期读取并写入电机 target。

## 模式

- `CHASSIS_RELAX`：速度和电流目标清零。
- `CHASSIS_ONLY`：直接使用 `vx/vy/vw`；消息枚举已定义，但当前 `set_mode()` 尚未处理 `CHASSIS_CMD_ONLY`。
- `CHASSIS_FOLLOW`：根据 Gimbal Yaw 偏差计算底盘角速度。
- `CHASSIS_SPIN`：固定角速度小陀螺，并根据云台角度旋转平移指令坐标系。

## 机械参数

`ChassisConfig` 中的轮径、半长、半宽、云台 Yaw 偏置、最大轮速、加减速度和小陀螺速度均与当前机器人绑定。移植时必须重新测量，尤其是：

```text
wheel_radius       = 0.07689 m
half_length        = 0.193560273 m
half_width         = 0.167982291 m
gimbal_yaw_offset  = -0.56 rad
max_wheel_omega    = 46 rad/s
```

## 注意事项

- 四个电机数组顺序必须在运动学、CAN 回调、功控和物理安装之间保持一致。
- 当前平移加速度限制按固定 `1 ms` 周期计算。
- 功控 Topic 在首个有效结果到来前为零，启动时底盘不会直接输出未限制电流。

