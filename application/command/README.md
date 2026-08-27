# Command

控制输入仲裁与语义转换模块。它订阅 DR16 和 VT13 原始数据，判断在线状态，并发布云台、底盘和发射机构可直接使用的控制消息。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/command/command.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/command
)
```

依赖 [`components/rc`](../../components/rc/README.md)、[`components/algorithm/math_tools`](../../components/algorithm/math_tools/README.md) 和 [`sdk/message`](../../sdk/message/README.md)。

## 使用方法

```cpp
Command command;

void control_task(void *argument)
{
    command.init();

    for (;;)
    {
        command.update();
        osDelay(1);
    }
}
```

当前工程将 `Command` 唯一实例放在 `gimbal_task` 中，并以 1 kHz 更新。由于三个输出 Topic 都只允许一个发布者，不要在其他任务重复创建 `Command`。

## 消息接口

| 方向 | Topic | 消息类型 |
| --- | --- | --- |
| 订阅 | `/dr16` | `Dr16Message` |
| 订阅 | `/vt13` | `Vt13Message` |
| 发布 | `/command/gimbal` | `GimbalCmdMessage` |
| 发布 | `/command/chassis` | `ChassisCmdMessage` |
| 发布 | `/command/shoot` | `ShootCmdMessage` |

两种遥控器均超过 `100 ms` 没有新消息时，模块发布全零/Relax 安全指令。

## 当前 DR16 映射

| 输入 | 输出语义 |
| --- | --- |
| `sw_2` 下 | 云台、底盘失能，摩擦轮关闭 |
| `sw_2` 中 | 云台使能，底盘跟随云台 |
| `sw_2` 上 | 云台使能，底盘小陀螺 |
| `sw_1` 中 -> 上 | 切换摩擦轮开关 |
| `sw_1` 下 | 摩擦轮开启时连发 |
| 拨轮越过 `300` 上升沿 | 单发序号加一 |
| `ch_2/ch_3` | 云台 Yaw/Pitch 角速度 |
| `ch_1/ch_0` | 底盘 `vx/vy` |

摇杆使用 `cubic_map(..., 0.5)` 做三次曲线映射。

## 当前限制

DR16 在线时优先使用 DR16。VT13 在线但 DR16 离线时的语义映射分支当前为空，因此只会发布默认安全消息；接入 VT13 控制前需要补齐该分支。

