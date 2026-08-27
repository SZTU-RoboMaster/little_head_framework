# Shoot

发射机构应用，控制一个拨盘电机和两个摩擦轮电机，支持摩擦轮启停、单发、连发、拨盘堵转反转恢复和裁判系统热量估计。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/shoot/shoot.cpp
    application/shoot/shoot_task.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/shoot
)
```

依赖 DJI [`motor`](../../components/motor/README.md)、[`sdk/message`](../../sdk/message/README.md)、CMSIS-RTOS2，以及运行时的 Command 和 Referee 消息。

## 硬件映射

三个电机均位于 CAN2，聚合发送 ID 为 `0x200`：

| 机构 | 反馈 ID | 控制方式 |
| --- | --- | --- |
| 拨盘 | `0x201` | 角度/速度/电流动态切换，减速比 36 |
| 左摩擦轮 | `0x202` | 速度 |
| 右摩擦轮 | `0x203` | 速度 |

Robot SDK 必须分发对应反馈，并在 1 ms 调度中统一发送 CAN2 `0x200`。

## 消息接口

| 方向 | Topic | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/command/shoot` | `ShootCmdMessage` | 摩擦轮、连发和单发序号 |
| 订阅 | `/referee` | `RefereeMessage` | 枪口热量、热量上限和冷却值 |

控制消息超过 `100 ms` 未更新时关闭摩擦轮并取消发射请求。

## 任务

```cpp
shoot.update_input();
shoot.update_feedback();
shoot.handle_safety();
shoot.set_mode();
shoot.update_control_state();
shoot.control();
shoot.output();
```

当前任务周期为 `1 ms`、优先级 `AboveNormal`。`output()` 只更新电机 target，PID 计算与 CAN 发送由 Robot SDK 完成。

摩擦轮电流变化用于估计发弹并累计热量，裁判系统热量变化时重新校准内部估计。

## 当前限制

- `SHOOT_DOUBLE` 和 `SHOOT_TRIPLE` 状态已定义，但 Command 当前只产生单发和连发。
- 热量阈值字段已经保留，但 `update_heat_state()` 尚未根据阈值实施减速或停火。
- 电流阈值、摩擦轮速度和拨盘方向与当前机构绑定，移植后必须重新标定。
- 首次测试应拆弹并架空机构，先确认摩擦轮方向和拨盘单发角度。

