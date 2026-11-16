# Gimbal

双轴云台应用，使用 INS 反馈进行角度/角速度双环控制，支持手动角速度输入和视觉目标跟踪，并向底盘发布 Yaw 电机总角度。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    application/gimbal/gimbal.c
    application/gimbal/gimbal_task.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/gimbal
)
```

依赖 DJI [`motor`](../../components/motor/README.md)、PID、Math Tools、[`sdk/message`](../../sdk/message/README.md)、[`application/command`](../command/README.md) 和 CMSIS-RTOS2。

## 硬件映射

| 轴 | CAN | 发送 ID | 反馈 ID |
| --- | --- | --- | --- |
| Yaw | CAN1 | `0x1FE` | `0x205` |
| Pitch | CAN2 | `0x1FE` | `0x205` |

Robot SDK 的 CAN 回调将反馈 ID 路由到 `gimbal.motor_yaw` 和 `gimbal.motor_pitch`，并在 1 ms 调度中发送两路 `0x1FE` 聚合帧。

## 消息接口

| 方向 | Topic | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/ins` | `Ins_Message_t` | 姿态角和角速度反馈 |
| 订阅 | `/vision` | `Vision_Message_t` | 视觉目标与前馈 |
| 订阅 | `/command/gimbal` | `Gimbal_Cmd_Message_t` | 模式和手动角速度 |
| 发布 | `/gimbal` | `Gimbal_Message_t` | Yaw 电机总角度，供底盘跟随 |

控制命令或 INS 超过 `100 ms` 未更新时自动进入 Relax；视觉消息超过 `100 ms` 或 `target_lock != 49` 时回到手动控制。Topic 在 `RobotSdk_Init()` 中统一注册，云台只创建订阅者和发布者。

## 任务

```c
void gimbal_task(void *argument)
{
    (void)Gimbal_Init();
    (void)Command_Init();
    osThreadFlagsSet(defaultTaskHandle, TASK_READY_GIMBAL);

    for (;;)
    {
        Command_Update();
        Gimbal_Update_Input();
        Gimbal_Update_Feedback();
        Gimbal_Handle_Safety();
        Gimbal_Set_Mode();
        Gimbal_Control();
        Gimbal_Calculate();
        Gimbal_Output();
        osDelayUntil(next_tick);
    }
}
```

当前任务周期 `1 ms`、优先级 `osPriorityHigh`、栈 `2048 B`。角度外环每 5 个周期计算一次，即 200 Hz；角速度内环每周期计算。

## 模式

- `GIMBAL_RELAX`：清除 PID 积分并输出零电流。
- `GIMBAL_ACTIVE`：首次进入时先将 Pitch 回中；完成后接收手动或视觉目标。

Pitch 目标限制在 `[-0.55, 0.5] rad`，回中目标为 `0 rad`。这些配置和 PID 参数位于 `gimbal.c`，移植后必须按机械限位和电机方向调整。

## 注意事项

- INS 坐标轴当前使用 `angle[2]/gyro[2]` 作为 Yaw，`angle[1]/gyro[1]` 作为 Pitch。
- 调试前确认机械限位、编码器方向和 IMU 正方向，避免闭环正反馈。
- `Command` 当前随 Gimbal Task 安装；若改成独立任务，应从这里移除初始化和更新。
- INS、视觉、DR16/VT13 等上游模块仍需迁移到 C 消息中心；完整固件在这些模块改完之前不能通过构建。

