# Motor

电机组件包含 DJI 电机和达妙电机驱动。两者都通过 CAN BSP 接收反馈、执行离线检测，并由上层任务或统一定时器调度控制发送。

## 安装

按需加入对应源文件：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/motor/motor_dji.c
    components/motor/motor_dm.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/motor
)
```

依赖：

- [`bsp/can`](../../bsp/can/README.md)
- [`components/algorithm/math_tools`](../algorithm/math_tools/README.md)
- DJI 电机额外依赖 [`components/algorithm/pid`](../algorithm/pid/README.md)

## DJI 电机

### 初始化

```c
MotorDji_t motor;

Pid_Init(&motor.omega_pid, 40.0f, 0.0f, 0.0f, 0.0f,
         0.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f,
         PID_D_FIRST_DISABLE);
MotorDji_Init(&motor, &hcan1, 0x200, 0x201,
              MOTOR_DJI_CONTROL_METHOD_OMEGA,
              3591.0f / 187.0f, 0);
```

参数依次为 CAN、聚合发送 ID、反馈 ID、控制模式、减速比和反向安装标志。支持电流、速度和角度三种控制模式。

### 接收与调度示例

```c
void can1_callback(BspCan_RxBuffer_t *rx)
{
    if (rx->header.StdId == 0x201)
    {
        MotorDji_CanRxCallback(&motor, rx->data);
    }
}

void control_1ms()
{
    MotorDji_SetTargetOmega(&motor, target_omega);
    MotorDji_Calculate(&motor);
    BspCan_Send(&hcan1, 0x200, can1_0x200_tx_data, 8);
}

void alive_100ms()
{
    MotorDji_CheckAlive100ms(&motor);
}
```

`MotorDji_Calculate()` 只填入全局聚合发送缓存，调用者必须在所有同组电机计算完成后统一发送 CAN 帧。角度模式的外环每 5 次计算一次，1 kHz 调度时为 200 Hz。

每个 target setter 都会刷新控制指令计数。若连续 `100 ms` 没有反馈或没有新的 target，电机输出归零并清除 PID 积分。

反馈通过 `motor.rx_data` 读取，包括单圈角度、总角度、速度、电流和温度。

## 达妙电机

### 初始化

```c
MotorDm_t motor;

MotorDm_Init(&motor, &hcan1,
             0x01,                         // CAN_ID
             0x11,                         // Master_ID / feedback ID
             MOTOR_DM_CONTROL_METHOD_MIT,
             12.5f, 30.0f, 10.0f,          // P_MAX, V_MAX, T_MAX
             20.0f, 1.0f,                  // KP, KD
             0,
             MOTOR_DM_ENABLE_STATUS_ENABLE);
```

控制模式与实际发送 ID：

| 模式 | 发送 ID |
| --- | --- |
| `MOTOR_DM_CONTROL_METHOD_MIT` | `CAN_ID` |
| `MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA` | `CAN_ID + 0x100` |
| `MOTOR_DM_CONTROL_METHOD_OMEGA` | `CAN_ID + 0x200` |

`P_MAX/V_MAX/T_MAX` 必须与达妙上位机中的电机参数一致。

### 接收与发送

```c
void can_callback(BspCan_RxBuffer_t *rx)
{
    if (rx->header.StdId == 0x11)
    {
        MotorDm_CanRxCallback(&motor, rx->data);
    }
}

void control_1ms()
{
    MotorDm_SetTargetAngle(&motor, target_angle);
    MotorDm_SetTargetOmega(&motor, target_omega);
    MotorDm_SetTargetTorque(&motor, target_torque);
    MotorDm_SetPidParams(&motor, kp, kd);
    MotorDm_SendControl(&motor);
}

void alive_100ms()
{
    MotorDm_CheckAlive100ms(&motor);
}
```

`MotorDm_SendControl()` 会根据反馈错误状态发送控制帧、使能帧或清错帧。电机未连接、控制指令超过 `100 ms` 未更新，或 `enable_status` 为 Disable 时，发送失能帧。

可用维护命令：`MotorDm_SendClearError()`、`MotorDm_SendEnableCmd()`、`MotorDm_SendDisableCmd()` 和 `MotorDm_SaveZeroPosition()`。保存零点前必须架空机构并确认机械位置。

## 注意事项

- 达妙电机驱动已加入构建，但当前 `RobotSdk` 没有为它分配 CAN 接收 ID 或定时调度。使用时需要在相应 CAN 回调中按反馈 ID 调用 `MotorDm_CanRxCallback()`，并周期调用 `MotorDm_CheckAlive100ms()` 和 `MotorDm_SendControl()`。
- 达妙电机的 `P_MAX/V_MAX/T_MAX` 与控制模式必须与电调配置一致；`MotorDm_SaveZeroPosition()` 会改变电机零点。

- CAN 回调、发送 ID 和电机实例必须一一对应。
- DJI 聚合缓存按反馈 ID 计算写入位置，错误的 ID 组合可能越界或覆盖其他电机命令。
- 达妙 `set_pid_params()` 不刷新控制在线计数，周期中仍需调用至少一个 target setter。
- 反向安装会同时反转目标与反馈方向，调试前确认坐标系约定。

