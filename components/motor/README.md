# Motor

电机组件包含 DJI 电机和达妙电机驱动。两者都通过 CAN BSP 接收反馈、执行离线检测，并由上层任务或统一定时器调度控制发送。

## 安装

按需加入对应源文件：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/motor/motor_dji.cpp
    components/motor/motor_dm.cpp
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

```cpp
MotorDji motor;

motor.omega_pid_.init(40.0f, 0.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
motor.init(&hcan1, 0x200, 0x201,
           MOTOR_DJI_CONTROL_METHOD_OMEGA,
           3591.0f / 187.0f,
           false);
```

参数依次为 CAN、聚合发送 ID、反馈 ID、控制模式、减速比和反向安装标志。支持电流、速度和角度三种控制模式。

### 接收与调度示例

```cpp
void can1_callback(CanRxBuffer *rx)
{
    if (rx->header.StdId == 0x201)
    {
        motor.can_rx_callback(rx->data);
    }
}

void control_1ms()
{
    motor.set_target_omega(target_omega);
    motor.calculate();
    can_data_send(&hcan1, 0x200, can1_0x200_tx_data, 8);
}

void alive_100ms()
{
    motor.check_alive_100ms();
}
```

`calculate()` 只填入全局聚合发送缓存，调用者必须在所有同组电机计算完成后统一发送 CAN 帧。角度模式的外环每 5 次 `calculate()` 计算一次，1 kHz 调度时为 200 Hz。

每个 target setter 都会刷新控制指令计数。若连续 `100 ms` 没有反馈或没有新的 target，电机输出归零并清除 PID 积分。

反馈通过 `rx_data_` 读取，包括单圈角度、总角度、速度、电流和温度。

## 达妙电机

### 初始化

```cpp
MotorDm motor;

motor.init(&hcan1,
           0x01,                         // CAN_ID
           0x11,                         // Master_ID / feedback ID
           MOTOR_DM_CONTROL_METHOD_MIT,
           12.5f, 30.0f, 10.0f,          // P_MAX, V_MAX, T_MAX
           20.0f, 1.0f,                  // KP, KD
           false,
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

```cpp
void can_callback(CanRxBuffer *rx)
{
    if (rx->header.StdId == 0x11)
    {
        motor.can_rx_callback(rx->data);
    }
}

void control_1ms()
{
    motor.set_target_angle(target_angle);
    motor.set_target_omega(target_omega);
    motor.set_target_torque(target_torque);
    motor.set_pid_params(kp, kd);
    motor.send_control();
}

void alive_100ms()
{
    motor.check_alive_100ms();
}
```

`send_control()` 会根据反馈错误状态发送控制帧、使能帧或清错帧。电机未连接、控制指令超过 `100 ms` 未更新，或 `enable_status` 为 Disable 时，发送失能帧。

可用维护命令：`send_clear_error()`、`send_enable_cmd()`、`send_disable_cmd()` 和 `save_zero_position()`。保存零点前必须架空机构并确认机械位置。

## 注意事项

- CAN 回调、发送 ID 和电机实例必须一一对应。
- DJI 聚合缓存按反馈 ID 计算写入位置，错误的 ID 组合可能越界或覆盖其他电机命令。
- 达妙 `set_pid_params()` 不刷新控制在线计数，周期中仍需调用至少一个 target setter。
- 反向安装会同时反转目标与反馈方向，调试前确认坐标系约定。

