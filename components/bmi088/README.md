# BMI088

BMI088 六轴 IMU 驱动及姿态估计算法。传感器通过 SPI1 读取，加速度计和陀螺仪 Data Ready 引脚触发采样；同目录还提供重力向量 Kalman Filter 和四元数 EKF。

## 安装

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    components/bmi088/third_party/bmi08a.c
    components/bmi088/third_party/bmi08g.c
    components/bmi088/third_party/bmi08xa.c
    components/bmi088/bmi088_interface.cpp
    components/bmi088/bmi088.cpp
    components/bmi088/quaternion_ekf.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    components/bmi088
    components/bmi088/third_party
)
```

依赖：

- [`bsp/dwt`](../../bsp/dwt/README.md)
- [`components/algorithm/math_tools`](../algorithm/math_tools/README.md)
- `components/algorithm/kf`
- CMSIS-DSP MatrixFunctions
- CMSIS-RTOS2，初始化接口中使用 `osDelay(10)`

## CubeMX 配置

当前接口文件硬编码为 SPI1：

| 信号 | 当前配置 |
| --- | --- |
| SPI1 | Master、Full Duplex、Mode 3、约 5.25 Mbit/s |
| Accel CS | `CS1_ACCEL`，PA4，默认高 |
| Gyro CS | `CS1_GYRO`，PB0，默认高 |
| Accel DRDY | `INT1_ACCEL`，PC4，下降沿 EXTI |
| Gyro DRDY | `INT1_GYRO`，PC5，下降沿 EXTI |

GPIO 标签必须生成 `CS1_ACCEL_Pin`、`CS1_GYRO_Pin`、`INT1_ACCEL_Pin` 和 `INT1_GYRO_Pin` 等宏。更换 SPI 或引脚时修改 `bmi088_interface.cpp`。

## 初始化与采样

`Bmi088::init()` 内部使用 RTOS 延时，应在调度器启动后的任务中调用：

```cpp
Bmi088 bmi088;

void imu_task(void *argument)
{
    int8_t result = bmi088.init();
    if (result != BMI08_OK)
    {
        // 初始化失败处理
    }

    for (;;)
    {
        osThreadFlagsWait(INS_DATA_READY_FLAG, osFlagsWaitAny, osWaitForever);
        // 使用 bmi088.rx_data_
    }
}
```

在 HAL EXTI 回调中转发引脚：

```cpp
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == INT1_ACCEL_Pin || gpio_pin == INT1_GYRO_Pin)
    {
        bmi088.exti_read_callback(gpio_pin);
    }
}
```

`rx_data_.accel` 单位为 `m/s^2`，`rx_data_.gyro` 单位为 `rad/s`，`rx_data_.temp` 单位为摄氏度。

## 当前传感器配置

- Accelerometer：1600 Hz ODR、6 g range、Normal bandwidth、Active mode。
- Gyroscope：1000 Hz ODR、2000 dps range、116 Hz bandwidth、Normal mode。
- Data Ready：推挽、低有效。

## 姿态算法

```cpp
GravityKf gravity_kf;
QuaternionEkf attitude;

gravity_kf.init(1.0f, 2000.0f);
attitude.init(10.0f, 0.001f, 1000000.0f, 0.9996f);

gravity_kf.update(gx, gy, gz, ax, ay, az, 0.001f);
attitude.update(gx, gy, gz,
                gravity_kf.gravity_vec_[0],
                gravity_kf.gravity_vec_[1],
                gravity_kf.gravity_vec_[2],
                0.001f);
```

姿态结果位于 `attitude.ins_.q`、`attitude.ins_.angle` 和 `attitude.ins_.gyro_bias`。

## 注意事项

- `dwt_init()` 必须先于 BMI088 初始化执行，且当前 DWT 延时按 168 MHz 计算。
- SPI 读写为轮询阻塞方式，当前 EXTI 优先级和执行时间必须纳入实时性评估。
- 当前驱动直接使用传感器轴向，没有安装方向旋转矩阵；机械安装改变时应在上层统一坐标系。
- 四元数 EKF 底层 KF 会在初始化时分配内存，只初始化一次。
- 初始陀螺仪 Z 轴零偏会被统计，reset后静置20秒用minitor读取gyro_bias_z_ 的值填入ekf的ins_.gyro_bias[2]中，注意符号。
