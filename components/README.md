# Components Layer

Components 层封装可复用的设备驱动、通信协议和控制算法。组件可以依赖 BSP 和 SDK Message Center，但不应依赖具体 application 的控制流程。

## 模块

| 目录 | 作用 |
| --- | --- |
| [`algorithm`](algorithm/README.md) | PID、卡尔曼滤波和数学工具 |
| [`motor`](motor/README.md) | DJI、达妙电机驱动和控制 |
| [`rc`](rc/README.md) | DR16、VT13 遥控器解析和消息发布 |
| [`referee`](referee/README.md) | RoboMaster 裁判系统协议解析 |
| [`bmi088`](bmi088/README.md) | BMI088 SPI 驱动和姿态 EKF |
| [`vision`](vision/README.md) | USB CDC 视觉协议收发 |
| [`support`](support/README.md) | CRC8/CRC16 通用校验函数 |

## 安装原则

每个目录都可按需加入工程，但必须同时安装 README 中列出的 BSP、SDK 和算法依赖。设备组件通常只负责：

1. 绑定一个 BSP 外设管理对象。
2. 在 BSP 回调中解析反馈。
3. 对上层提供结构化数据，或通过 Message Center 发布消息。
4. 由上层任务或统一调度器周期调用控制、发送和离线检测。

硬件实例、GPIO、CAN ID 和控制参数不是通用配置。复制模块后应先检查 `init()` 参数和接口文件中的硬编码映射。
