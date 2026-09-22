# Robot SDK, Message Bus, and RTOS Refactor Design

Date: 2026-07-11

Status: Approved

## 1. Context

The current firmware runs control logic through `application/initial/initial_task.cpp`.
That file owns application and device objects, routes BSP callbacks directly to object
internals, runs INS/Gimbal/Chassis logic in one busy loop, and runs motor PID and CAN
transmission from TIM7. `main.cpp` currently has FreeRTOS startup disabled.

This refactor separates application logic from device and BSP code, restores RTOS task
scheduling, introduces a small ROS-like latest-value message bus, and centralizes motor
ownership in a deterministic 1 kHz MotorManager.

## 2. Goals

- Run `ins_task`, `gimbal_task`, `chassis_task`, and `shoot_task` at 1 kHz.
- Keep task creation in the CubeMX-generated `freertos.c` USER CODE regions.
- Keep weak task definitions in `freertos.c` and strong C-linkage definitions in each
  application module's `.cpp` file.
- Split the current `initial_task` responsibilities between SDK startup/services and
  application modules.
- Prevent application code from accessing BSP handles or device objects directly.
- Add Node, Publisher, and Subscriber APIs with ROS-style topic paths.
- Use latest-value semantics, automatic millisecond publication timestamps, no
  callbacks, and no dynamic allocation in the message bus.
- Let a single MotorManager own all motors, run motor inner loops, route CAN feedback, and
  send CAN commands from TIM7.
- Add a minimal CommandManager that currently maps DR16 input to semantic robot commands
  and can later arbitrate DR16, VT13, and video-link input.
- Preserve current behavior while creating clean boundaries for later expansion.

## 3. Non-goals

- Runtime creation or deletion of topics.
- Message history, FIFO subscriptions, callbacks, topic links, filters, or remote topic
  serialization.
- Multiple publishers for one topic.
- Implementing VT13/video-link arbitration in this iteration.
- Sending an explicit DM motor disable frame in this iteration.
- Adding a TIM2 timebase. Millisecond-resolution publication timestamps are sufficient for
  freshness and offline detection; DWT remains the microsecond delay and diagnostic timer.
- Reworking unrelated controller algorithms or tuning values.

## 4. Layering and Ownership

```text
Core
  main.cpp
  freertos.c                 CubeMX and CMSIS-RTOS2 glue

application
  application.cpp            application_init()
  ins/
  command/
  gimbal/
  chassis/
  shoot/

sdk
  message/                   MessageCenter, Node, handles, contracts
  motor/                     MotorManager and static motor configuration
  device/                    ImuService, RemoteService, VisionService
  robot_sdk.cpp              initialization, startup, BSP callback routing

components
  motor/
  bmi088/
  rc/
  vision/
  algorithm/

bsp
  can/
  uart/
  usb/
  tim/
  dwt/                       existing microsecond delay and diagnostic timing
  interrupt/                 interrupt save/restore critical section
```

The allowed dependency direction is:

```text
application -> sdk/message
application -> components/algorithm
sdk         -> components/device -> bsp
Core        -> SDK and application initialization entry points
sdk         -X-> application
```

SDK owns the MessageCenter, MotorManager, all motor instances, Bmi088, Dr16, Vision,
hardware configuration, and BSP callback routing. Application owns Ins, CommandManager,
Gimbal, Chassis, Shoot, outer-loop controllers, state machines, kinematics, and fusion
algorithms.

Application must not include CAN/UART/USB handles or directly access Dr16, Bmi088, Vision,
MotorDji, or MotorDm instances.

`application/initial` is removed after migration. It is not renamed wholesale because that
would make SDK depend on application objects.

## 5. Startup and Shutdown-free Lifecycle

Startup order is:

```text
HAL and CubeMX peripheral initialization
  -> sdk_init()
  -> application_init()
  -> MX_FREERTOS_Init()
  -> sdk_start()
       -> MessageCenter::finalize()
       -> enable runtime callbacks and start TIM7
  -> osKernelStart()
```

All nodes, publishers, and subscribers are registered before `finalize()`. After finalization,
new registrations are rejected. The firmware has no runtime topic/device/task teardown path.

The existing cross-layer `init_finished` global is removed. BSP callbacks invoke only a
registered callback, and runtime services are enabled only by `sdk_start()` after the object
graph is complete.

## 6. CMSIS-RTOS2 Tasks

`Core/Src/freertos.c` remains a C file. It contains `osThreadId_t` task handles,
`osThreadAttr_t` definitions, `osThreadNew()` calls, and weak task bodies inside CubeMX
USER CODE regions.
The generated `test` task and its handle are removed so only the four approved application
tasks consume FreeRTOS heap.

Example weak definition:

```c
__weak void chassis_task(void *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(1U);
    }
}
```

Each application `.cpp` file provides the corresponding strong C-linkage definition:

```cpp
extern "C" void chassis_task(void *argument)
{
    (void)argument;
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        next_tick += 1U;
        chassis.update();
        osDelayUntil(next_tick);
    }
}
```

`main.cpp` declares the C entry point with `extern "C" void MX_FREERTOS_Init(void);`.

Task configuration is:

| Task | CMSIS-RTOS2 priority | FreeRTOS priority | `stack_size` bytes |
|---|---:|---:|---:|
| `ins_task` | `osPriorityRealtime` | 48 | 4096 |
| `gimbal_task` | `osPriorityHigh` | 40 | 2048 |
| `chassis_task` | `osPriorityAboveNormal` | 32 | 2048 |
| `shoot_task` | `osPriorityAboveNormal` | 32 | 2048 |

The CMSIS-RTOS2 adapter passes the numeric CMSIS priority directly to FreeRTOS. It interprets
`osThreadAttr_t::stack_size` as bytes and divides it by `sizeof(StackType_t)` before calling
`xTaskCreate()`. Because no `stack_mem` or `cb_mem` is supplied, the four task stacks and
control blocks are dynamically allocated from the existing FreeRTOS heap; middleware and SDK
storage remain statically allocated.

FreeRTOS configuration changes:

- `INCLUDE_vTaskDelayUntil = 1`
- `INCLUDE_uxTaskGetStackHighWaterMark = 1`
- `configCHECK_FOR_STACK_OVERFLOW = 2`
- `configUSE_MALLOC_FAILED_HOOK = 1`

The firmware implements `vApplicationStackOverflowHook()` and
`vApplicationMallocFailedHook()`. Every task handle is checked after creation.

## 7. Task Workflows

### 7.1 INS

```text
update /imu/raw
  -> if new, run GravityKf and QuaternionEkf
  -> publish /ins/state
  -> delay until next 1 ms release
```

Bmi088 remains a device component owned by SDK. GravityKf and QuaternionEkf move out of
Bmi088 and into the INS application.

### 7.2 Gimbal

```text
CommandManager::update()
  -> update command, INS, vision, and motor-feedback subscriptions
  -> input/safety/mode/control/output pipeline
  -> publish /motor/gimbal/target
  -> publish /gimbal/state
  -> delay until next 1 ms release
```

Running CommandManager first lets lower-priority Chassis and Shoot tasks consume commands
from the same RTOS tick. CommandManager can later move to a dedicated task without changing
the command topics.

### 7.3 Chassis

```text
update chassis command, gimbal state, and motor feedback
  -> input/safety/mode/control/kinematics/output pipeline
  -> publish /motor/chassis/target
  -> publish /chassis/state
  -> delay until next 1 ms release
```

### 7.4 Shoot

```text
update shoot command, vision command, and motor feedback
  -> input/safety/mode/control/output pipeline
  -> publish /motor/shoot/target
  -> publish /shoot/state
  -> delay until next 1 ms release
```

## 8. MessageCenter Design

### 8.1 Message Clock Contract

The public middleware clock is `uint32_t` milliseconds. MessageCenter calls `HAL_GetTick()`
directly; no separate BSP time wrapper or injected clock abstraction is added.

Publication timestamps are used only for freshness and offline detection; they are not unique
message identifiers and do not order multiple publications within one millisecond. Sequence
numbers provide update detection.

Unsigned subtraction provides wrap-safe timeout arithmetic across the approximately 49.7-day
`uint32_t` millisecond wrap. The existing DWT implementation remains responsible for BMI088
microsecond delay and execution-time diagnostics. TIM2 is not configured. If a future sensor
requires a precise acquisition timestamp, that timestamp is added to the sensor payload and
does not change MessageCenter metadata.

### 8.2 Public API

```cpp
MessageCenter message_center;
Node gimbal_node{message_center, "gimbal"};

auto state_pub =
    gimbal_node.advertise<GimbalState>("/gimbal/state");

auto ins_sub =
    gimbal_node.subscribe<InsState>("/ins/state");
```

The handles are small value objects:

```text
Publisher<T>
  TopicSlot* topic

Subscriber<T>
  TopicSlot* topic
  uint32_t last_sequence
  uint32_t last_timestamp_ms
  bool received
```

`Node` contains only its name and delegates registration to MessageCenter. Node names are
used for startup diagnostics.

### 8.3 Static Storage

Initial compile-time limits are:

- topic slots: 24
- topic-name storage: 32 bytes including the null terminator
- maximum payload size: 256 bytes
- aligned message pool: 4096 bytes

MessageCenter owns:

```text
TopicSlot topic_slots[24]
alignas(max_align_t) byte message_pool[4096]
size_t message_pool_offset
bool finalized
```

Each TopicSlot stores:

- path
- message type token
- payload size and alignment
- payload address in the static pool
- publication timestamp
- sequence number
- published flag
- publisher-registered flag
- publisher node name

Publisher and Subscriber handles do not allocate from the message pool. Registration uses
an aligned bump allocator and fails deterministically when the configured capacity is
exceeded.

### 8.4 Type and Publisher Rules

- Message payloads must be trivially copyable.
- Payloads larger than 256 bytes are rejected at compile time or registration.
- A type token is generated per C++ type without RTTI.
- A path may have exactly one publisher and any number of Subscriber handles.
- Subscriber-before-publisher registration is allowed.
- Same path with a different type, size, or alignment is an initialization error.
- Topic paths are centralized as constants to prevent spelling drift.
- Every registration result is checked by `sdk_init()` or `application_init()`.
- `finalize()` rejects missing publishers, duplicate publishers, and exhausted storage.

### 8.5 Latest-value Semantics

`publish(data)` performs:

```text
timestamp = HAL_GetTick()
save interrupt state and enter a short global critical section
  -> increment sequence
  -> store timestamp
  -> copy payload into the topic slot
  -> mark topic published
restore interrupt state
```

`update(output)` performs:

```text
save interrupt state and enter a short global critical section
  -> return false if never published or sequence is unchanged
  -> copy payload, sequence, and timestamp
restore interrupt state
cache sequence and timestamp in Subscriber
return true
```

`is_fresh(timeout_ms)` returns false before the first successful update. Afterwards it uses
wrap-safe unsigned subtraction:

```cpp
HAL_GetTick() - last_timestamp_ms <= timeout_ms
```

There is no age-returning API. Timestamp and sequence are middleware metadata, not fields
duplicated in every payload. If a future sensor needs an acquisition timestamp, that value
is added to that sensor's payload separately.

### 8.6 Concurrency

MessageCenter does not use a FreeRTOS mutex. BSP provides interrupt state save/restore so
the same short critical section works before scheduler startup, in tasks, and in ISRs.

No string search, clock read, logging, callback, filtering, queue work, or device access is
allowed inside the critical section. The 256-byte payload limit bounds interrupt latency.

### 8.7 Explicitly Unsupported Features

- callback subscriptions
- FIFO or historical messages
- runtime registration/deletion
- multiple publishers
- links and filters
- cross-device topic serialization
- publisher-side blocking

## 9. Topics and Contracts

All payload structs use natural alignment. Packed USB/CAN wire structs remain private to the
corresponding device service and are converted to/from message contracts.

| Topic | Publisher | Subscribers | Publication event |
|---|---|---|---|
| `/imu/raw` | ImuService | INS | valid gyro data-ready sample |
| `/ins/state` | INS | Gimbal, VisionService | successful fusion update |
| `/input/dr16` | RemoteService | CommandManager | valid DR16 frame |
| `/vision/command` | VisionService | Gimbal, Shoot | valid CRC-checked vision frame |
| `/command/gimbal` | CommandManager | Gimbal | every CommandManager update |
| `/command/chassis` | CommandManager | Chassis | every CommandManager update |
| `/command/shoot` | CommandManager | Shoot | every CommandManager update |
| `/gimbal/state` | Gimbal | Chassis | every Gimbal cycle |
| `/chassis/state` | Chassis | none required initially | every Chassis cycle |
| `/shoot/state` | Shoot | VisionService | every Shoot cycle |
| `/motor/gimbal/target` | Gimbal | MotorManager | every Gimbal cycle |
| `/motor/gimbal/feedback` | MotorManager | Gimbal | every TIM7 cycle |
| `/motor/chassis/target` | Chassis | MotorManager | every Chassis cycle |
| `/motor/chassis/feedback` | MotorManager | Chassis | every TIM7 cycle |
| `/motor/shoot/target` | Shoot | MotorManager | every Shoot cycle |
| `/motor/shoot/feedback` | MotorManager | Shoot | every TIM7 cycle |

Initial contract contents are:

```text
ImuRaw
  accel[3], gyro[3], temperature

InsState
  euler_angle[3], quaternion[4], angular_velocity[3]

Dr16State
  ch[4], sw[2], mouse x/y/z/buttons, key_code, wheel

VisionCommand
  yaw/pitch angle, velocity, acceleration
  target_locked, fire_command

ChassisCommand
  mode: Disabled, FollowGimbal, Spin
  vx, vy, vw

GimbalCommand
  mode: Disabled, Active, LowerHead, extensible modes
  yaw_rate, pitch_rate

ShootCommand
  mode: Disabled, Idle, Continuous
  monotonically increasing single-shot request counter

GimbalState
  yaw/pitch relative mechanical angle
  yaw/pitch angular velocity
  mode

ChassisState
  velocity_x, velocity_y, omega, mode

ShootState
  trigger state, friction-wheel speed, shoot speed, bullet count, mode
```

Single-shot requests use a request counter rather than a one-cycle boolean so latest-value
delivery cannot silently lose a shot request.

## 10. CommandManager

CommandManager belongs to application because it performs robot-specific arbitration and
semantic mapping. SDK services publish raw input-device state only.

The first implementation subscribes only to `/input/dr16` and maps DR16 channels, switches,
mouse, keyboard, and wheel data to the three semantic command topics. If DR16 has not been
fresh for 100 ms, it continues publishing fresh commands with all three command modes set
to Disabled.

Future `/input/vt13` and `/input/video` publishers and arbitration policy are added only to
CommandManager. Gimbal, Chassis, and Shoot remain unchanged.

## 11. Motor Contracts

Motor messages are hardware-type explicit.

```text
DjiMotorTarget
  control_mode
  angle
  omega
  current
  feedforward_omega

DmMotorTarget
  angle
  omega
  torque
  kp
  kd
```

Current grouped targets are:

```text
GimbalMotorTarget
  DjiMotorTarget yaw, pitch

ChassisMotorTarget
  DjiMotorTarget wheel[4]

ShootMotorTarget
  DjiMotorTarget trigger, friction_left, friction_right
```

Corresponding feedback contains online state, fault state where supported, wrapped angle,
total angle, angular velocity, current or torque, and temperatures. If a future mechanism
uses a DM motor, the corresponding grouped field changes to DmMotorTarget/DmMotorFeedback.

## 12. MotorManager

MotorManager owns separate fixed arrays of MotorDji and MotorDm rather than a polymorphic
base hierarchy. Static configuration tables define:

- logical MotorId
- motor type and array index
- CAN bus and feedback ID
- CAN command frame and byte offset
- gearbox ratio and installation direction
- DJI inner-loop PID parameters
- DM p/v/t limits and default gains

A static CAN route table maps `(bus, receive_id)` to the owning motor object.

CAN RX ISR flow:

```text
MotorManager::on_can_rx(bus, id, data)
  -> route lookup
  -> MotorDji/MotorDm feedback parser
```

TIM7 runs every 1 ms:

```text
MotorManager::update()
  1. update the three grouped target subscribers
  2. evaluate 100 ms target freshness
  3. replace stale targets with safe targets
  4. perform motor online checks every 100 cycles
  5. apply modes and targets
  6. run DJI inner loops and compose grouped CAN frames
  7. compose/send DM frames
  8. send each required grouped CAN frame once
  9. publish three grouped feedback topics

VisionService::update()
  1. update INS and Shoot state
  2. check USB transmit availability
  3. serialize latest data into a stable static buffer
  4. compute CRC and start non-blocking transmit
```

Safe motor targets are:

```text
DJI: Current mode, current = 0
DM: torque = 0, kp = 0, kd = 0
```

This iteration does not add an explicit DM enabled field or send a user-requested disable
frame. DM KP/KD remain application-controlled runtime fields. If hardware feedback reports
that a DM motor is not enabled, the driver may still send its enable frame to recover.

Motor feedback is published every TIM7 cycle. The topic timestamp indicates MotorManager
liveness, while each feedback element's `online` and fault fields indicate motor state.

CAN mailbox busy and USB busy are non-blocking. The current cycle is dropped and the next
cycle sends the newest state.

## 13. Device Services

### 13.1 ImuService

SDK owns Bmi088 and routes EXTI callbacks to it. A valid gyro data-ready event publishes
the latest gyro and accelerometer values to `/imu/raw`. Bmi088 contains device access and
unit conversion only.

### 13.2 RemoteService

SDK owns one Dr16 object. UART3/DMA callbacks parse frames and publish `/input/dr16` only
after successful frame validation. Gimbal, Chassis, and Shoot never hold a Dr16 pointer.

### 13.3 VisionService

SDK owns Vision and routes USB receive data to it. Valid frames publish
`/vision/command`. TIM7 drives outgoing vision data. VisionService reads `/ins/state` for
orientation/angular velocity/quaternion and `/shoot/state` for shoot information.

Before modifying its static transmit buffer, VisionService confirms USB is idle. If
`CDC_Transmit_FS()` reports busy, the current cycle is skipped without blocking or queuing.

## 14. Interrupt Priorities

The project uses `NVIC_PRIORITYGROUP_4`; all configured subpriorities remain zero. Lower
numeric NVIC values have higher urgency.

| NVIC priority | Interrupts |
|---:|---|
| 4 | EXTI4 BMI088 accel, EXTI9_5 BMI088 gyro, SPI1 DMA2 Stream0/5 |
| 5 | CAN1 RX0/RX1, CAN2 RX0/RX1, TIM7, OTG_FS |
| 6 | USART3, USART3 RX DMA1 Stream1 |
| 7 | USART1, USART6, their RX/TX DMA2 Stream1/2/6/7 |
| 8 | EXTI3 IST8310 |
| 10 | EXTI0 key |
| 15 | SysTick and PendSV |

FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` remains 5. ISRs at priorities 0-4
must not call FreeRTOS/CMSIS FromISR APIs. BMI088 and SPI DMA at priority 4 call only device
code and the non-RTOS MessageCenter critical section.

CAN RX, TIM7, and OTG_FS deliberately share priority 5 and cannot preempt each other. This
prevents CAN feedback parsing from modifying a motor while TIM7 is calculating it, and
prevents USB ISR re-entry while TIM7 starts a CDC transfer.

## 15. Freshness and Safe-state Policy

| Data | Timeout |
|---|---:|
| `/input/dr16` | 100 ms |
| `/command/*` | 100 ms |
| `/imu/raw` | 10 ms |
| `/ins/state` | 10 ms |
| `/vision/command` | 100 ms |
| `/gimbal/state` | 10 ms |
| `/motor/*/target` | 100 ms |
| `/motor/*/feedback` | 100 ms |

Middleware determines freshness only. Application defines its own safe behavior for stale
command, INS, vision, gimbal, and feedback data. MotorManager is the final low-level safety
owner for stale motor targets.

## 16. Error Handling

The following startup errors stop startup and enter the common Error_Handler after recording
a fixed error code:

- MessageCenter topic or byte pool exhaustion
- topic name/type/size/alignment conflict
- duplicate or missing publisher
- CMSIS task creation failure
- duplicate CAN receive route
- invalid motor command-frame placement

Device unavailability does not halt startup:

- BMI088 failure produces no IMU state; dependent applications enter safe mode by timeout.
- DR16 loss causes CommandManager to publish disabled semantic commands.
- motor loss sets feedback `online = false` and forces safe output.
- USB unavailable/busy skips vision transmission.
- CAN mailbox busy drops the current command frame and retries only with latest state next
  cycle.

High-frequency ISRs never print logs. Fixed diagnostic counters record CAN failures, USB
busy/failures, task overruns, TIM7 current/max execution time, stack high-water marks, and
minimum FreeRTOS heap.

## 17. Verification

### 17.1 Cross-build Checks

- complete ARM CMake build
- C weak task symbols are overridden by C++ `extern "C"` strong symbols
- no accidental `freertos.cpp` or duplicate task symbols
- final RAM, Flash, and FreeRTOS heap budget inspection
- no application dependency on BSP/device headers

### 17.2 On-target Checks

- all four tasks maintain 1 kHz release periods
- observed task order is INS, Gimbal/CommandManager, then Chassis/Shoot
- TIM7 worst-case execution remains below 500 us
- DR16 removal produces disabled semantic commands within 100 ms
- stopping an application target publisher produces zero motor output within 100 ms
- individual motor removal changes its online field and output safely
- USB busy never blocks TIM7
- stack high-water marks leave safe margin before reducing initial stack sizes
- interrupt priority registers match the approved table

## 18. Migration Order

Implementation is staged so the firmware remains buildable at checkpoints:

1. MessageCenter and its interrupt-safe latest-value storage.
2. Message contracts, nodes, SDK/application startup, and CMSIS task restoration.
3. ImuService/INS split and RemoteService/CommandManager.
4. MotorManager, motor ownership transfer, CAN routing, and grouped motor topics.
5. Gimbal, Chassis, and Shoot migration to semantic commands and motor messages.
6. VisionService migration and TIM7 non-blocking transmit.
7. Remove `application/initial`, remove `init_finished`, apply NVIC priorities, and complete
   on-target diagnostics.

Each stage includes an ARM cross-build before proceeding to the next stage.
