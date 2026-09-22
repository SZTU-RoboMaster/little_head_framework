# Robot SDK, Message Bus, and RTOS Refactor Implementation Plan

> **Status:** Superseded. Do not execute this plan. Implementation now proceeds incrementally
> through conversation, without the host-test harness or plan task sequence.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current cross-layer `initial_task` architecture with four 1 kHz CMSIS-RTOS2 tasks, a statically allocated latest-value message bus, SDK-owned device services, and a TIM7-driven MotorManager while preserving the current robot control mapping and tuning.

**Architecture:** Build the middleware and device services beside the current runtime first, with no callbacks enabled. Perform one controlled cutover only after every new publisher, subscriber, motor route, and task body exists; this prevents two object graphs from owning or transmitting to the same motor. The final dependency direction is `Core -> SDK/application`, `application -> sdk/message + algorithm components`, and `SDK -> device components -> BSP`.

**Tech Stack:** C++17, C11, STM32F407 HAL, FreeRTOS 10.3.1, CMSIS-RTOS2, CMake/Ninja, native MinGW `g++` host tests with CTest, GNU Arm Embedded cross-build.

## Global Constraints

- Keep `Core/Src/freertos.c` as C. Put task handles, definitions, creation, prototypes, and weak bodies only in CubeMX USER CODE regions.
- Put each strong task function in its owning application `.cpp` with `extern "C"` linkage. Do not create `robot_tasks.cpp`.
- The message bus, services, nodes, messages, and motor manager use fixed storage only. CMSIS task stacks remain dynamically allocated from the existing FreeRTOS heap.
- Do not add an explicit DM enable field or user-requested disable frame. A stale DM target resolves to `torque = 0`, `kp = 0`, and `kd = 0`; the existing driver may still send an enable frame when motor feedback reports disabled.
- Publication timestamps are assigned only by middleware. The public timestamp unit is milliseconds and is backed by `HAL_GetTick()`; sequence numbers, not timestamps, distinguish updates within one millisecond.
- `Subscriber::update()` is polling-only and returns whether a newer sample was copied. There are no callbacks, queues, histories, filters, or runtime topic deletion.
- Preserve current DR16 mappings, controller gains, gearbox ratios, installation directions, and CAN command IDs. Do not add vision-controlled firing or new robot modes in this migration.
- Treat `/motor/*/feedback` freshness as 100 ms. A fresh feedback topic still requires each used motor's `online` field to be true.
- Every task ends with a clean ARM cross-build. Do not proceed while the current task's tests or cross-build fail.

## Target File Map

```text
bsp/
  interrupt/bsp_interrupt.h/.cpp  PRIMASK save/restore
  dwt/bsp_dwt.h/.cpp              existing microsecond delay and diagnostic timing
  can/bsp_can.h/.cpp              callback registration and non-blocking send
  uart/bsp_uart.h/.cpp            callback registration and DMA receive
  usb/bsp_usb.h/.cpp              RX callback and TX-idle query
  tim/bsp_tim.h/.cpp              callback registration and explicit start

sdk/
  message/message_center.h/.cpp   MessageCenter, Node, Publisher, Subscriber
  message/robot_messages.h        all inter-layer payload contracts
  message/topics.h                canonical topic path constants
  motor/motor_manager.h/.cpp      motor ownership, routing, control, grouped IO
  motor/motor_safety.h            pure safe-target helpers used by host tests
  device/imu_service.h/.cpp       BMI088 -> /imu/raw
  device/remote_service.h/.cpp    DR16 -> /input/dr16
  device/vision_service.h/.cpp    USB vision bridge
  robot_diagnostics.h             fixed runtime counters
  robot_error.h/.cpp              C-compatible fatal error entry point
  robot_sdk.h/.cpp                SDK object graph, init/start, BSP routing

application/
  application.h/.cpp              application_init()
  ins/ins.h/.cpp                  fusion and ins_task
  command/command_manager.h/.cpp  DR16 semantic mapping
  gimbal/gimbal.h/.cpp            outer control and gimbal_task
  chassis/chassis.h/.cpp          outer control/kinematics and chassis_task
  shoot/shoot.h/.cpp              firing state machine and shoot_task

components/
  algorithm/ins/quaternion_ekf.h/.cpp  GravityKf and QuaternionEkf
  bmi088/bmi088.h/.cpp                 acquisition and unit conversion only
  motor/motor_dji.h/.cpp               DJI protocol and inner-loop driver
  motor/motor_dm.h/.cpp                DM protocol driver with runtime kp/kd
  rc/dr16.h/.cpp                       18-byte frame decoder
  vision/vision.h/.cpp                 private packed USB wire conversion

Core/
  Src/main.cpp                    startup ordering and scheduler launch
  Src/freertos.c                  CMSIS task creation and weak C task bodies
  Inc/FreeRTOSConfig.h            delay/high-water/failure-hook settings

tests/
  CMakeLists.txt
  support/test_assert.h
  fakes/hal_tick_fake.h/.cpp
  fakes/bsp_interrupt_fake.cpp
  message_center_test.cpp
  message_contracts_test.cpp
  topic_graph_test.cpp
  command_manager_test.cpp
  motor_safety_test.cpp
  check_layering.cmake
```

---

### Task 1: Add the host-test harness, critical section, and MessageCenter

**Files:**
- Create: `bsp/interrupt/bsp_interrupt.h`
- Create: `bsp/interrupt/bsp_interrupt.cpp`
- Create: `sdk/message/message_center.h`
- Create: `sdk/message/message_center.cpp`
- Create: `tests/CMakeLists.txt`
- Create: `tests/support/test_assert.h`
- Create: `tests/fakes/hal_tick_fake.h`
- Create: `tests/fakes/hal_tick_fake.cpp`
- Create: `tests/fakes/bsp_interrupt_fake.cpp`
- Create: `tests/message_center_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Establish a dependency-free host test target**

Set both firmware and host builds to C++17. The host project must compile only the middleware source plus BSP fakes; it must not include HAL, FreeRTOS, or ARM DSP headers.

```cmake
# tests/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(robot_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()

add_executable(message_center_test
    message_center_test.cpp
    fakes/hal_tick_fake.cpp
    fakes/bsp_interrupt_fake.cpp
    ../sdk/message/message_center.cpp
)
target_include_directories(message_center_test PRIVATE ..)
add_test(NAME message_center COMMAND message_center_test)
```

- [ ] **Step 2: Write failing MessageCenter tests**

Cover these cases in separate test functions using a fresh `MessageCenter` per case:

- publisher-first and subscriber-first registration;
- two subscribers retaining independent `last_sequence` values;
- two publishes before one update returning only the newest value;
- `update()` returning false before first publish and when sequence is unchanged;
- middleware stamping the fake time at publish, not at subscriber update;
- `is_fresh()` false before update, true at the timeout boundary, false after it;
- unsigned timestamp wrap from `0xfffffff0U` to `0x00000020U`;
- duplicate publisher, type mismatch, overlong path, 25th topic, pool exhaustion;
- missing publisher at `finalize()`;
- publication rejected before finalization and accepted after finalization;
- registration rejected after finalization.

Run:

```powershell
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests
```

Expected: compilation fails because `sdk/message/message_center.h` does not exist yet.

- [ ] **Step 3: Implement the BSP critical-section interface**

Use these exact target signatures:

```cpp
// bsp/interrupt/bsp_interrupt.h
#pragma once
#include <cstdint>

using BspInterruptState = uint32_t;
BspInterruptState bsp_interrupt_save();
void bsp_interrupt_restore(BspInterruptState state);
```

`bsp_interrupt_save()` reads PRIMASK, disables interrupts, executes data/instruction barriers,
and restore writes the saved PRIMASK rather than blindly enabling interrupts. Keep the existing
DWT-backed microsecond delay unchanged.

- [ ] **Step 4: Implement the fixed-capacity middleware**

The public API and limits are fixed as follows:

```cpp
#include <cstddef>
#include <cstdint>

namespace robot::message {

inline constexpr std::size_t kMaxTopics = 24U;
inline constexpr std::size_t kTopicNameCapacity = 32U;
inline constexpr std::size_t kMaxPayloadSize = 256U;
inline constexpr std::size_t kMessagePoolSize = 4096U;

enum class MessageError : uint8_t {
    None,
    InvalidTopicName,
    TooManyTopics,
    PayloadTooLarge,
    PoolExhausted,
    TypeMismatch,
    DuplicatePublisher,
    MissingPublisher,
    AlreadyFinalized,
};

class MessageCenter;

struct TopicSlot {
    char path[kTopicNameCapacity]{};
    const void *type_token = nullptr;
    void *payload = nullptr;
    std::size_t payload_size = 0U;
    std::size_t payload_alignment = 0U;
    uint32_t timestamp_ms = 0U;
    uint32_t sequence = 0U;
    bool published = false;
    bool publisher_registered = false;
    bool active = false;
    const char *publisher_node = nullptr;
};

template <typename T> class Publisher {
public:
    Publisher() = default;
    bool publish(const T &value) const;
    explicit operator bool() const { return topic_ != nullptr; }
private:
    friend class MessageCenter;
    explicit Publisher(TopicSlot *topic) : topic_(topic) {}
    TopicSlot *topic_ = nullptr;
};

template <typename T> class Subscriber {
public:
    Subscriber() = default;
    bool update(T &output);
    bool is_fresh(uint32_t timeout_ms) const;
    explicit operator bool() const { return topic_ != nullptr; }
private:
    friend class MessageCenter;
    explicit Subscriber(TopicSlot *topic) : topic_(topic) {}
    TopicSlot *topic_ = nullptr;
    uint32_t last_sequence_ = 0U;
    uint32_t last_timestamp_ms_ = 0U;
    bool received_ = false;
};

class Node {
public:
    Node(MessageCenter &center, const char *name) : center_(&center), name_(name) {}
    template <typename T> Publisher<T> advertise(const char *path);
    template <typename T> Subscriber<T> subscribe(const char *path);
private:
    MessageCenter *center_;
    const char *name_;
};

class MessageCenter {
public:
    MessageCenter();
    MessageCenter(const MessageCenter &) = delete;
    MessageCenter &operator=(const MessageCenter &) = delete;
    template <typename T> Publisher<T> advertise(const char *node, const char *path);
    template <typename T> Subscriber<T> subscribe(const char *node, const char *path);
    bool finalize();
    MessageError error() const { return error_; }
    bool finalized() const { return finalized_; }
    std::size_t topic_count() const { return topic_count_; }
private:
    TopicSlot *register_topic(const char *node, const char *path,
                              const void *type_token, std::size_t payload_size,
                              std::size_t payload_alignment, bool publisher);
    TopicSlot topic_slots_[kMaxTopics]{};
    alignas(std::max_align_t) std::byte message_pool_[kMessagePoolSize]{};
    std::size_t topic_count_ = 0U;
    std::size_t pool_offset_ = 0U;
    MessageError error_ = MessageError::None;
    bool finalized_ = false;
};

} // namespace robot::message
```

Keep `TopicSlot` as middleware-internal data even though its complete layout is in the header for the handle templates. All Node objects are static SDK/application members and use string-literal names, so the stored publisher node pointer remains valid for the firmware lifetime. MessageCenter calls `HAL_GetTick()` directly. Its header declares the C-linkage function signature without including the full HAL header; the firmware links the HAL definition and host tests link `hal_tick_fake.cpp`.

Keep template definitions in the header. Keep name lookup, aligned bump allocation, type conflict checks, and finalization in the `.cpp`. Use a per-type address token without RTTI plus compile-time checks for `std::is_trivially_copyable_v<T>`, `sizeof(T) <= kMaxPayloadSize`, and `alignof(T) <= alignof(std::max_align_t)`. A handle stores a direct `TopicSlot*`; no string lookup occurs during `publish()`, `update()`, or `is_fresh()`.

The publish critical section performs only sequence increment, timestamp assignment, payload copy, and published flag assignment. The update critical section performs only sequence comparison and copying payload/metadata. Read the clock outside both critical sections.

`Subscriber::is_fresh(timeout_ms)` returns `received_ && static_cast<uint32_t>(HAL_GetTick() - last_timestamp_ms_) <= timeout_ms`. `Publisher::publish()` calls `HAL_GetTick()` before entering its critical section. Do not add an age-returning API.

Use no FreeRTOS mutex: the same PRIMASK save/restore path must work before scheduler startup, in tasks, and in ISRs. The 256-byte payload cap bounds the maximum copy performed while interrupts are masked; string search, logging, clock reads, callbacks, and device work stay outside the critical section.

- [ ] **Step 5: Run host tests and the ARM cross-build**

```powershell
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
cmake --preset Debug
cmake --build --preset Debug --parallel
```

Expected: `message_center` passes and the firmware links successfully.

- [ ] **Step 6: Commit the middleware foundation**

```powershell
git add CMakeLists.txt bsp/interrupt sdk/message/message_center.h sdk/message/message_center.cpp tests
git commit -m "feat: add static message center foundation"
```

---

### Task 2: Define all message contracts and the canonical topic graph

**Files:**
- Create: `sdk/message/robot_messages.h`
- Create: `sdk/message/topics.h`
- Create: `tests/message_contracts_test.cpp`
- Create: `tests/topic_graph_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write failing contract and graph tests**

`message_contracts_test.cpp` must assert that every payload is trivially copyable, naturally aligned, no larger than 256 bytes, and that the three grouped motor payloads have the expected element counts. `topic_graph_test.cpp` must register the approved publishers and subscribers with mock nodes in both mixed and subscriber-first order, then require `finalize()` to succeed with exactly 16 topic slots.

Use the static node names `imu`, `remote`, `vision`, `motor`, `command`, `ins`, `gimbal`, `chassis`, and `shoot` in the production registrations and mirror them in the graph test.

Run:

```powershell
cmake --build build/host-tests
```

Expected: compilation fails because `robot_messages.h` and `topics.h` do not exist.

- [ ] **Step 2: Add canonical topic paths**

```cpp
namespace robot::topics {
inline constexpr char kImuRaw[] = "/imu/raw";
inline constexpr char kInsState[] = "/ins/state";
inline constexpr char kInputDr16[] = "/input/dr16";
inline constexpr char kVisionCommand[] = "/vision/command";
inline constexpr char kCommandGimbal[] = "/command/gimbal";
inline constexpr char kCommandChassis[] = "/command/chassis";
inline constexpr char kCommandShoot[] = "/command/shoot";
inline constexpr char kGimbalState[] = "/gimbal/state";
inline constexpr char kChassisState[] = "/chassis/state";
inline constexpr char kShootState[] = "/shoot/state";
inline constexpr char kMotorGimbalTarget[] = "/motor/gimbal/target";
inline constexpr char kMotorGimbalFeedback[] = "/motor/gimbal/feedback";
inline constexpr char kMotorChassisTarget[] = "/motor/chassis/target";
inline constexpr char kMotorChassisFeedback[] = "/motor/chassis/feedback";
inline constexpr char kMotorShootTarget[] = "/motor/shoot/target";
inline constexpr char kMotorShootFeedback[] = "/motor/shoot/feedback";
} // namespace robot::topics
```

- [ ] **Step 3: Add exact payload contracts**

Use natural alignment and default member initialization. Do not apply `packed` to these contracts.

```cpp
namespace robot::msg {

enum class ChassisMode : uint8_t { Disabled, FollowGimbal, Spin };
enum class GimbalMode : uint8_t { Disabled, Active, LowerHead };
enum class ShootMode : uint8_t { Disabled, Idle, Continuous };
enum class TriggerState : uint8_t { Disabled, Idle, Angle, Speed, Blocked };
enum class DjiControlMode : uint8_t { Current, Omega, Angle };

struct ImuRaw {
    float accel[3]{};
    float gyro[3]{};
    float temperature{};
};

struct InsState {
    float euler_angle[3]{};
    float quaternion[4]{1.0F, 0.0F, 0.0F, 0.0F};
    float angular_velocity[3]{};
};

struct Dr16State {
    int16_t channel[4]{};
    uint8_t switch_position[2]{};
    int16_t mouse_x{};
    int16_t mouse_y{};
    int16_t mouse_z{};
    uint8_t mouse_left{};
    uint8_t mouse_right{};
    uint16_t key_code{};
    int16_t wheel{};
};

struct VisionCommand {
    float yaw_angle{};
    float yaw_velocity{};
    float yaw_acceleration{};
    float pitch_angle{};
    float pitch_velocity{};
    float pitch_acceleration{};
    bool target_locked{};
    bool fire_command{};
};

struct ChassisCommand {
    ChassisMode mode{ChassisMode::Disabled};
    float vx{};
    float vy{};
    float vw{};
};

struct GimbalCommand {
    GimbalMode mode{GimbalMode::Disabled};
    float yaw_rate{};
    float pitch_rate{};
};

struct ShootCommand {
    ShootMode mode{ShootMode::Disabled};
    uint32_t single_shot_request{};
};

struct GimbalState {
    GimbalMode mode{GimbalMode::Disabled};
    float yaw_relative_angle{};
    float pitch_relative_angle{};
    float yaw_angular_velocity{};
    float pitch_angular_velocity{};
};

struct ChassisState {
    ChassisMode mode{ChassisMode::Disabled};
    float velocity_x{};
    float velocity_y{};
    float omega{};
};

struct ShootState {
    ShootMode mode{ShootMode::Disabled};
    TriggerState trigger_state{TriggerState::Disabled};
    float friction_left_omega{};
    float friction_right_omega{};
    float shoot_speed{};
    uint32_t bullet_count{};
};

struct DjiMotorTarget {
    DjiControlMode control_mode{DjiControlMode::Current};
    float angle{};
    float omega{};
    float current{};
    float feedforward_omega{};
};

struct DmMotorTarget {
    float angle{};
    float omega{};
    float torque{};
    float kp{};
    float kd{};
};

struct DjiMotorFeedback {
    bool online{};
    float angle{};
    float total_angle{};
    float omega{};
    int16_t current{};
    uint8_t temperature{};
};

struct DmMotorFeedback {
    bool online{};
    uint8_t fault_code{};
    float angle{};
    float total_angle{};
    float omega{};
    float torque{};
    uint8_t mos_temperature{};
    uint8_t rotor_temperature{};
};

struct GimbalMotorTarget { DjiMotorTarget yaw{}; DjiMotorTarget pitch{}; };
struct ChassisMotorTarget { DjiMotorTarget wheel[4]{}; };
struct ShootMotorTarget {
    DjiMotorTarget trigger{};
    DjiMotorTarget friction_left{};
    DjiMotorTarget friction_right{};
};
struct GimbalMotorFeedback { DjiMotorFeedback yaw{}; DjiMotorFeedback pitch{}; };
struct ChassisMotorFeedback { DjiMotorFeedback wheel[4]{}; };
struct ShootMotorFeedback {
    DjiMotorFeedback trigger{};
    DjiMotorFeedback friction_left{};
    DjiMotorFeedback friction_right{};
};

} // namespace robot::msg
```

- [ ] **Step 4: Run contract, graph, middleware, and cross-build checks**

```powershell
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
cmake --build --preset Debug --parallel
```

Expected: all host tests pass; the firmware still links.

- [ ] **Step 5: Commit the contracts**

```powershell
git add sdk/message/robot_messages.h sdk/message/topics.h tests
git commit -m "feat: define robot message contracts"
```

---

### Task 3: Harden CMSIS-RTOS2 task creation and fatal diagnostics without switching runtime yet

**Files:**
- Create: `sdk/robot_error.h`
- Create: `sdk/robot_error.cpp`
- Modify: `Core/Src/freertos.c`
- Modify: `Core/Inc/FreeRTOSConfig.h`
- Modify: `little_head_test.ioc`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add a C-compatible fatal error contract**

```c
typedef enum {
    ROBOT_FATAL_NONE = 0,
    ROBOT_FATAL_MESSAGE_REGISTRATION,
    ROBOT_FATAL_MESSAGE_FINALIZE,
    ROBOT_FATAL_TASK_CREATE,
    ROBOT_FATAL_STACK_OVERFLOW,
    ROBOT_FATAL_MALLOC_FAILED,
    ROBOT_FATAL_MOTOR_CONFIG,
    ROBOT_FATAL_SCHEDULER_RETURNED
} RobotFatalError;

extern volatile RobotFatalError g_robot_fatal_error;
void robot_fatal_error(RobotFatalError error);
```

Wrap declarations with `extern "C"` only under `__cplusplus`. The implementation records the first fatal code and calls `Error_Handler()`.

- [ ] **Step 2: Synchronize CubeMX FreeRTOS settings**

Remove `FREERTOS.Tasks01=test,...` from `little_head_test.ioc` and remove `Tasks01` from `FREERTOS.IPParameters`. Add these exact keys and include them in `FREERTOS.IPParameters`:

```text
FREERTOS.INCLUDE_uxTaskGetStackHighWaterMark=1
FREERTOS.INCLUDE_vTaskDelayUntil=1
FREERTOS.configCHECK_FOR_STACK_OVERFLOW=2
FREERTOS.configUSE_MALLOC_FAILED_HOOK=1
```

Mirror the generated values in `FreeRTOSConfig.h`:

```c
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
```

Keep `configTOTAL_HEAP_SIZE` at 15360 bytes for the initial deployment and inspect its minimum remaining value on target before changing it.

- [ ] **Step 3: Normalize the four final application threads in USER CODE regions**

The current workspace already creates INS, Gimbal, Chassis, Shoot, Vision, and default threads
with CMSIS-RTOS2. Keep the compatibility Vision/default threads until the controlled cutover in
Task 8, but normalize the four final application handles, attributes, prototypes, creation calls,
weak bodies, and failure checks in CubeMX USER CODE regions. Use byte-valued `stack_size` fields.

```c
osThreadId_t insTaskHandle;
const osThreadAttr_t insTask_attributes = {
    .name = "insTask",
    .stack_size = 4096U,
    .priority = osPriorityRealtime,
};

osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTask_attributes = {
    .name = "gimbalTask",
    .stack_size = 2048U,
    .priority = osPriorityHigh,
};

osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
    .name = "chassisTask",
    .stack_size = 2048U,
    .priority = osPriorityAboveNormal,
};

osThreadId_t shootTaskHandle;
const osThreadAttr_t shootTask_attributes = {
    .name = "shootTask",
    .stack_size = 2048U,
    .priority = osPriorityAboveNormal,
};

insTaskHandle = osThreadNew(ins_task, NULL, &insTask_attributes);
gimbalTaskHandle = osThreadNew(gimbal_task, NULL, &gimbalTask_attributes);
chassisTaskHandle = osThreadNew(chassis_task, NULL, &chassisTask_attributes);
shootTaskHandle = osThreadNew(shoot_task, NULL, &shootTask_attributes);

if (insTaskHandle == NULL || gimbalTaskHandle == NULL ||
    chassisTaskHandle == NULL || shootTaskHandle == NULL) {
    robot_fatal_error(ROBOT_FATAL_TASK_CREATE);
}
```

This project's CMSIS-RTOS2 adapter interprets `osThreadAttr_t::stack_size` as bytes and divides
it by `sizeof(StackType_t)` before calling `xTaskCreate()`. It passes the CMSIS priority enum
directly to FreeRTOS, so the effective priorities are 48 for INS, 40 for Gimbal, and 32 for
Chassis/Shoot. Because `stack_mem` and `cb_mem` are null, task stacks and control blocks use the
existing FreeRTOS heap.

The weak body for each task has signature `void task_name(void *argument)`, ignores `argument`,
and loops with `osDelay(1U)`. Implement `vApplicationStackOverflowHook()` and
`vApplicationMallocFailedHook()` by calling the fatal error entry point. Task 8 removes the
compatibility `visionTask`, `defaultTask`, and duplicate default-task USB initialization after
VisionService is active.

- [ ] **Step 4: Cross-build the C/C++ linkage boundary**

```powershell
cmake --build --preset Debug --clean-first --parallel
```

Expected: `freertos.c` compiles as C and the firmware links. `main.cpp` still runs the old runtime at this checkpoint, so task sections may be removed by linker garbage collection.

- [ ] **Step 5: Commit the RTOS foundation**

```powershell
git add Core/Src/freertos.c Core/Inc/FreeRTOSConfig.h little_head_test.ioc sdk/robot_error.h sdk/robot_error.cpp CMakeLists.txt
git commit -m "feat: define robot rtos tasks and failure hooks"
```

---

### Task 4: Implement RemoteService and the DR16 semantic CommandManager

**Files:**
- Create: `sdk/device/remote_service.h`
- Create: `sdk/device/remote_service.cpp`
- Create: `sdk/robot_sdk.h`
- Create: `sdk/robot_sdk.cpp`
- Create: `application/application.h`
- Create: `application/application.cpp`
- Create: `application/command/command_manager.h`
- Create: `application/command/command_manager.cpp`
- Create: `tests/command_manager_test.cpp`
- Modify: `components/rc/dr16.h`
- Modify: `components/rc/dr16.cpp`
- Modify: `tests/CMakeLists.txt`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing semantic-mapping tests**

Test the exact approved initial mapping:

- stale or never-received DR16 publishes all three modes as Disabled every update;
- `switch_position[1] == 1` maps Gimbal to Active and Chassis to FollowGimbal; positions 2 and 3 disable both;
- chassis `vx = channel[1] / 660.0F * 4.27F`, `vy = -channel[0] / 660.0F * 4.27F`, and `vw = 0.0F`;
- gimbal `yaw_rate = -channel[2] / 660.0F * 2.0F` and `pitch_rate = channel[3] / 660.0F * 1000.0F`; these values preserve the old per-1-ms target increments when Gimbal integrates rate by 0.001 s;
- a `switch_position[0]` transition from 3 to 1 toggles friction enable exactly once;
- while friction is enabled, switch 2 selects Continuous and other positions select Idle;
- while friction is enabled, one wheel crossing from at most 300 to greater than 300 increments `single_shot_request` exactly once, and holding the wheel high does not increment again;
- the same wheel crossing while friction is disabled does not change the request counter;
- after a 100 ms input timeout, edge history is re-armed from the first recovered frame so reconnection cannot create a false toggle or shot.

Run:

```powershell
cmake --build build/host-tests
```

Expected: compilation fails because CommandManager is absent.

- [ ] **Step 2: Make DR16 parsing report validity**

Change `Dr16::uart_rx_callback()` to return `bool`. Return false for a non-18-byte frame or any channel outside `[-660, 660]`; commit decoded data to `data_` only after all validation passes. Keep UART DMA restart behavior for a wrong frame length.

- [ ] **Step 3: Implement RemoteService**

RemoteService owns no policy. It registers `/input/dr16`, calls the SDK-owned `Dr16`, converts `Dr16Data` to `robot::msg::Dr16State`, and publishes only when parsing succeeds.

```cpp
class RemoteService {
public:
    bool init(robot::message::MessageCenter &center, Dr16 &dr16);
    void start(UART_HandleTypeDef *uart);
    void on_uart_rx(uint8_t *data, uint16_t length);
private:
    Dr16 *dr16_ = nullptr;
    robot::message::Publisher<robot::msg::Dr16State> publisher_{};
};
```

`start()` is not called until `sdk_start()`, after MessageCenter finalization.

- [ ] **Step 4: Implement CommandManager**

CommandManager registers one subscriber and three publishers. `update()` first consumes a new DR16 sample, then evaluates `is_fresh(100000U)`, updates edge state, and publishes all three commands every call. Keep the monotonic shot request counter across disabled periods.

Keep CommandManager as a normal constructible class so each host test can use a fresh instance and MessageCenter. Production keeps one file-local static instance and exposes only these application-level wrappers, avoiding a global public object:

```cpp
bool command_manager_init(robot::message::MessageCenter &center);
void command_manager_update();
```

- [ ] **Step 5: Add SDK/application aggregate initialization without switching `main`**

`sdk_init()` creates message registrations and device objects but starts no BSP receive path. `application_init()` registers CommandManager. The current `main.cpp` remains unchanged until the controlled cutover task.

```cpp
bool sdk_init();
bool sdk_start();
robot::message::MessageCenter &sdk_message_center();

bool application_init(robot::message::MessageCenter &center);
```

Core passes `sdk_message_center()` into `application_init()`. No application source includes `robot_sdk.h`; application depends only on `sdk/message` contracts and algorithm components.

- [ ] **Step 6: Run host and cross-build checks**

```powershell
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
cmake --build --preset Debug --parallel
```

Expected: CommandManager tests pass and the unchanged old runtime still links.

- [ ] **Step 7: Commit input semantics**

```powershell
git add components/rc sdk/device/remote_service.* sdk/robot_sdk.* application/application.* application/command tests CMakeLists.txt
git commit -m "feat: publish dr16 semantic commands"
```

---

### Task 5: Split BMI088 acquisition from INS fusion

**Files:**
- Create: `sdk/device/imu_service.h`
- Create: `sdk/device/imu_service.cpp`
- Create: `sdk/robot_diagnostics.h`
- Create: `application/ins/ins.h`
- Create: `application/ins/ins.cpp`
- Rename: `components/bmi088/quaternion_ekf.h` -> `components/algorithm/ins/quaternion_ekf.h`
- Rename: `components/bmi088/quaternion_ekf.cpp` -> `components/algorithm/ins/quaternion_ekf.cpp`
- Modify: `sdk/robot_sdk.h`
- Modify: `sdk/robot_sdk.cpp`
- Modify: `application/application.cpp`
- Modify: `components/bmi088/bmi088.h`
- Modify: `components/bmi088/bmi088.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add a sample-ready result to BMI088 without moving fusion yet**

Make `Bmi088::exti_read_callback(uint16_t)` return true only after a valid gyro data-ready read. Keep accelerometer and temperature values cached. Preserve the existing filter members temporarily so the old runtime continues to compile; they are removed in the cutover task. Keep `bmi08_delay_us()` on the existing DWT-backed implementation; TIM2 is not added.

- [ ] **Step 2: Implement ImuService registration and ISR publication**

Create the complete fixed diagnostics layout here so every later service uses one object:

```cpp
struct RobotDiagnostics {
    volatile uint32_t can_tx_busy[2][3]{};
    volatile uint32_t can_tx_error[2][3]{};
    volatile uint32_t usb_tx_busy{};
    volatile uint32_t usb_tx_error{};
    volatile uint32_t task_overrun[4]{};
    volatile uint32_t task_min_stack_words[4]{UINT32_MAX, UINT32_MAX,
                                               UINT32_MAX, UINT32_MAX};
    volatile uint32_t tim7_current_us{};
    volatile uint32_t tim7_max_us{};
    volatile uint32_t minimum_heap_bytes{UINT32_MAX};
    volatile uint32_t bmi088_init_fail{};
    volatile uint32_t can_start_fail[2]{};
    volatile uint32_t uart3_start_fail{};
    volatile uint32_t usb_unavailable{};
};

extern RobotDiagnostics g_robot_diagnostics;
```

Define the single object in `robot_sdk.cpp`. ImuService registers `/imu/raw`. In the GPIO EXTI route, call BMI088 and publish a snapshot only when the gyro callback reports ready. The snapshot contains the latest cached accelerometer, gyro, and temperature values. Do not run either Kalman filter in the ISR. A BMI088 initialization failure records a diagnostic and leaves this publisher silent; it does not make `sdk_start()` fatal.

- [ ] **Step 3: Move the fusion algorithm out of the BMI088 device directory**

Use `git mv` for both `quaternion_ekf` files and update includes/CMake paths. `GravityKf` and `QuaternionEkf` remain reusable algorithm classes under `components/algorithm/ins`; application owns their instances and execution. The temporary Bmi088 filter members include the new algorithm path until Task 8 removes them.

- [ ] **Step 4: Implement the INS application**

INS owns `GravityKf` and `QuaternionEkf`, subscribes to `/imu/raw`, and publishes `/ins/state` after every new sample. Preserve the current initialization and axis convention:

```cpp
gravity_kf_.init(1.0F, 2000.0F);
quaternion_ekf_.init(10.0F, 0.001F, 1000000.0F, 0.9996F);

state.euler_angle[0] = quaternion_ekf_.ins_.angle[0];
state.euler_angle[1] = -quaternion_ekf_.ins_.angle[1];
state.euler_angle[2] = quaternion_ekf_.ins_.angle[2];
```

Copy `quaternion_ekf_.ins_.q` and the raw gyro vector into the remaining state fields. If `/imu/raw` is not fresh for 10 ms, do not publish a synthetic INS state.

- [ ] **Step 5: Add the strong INS task in `ins.cpp`**

```cpp
extern "C" void ins_task(void *argument)
{
    (void)argument;
    uint32_t next_tick = osKernelGetTickCount();
    for (;;) {
        next_tick += 1U;
        ins_update();
        osDelayUntil(next_tick);
    }
}
```

- [ ] **Step 6: Cross-build and inspect the strong symbol**

```powershell
cmake --build --preset Debug --clean-first --parallel
arm-none-eabi-nm -C build/Debug/little_head_test.elf | Select-String -Pattern "ins_task"
```

Expected: the cross-build passes. If the symbol is retained at this pre-cutover checkpoint, it is `T ins_task`, not a mangled C++ name.

- [ ] **Step 7: Commit the INS split**

```powershell
git add sdk/device/imu_service.* sdk/robot_diagnostics.h sdk/robot_sdk.* application/ins application/application.cpp components/bmi088 components/algorithm/ins CMakeLists.txt
git commit -m "feat: split imu acquisition from ins fusion"
```

---

### Task 6: Centralize all motor ownership and safety in MotorManager

**Files:**
- Create: `sdk/motor/motor_safety.h`
- Create: `sdk/motor/motor_manager.h`
- Create: `sdk/motor/motor_manager.cpp`
- Create: `tests/motor_safety_test.cpp`
- Modify: `components/motor/motor_dji.h`
- Modify: `components/motor/motor_dji.cpp`
- Modify: `components/motor/motor_dm.h`
- Modify: `components/motor/motor_dm.cpp`
- Modify: `bsp/can/bsp_can.h`
- Modify: `bsp/can/bsp_can.cpp`
- Modify: `sdk/robot_sdk.h`
- Modify: `sdk/robot_sdk.cpp`
- Modify: `tests/CMakeLists.txt`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing pure safety-policy tests**

Test that:

- a safe DJI target is Current mode with all numeric targets zero;
- a safe DM target has zero angle, omega, torque, kp, and kd;
- a target is accepted at exactly 100000 us and replaced by safe output at 100001 us;
- unsigned time wrap preserves the same boundary;
- dynamic DM kp/kd values pass through unchanged while fresh;
- no explicit DM enabled/disabled field exists in the contract.

Run `cmake --build build/host-tests`; expect failure until `motor_safety.h` is implemented.

Use these pure helpers; the test obtains the `fresh` argument from a real Subscriber driven by the fake `HAL_GetTick()` implementation:

```cpp
inline robot::msg::DjiMotorTarget make_safe_dji_target()
{
    return {robot::msg::DjiControlMode::Current, 0.0F, 0.0F, 0.0F, 0.0F};
}

inline robot::msg::DmMotorTarget make_safe_dm_target()
{
    return {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
}

template <typename Target>
Target select_fresh_target(bool fresh, const Target &requested,
                           const Target &safe)
{
    return fresh ? requested : safe;
}
```

- [ ] **Step 2: Add read-only motor state and target application APIs**

Keep PID calculation in `MotorDji`. Add `is_online()`, a const feedback accessor, and hardware-neutral PID configuration methods. Keep the existing current/angle/omega/feedforward setters. For DM, add `is_online()` and a const feedback accessor while retaining its angle, omega, torque, and kp/kd setters. Neither motor component may include an SDK message header.

Retain the old public PID and feedback members only through this pre-cutover checkpoint because the current Gimbal/Chassis still compile against them. Task 8 removes those callers and immediately moves the members behind the new component API.

MotorManager performs the conversion from `DjiMotorTarget`/`DmMotorTarget` to those component setters every cycle. Retain the existing DM auto-enable behavior when feedback reports disabled.

- [ ] **Step 3: Implement the exact static motor configuration**

MotorManager owns nine DJI motors and a zero-length DM array for the current robot. Use this mapping:

| Logical motor | CAN | RX ID | Gearbox | Initial mode | Inner PID |
|---|---|---:|---:|---|---|
| chassis wheel 0 | CAN1 | 0x201 | 3591/187 | Omega | omega 300/0/0 |
| chassis wheel 1 | CAN1 | 0x202 | 3591/187 | Omega | omega 300/0/0 |
| chassis wheel 2 | CAN1 | 0x203 | 3591/187 | Omega | omega 300/0/0 |
| chassis wheel 3 | CAN1 | 0x204 | 3591/187 | Omega | omega 300/0/0 |
| gimbal yaw | CAN1 | 0x205 | 1 | Omega | omega 1000/0/0 |
| gimbal pitch | CAN2 | 0x206 | 1 | Angle | angle 0/0/0, omega 0/0/0 |
| shoot trigger | CAN2 | 0x201 | 36 | Angle | angle 400/0/0, omega 40/0/0 |
| friction right | CAN2 | 0x202 | 1 | Omega | omega 40/0/0 |
| friction left | CAN2 | 0x203 | 1 | Omega | omega 40/0/0 |

All nine entries retain the current `reverse = false` installation setting. Validate duplicate `(CAN bus, RX ID)` routes and duplicate two-byte command-frame offsets during `init()`. Invalid configuration returns false and becomes `ROBOT_FATAL_MOTOR_CONFIG` when `sdk_init()` handles it.

- [ ] **Step 4: Implement grouped subscriptions, feedback, and one-send-per-frame output**

Use the fixed `g_robot_diagnostics` object created in Task 5 for all CAN send outcomes.

Register all six `/motor/*` topics. Every TIM7 update performs this exact order:

1. update three grouped target subscribers;
2. replace targets older than 100 ms with safe targets;
3. every 100 cycles call each motor's online check;
4. apply current targets;
5. calculate all DJI inner loops and compose buffers;
6. send configured DM frames;
7. send CAN1 `0x200`, CAN1 `0x1fe`, CAN2 `0x200`, and CAN2 `0x1fe` once each;
8. publish all three grouped feedback messages.

Keep the existing command IDs `0x1fe` and buffer byte placement; this task does not normalize them to a different DJI convention. If `HAL_CAN_AddTxMessage()` returns busy/error, increment the appropriate diagnostic counter and send only the newest state on the next TIM7 cycle.

Publish grouped feedback every TIM7 cycle even when a motor is absent. The middleware publication timestamp therefore represents MotorManager liveness; each element's `online` and fault fields represent the corresponding physical motor's state. Subscribers use a 100 ms topic freshness timeout and still check the element-level `online` flags.

- [ ] **Step 5: Route CAN callbacks through RobotSdk**

Use separate CAN1 and CAN2 BSP callbacks to call `MotorManager::on_can_rx(bus, id, data)`. Do not expose motor objects or route tables from SDK.

- [ ] **Step 6: Run tests and cross-build**

```powershell
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
cmake --build --preset Debug --clean-first --parallel
```

Expected: all safety and middleware tests pass; static motor configuration links with no application ownership changes yet.

- [ ] **Step 7: Commit MotorManager**

```powershell
git add sdk/motor components/motor bsp/can sdk/robot_sdk.* tests CMakeLists.txt
git commit -m "feat: centralize motor scheduling and safety"
```

---

### Task 7: Add the non-blocking VisionService

**Files:**
- Create: `sdk/device/vision_service.h`
- Create: `sdk/device/vision_service.cpp`
- Modify: `components/vision/vision.h`
- Modify: `components/vision/vision.cpp`
- Modify: `bsp/usb/bsp_usb.h`
- Modify: `bsp/usb/bsp_usb.cpp`
- Modify: `sdk/robot_sdk.h`
- Modify: `sdk/robot_sdk.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Make vision receive parsing report a complete valid frame**

Change the component receive API to return `bool` only when at least one complete header- and CRC-valid frame was decoded. Preserve stream re-synchronization and partial-frame buffering. Convert the packed wire frame into a naturally aligned component data structure before returning it.

- [ ] **Step 2: Keep packed wire layouts private**

Move `RobotData` and `VisionData` packed declarations out of `vision.h` and into `vision.cpp`. Expose naturally aligned `VisionRxData` and `VisionTxData` only. Keep the current wire field order and CRC16 algorithm unchanged. Until Task 8 deletes `initial_task`, retain public naturally aligned `rx_data_`/`tx_data_` members and a no-argument `send()` wrapper so the old runtime still compiles; these are compatibility shims, not additional wire layouts.

- [ ] **Step 3: Add a BSP TX-idle query and stable-buffer send**

`bsp_usb_tx_idle()` reads the CDC class `TxState`. The new `Vision::send(const VisionTxData &data)` first checks idle, then serializes into a member buffer, appends CRC, and calls `CDC_Transmit_FS()`. Never mutate that buffer while USB owns it. Return an enum distinguishing Sent, Busy, and Error so VisionService can update fixed counters. The temporary no-argument wrapper delegates using `tx_data_`.

- [ ] **Step 4: Implement VisionService bus bridging**

Register publisher `/vision/command` and subscribers `/ins/state` and `/shoot/state`. A valid RX frame publishes converted yaw/pitch kinematics, lock, and fire fields. `VisionService::update()` runs after MotorManager in TIM7, consumes the latest INS/Shoot state, and sends only when USB is idle. The application subscriptions may observe vision fields, but this migration does not turn `fire_command` into a shot request.

For outgoing mode 33 data, map INS yaw/pitch Euler angles, yaw/pitch angular velocities, and all four quaternion elements directly; map ShootState `shoot_speed` and `bullet_count` directly. Before the first corresponding state messages, transmit zero angles/velocities/shoot fields and identity quaternion. Convert RX `target_lock == 49` to true (`50` to false) and any nonzero fire byte to true.

- [ ] **Step 5: Cross-build**

```powershell
cmake --build --preset Debug --clean-first --parallel
```

Expected: no packed wire type appears in a public header; USB busy is non-blocking.

- [ ] **Step 6: Commit VisionService**

```powershell
git add sdk/device/vision_service.* components/vision bsp/usb sdk/robot_sdk.* CMakeLists.txt
git commit -m "feat: add nonblocking vision message bridge"
```

---

### Task 8: Perform the controlled application and runtime cutover

**Files:**
- Modify: `sdk/robot_sdk.h`
- Modify: `sdk/robot_sdk.cpp`
- Modify: `sdk/device/imu_service.h`
- Modify: `sdk/device/imu_service.cpp`
- Modify: `sdk/device/remote_service.h`
- Modify: `sdk/device/remote_service.cpp`
- Modify: `sdk/device/vision_service.h`
- Modify: `sdk/device/vision_service.cpp`
- Modify: `application/gimbal/gimbal.h`
- Modify: `application/gimbal/gimbal.cpp`
- Modify: `application/chassis/chassis.h`
- Modify: `application/chassis/chassis.cpp`
- Modify: `application/shoot/shoot.h`
- Modify: `application/shoot/shoot.cpp`
- Modify: `application/application.cpp`
- Modify: `components/bmi088/bmi088.h`
- Modify: `components/bmi088/bmi088.cpp`
- Modify: `components/motor/motor_dji.h`
- Modify: `components/motor/motor_dji.cpp`
- Modify: `components/motor/motor_dm.h`
- Modify: `components/motor/motor_dm.cpp`
- Modify: `components/rc/dr16.h`
- Modify: `components/rc/dr16.cpp`
- Modify: `components/vision/vision.h`
- Modify: `components/vision/vision.cpp`
- Modify: `bsp/can/bsp_can.h`
- Modify: `bsp/can/bsp_can.cpp`
- Modify: `bsp/uart/bsp_uart.h`
- Modify: `bsp/uart/bsp_uart.cpp`
- Modify: `bsp/usb/bsp_usb.h`
- Modify: `bsp/usb/bsp_usb.cpp`
- Modify: `bsp/tim/bsp_tim.h`
- Modify: `bsp/tim/bsp_tim.cpp`
- Modify: `Core/Src/main.cpp`
- Modify: `CMakeLists.txt`
- Delete: `application/initial/initial_task.h`
- Delete: `application/initial/initial_task.cpp`

- [ ] **Step 1: Refactor Gimbal to messages only**

Remove `Bmi088`, `Dr16*`, `MotorDji`, and all HAL/BSP/device includes. Register command, INS, vision, and motor-feedback subscribers plus motor-target and gimbal-state publishers.

Preserve the yaw outer PID `60/0/0`, center angles, nearest-transposition state machine, and existing target integration by applying:

```cpp
target_yaw_angle_ += command.yaw_rate * 0.001F;
target_pitch_angle_ += command.pitch_rate * 0.001F;
```

Require command freshness 100 ms, INS freshness 10 ms, feedback freshness 100 ms, and both motor `online` flags before Active output. Disabled or stale output publishes Current mode with zero current for yaw and pitch. `LowerHead` is represented in the enum but resolves to Disabled until a separate behavior is designed. Publish `/gimbal/state` every cycle.

Build GimbalState from yaw/pitch motor `total_angle`, INS `angular_velocity[2]` for yaw rate, and pitch motor omega for pitch rate, matching the current feedback sources. Subscribe to VisionCommand and track its 100 ms freshness, but do not feed it into the controller in this behavior-preserving cutover.

Define `gimbal_task` in `gimbal.cpp`; call `command_manager_update()` before `gimbal_update()` and use `osDelayUntil(..., 1U)`.

- [ ] **Step 2: Refactor Chassis to messages only**

Remove `Dr16*`, `MotorDji[4]`, direct Gimbal setter, and device/BSP headers. Subscribe to chassis command, gimbal state, and chassis motor feedback. Preserve omega PID `4/0/0`, wheel radius `0.075F`, center distance `0.59463F`, yaw offset `-0.56F`, and current mecanum equations.

In FollowGimbal, use command `vx`/`vy` and compute omega from the existing gimbal-yaw PID. In Spin, pass command `vx`/`vy`/`vw` into the same mecanum solver. The initial DR16 CommandManager does not select Spin, so this branch adds no new current control mapping while keeping the semantic contract usable by the later arbitrator.

Require command freshness 100 ms, gimbal state freshness 10 ms for FollowGimbal, feedback freshness 100 ms, and all four online flags. Disabled output sends four Current-mode zero targets. Publish target and state every cycle. Define the strong `chassis_task` in `chassis.cpp` with `osDelayUntil`.

- [ ] **Step 3: Refactor Shoot to messages only**

Remove its private `Dr16` and three `MotorDji` objects. Subscribe to shoot command, vision command, and grouped feedback. Preserve friction target speed, trigger state machine, block thresholds/timers, trigger step `2*pi/8`, and existing motor directions.

Consume `single_shot_request` with unsigned delta arithmetic: `new_request - last_request` is added to a pending-shot count, then `last_request` is updated. Starting one angle shot decrements the pending count by one. This preserves multiple requests even if latest-value delivery skips intermediate command samples, and repeatedly observing the same counter creates no duplicate. Require command and feedback freshness of 100 ms and all three online flags. Vision command freshness is 100 ms but `fire_command` remains observational only. Disabled output sends Current-mode zero targets to all three motors. Define the strong `shoot_task` in `shoot.cpp` with `osDelayUntil`.

Publish actual trigger state and left/right friction feedback every cycle. Initialize `shoot_speed` and `bullet_count` to zero because the current firmware has no calibrated muzzle-speed or referee-derived bullet-count source; do not synthesize either value from motor speed.

- [ ] **Step 4: Finish device-component ownership cleanup**

Remove `GravityKf`, `QuaternionEkf`, and `update_flag_` from `Bmi088`; fusion now exists only in `application/ins`. Split device initialization from interrupt enable so `sdk_init()` configures the object and `sdk_start()` enables data-ready delivery after MessageCenter finalization. Move the temporary public DJI PID/feedback and DM feedback members behind their configuration/accessor APIs now that no application source reads them directly.

Reduce Dr16 to a pure `bool decode(const uint8_t *frame, uint16_t length, Dr16Data &output)` component. Remove its UART handle, BSP include, status, alive check, and RX counters; RemoteService owns DMA restart and middleware freshness.

- [ ] **Step 5: Remove the temporary Vision compatibility API**

Delete the public `Vision::rx_data_`, `Vision::tx_data_`, no-argument `send()` shim, old `VisionStatus`, `check_alive_100ms()`, and RX flag counters retained through Task 7. VisionService becomes the only owner of naturally aligned vision input/output data and middleware freshness, while packed wire storage remains private inside the component.

- [ ] **Step 6: Remove `init_finished` and make BSP activation explicit**

Delete every `extern uint8_t init_finished` and callback guard. Registration functions store callbacks without starting hardware. Add explicit start calls used only by `sdk_start()`:

```cpp
void can_register_callback(CAN_HandleTypeDef *handle, can_callback_t callback);
bool can_start(CAN_HandleTypeDef *handle);
void uart_register_callback(UART_HandleTypeDef *handle, uart_callback_t callback,
                            uint16_t receive_length);
bool uart_start(UART_HandleTypeDef *handle);
void tim_register_callback(TIM_HandleTypeDef *handle, tim_callback_t callback);
bool tim_start(TIM_HandleTypeDef *handle);
void usb_register_callback(usb_callback_t callback);
```

USB RX drops data when no callback is registered. TIM7 is started last, after CAN/UART/USB callback installation and BMI interrupt setup.

`sdk_start()` first finalizes MessageCenter, then installs CAN1/CAN2, UART3, USB, EXTI, and TIM7 routes in that order. A message graph or motor-route error is fatal; a BMI088, DR16 UART, CAN peripheral, or USB availability failure records diagnostics and leaves dependent topics silent/safe without halting startup.

The final private `RobotSdk` aggregate owns one MessageCenter, MotorManager, Bmi088, Dr16, Vision, ImuService, RemoteService, and VisionService. Services hold references to their device objects; MotorManager alone owns all motor objects. Expose only the three free functions `sdk_init()`, `sdk_start()`, and `sdk_message_center()`.

- [ ] **Step 7: Wire the final startup order in `main.cpp`**

In USER CODE sections include `robot_sdk.h`, `application.h`, and `robot_error.h`. Declare `extern "C" void MX_FREERTOS_Init(void);`. Replace `task_init()` and the busy-loop work with:

```cpp
if (!sdk_init()) {
    robot_fatal_error(ROBOT_FATAL_MESSAGE_REGISTRATION);
}
if (!application_init(sdk_message_center())) {
    robot_fatal_error(ROBOT_FATAL_MESSAGE_REGISTRATION);
}
MX_FREERTOS_Init();
if (!sdk_start()) {
    robot_fatal_error(ROBOT_FATAL_MESSAGE_FINALIZE);
}
osKernelStart();
robot_fatal_error(ROBOT_FATAL_SCHEDULER_RETURNED);
```

Within `sdk_init()`, register MotorManager, ImuService, RemoteService, and VisionService in that order. Within `application_init()`, register CommandManager, INS, Gimbal, Chassis, and Shoot in that order. The `while (1)` body remains empty and unreachable.

- [ ] **Step 8: Replace the CMake source and include lists atomically**

Add every SDK, application, BSP interrupt, and Shoot source explicitly. Remove `application/initial/initial_task.cpp` and its include directory. Keep `Core/Src/freertos.c` in `cmake/stm32cubemx/CMakeLists.txt`; do not create or reference `freertos.cpp`.

- [ ] **Step 9: Run all host tests and a clean cross-build**

```powershell
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
cmake --build --preset Debug --clean-first --parallel
arm-none-eabi-nm -C build/Debug/little_head_test.elf | Select-String -Pattern "(ins_task|gimbal_task|chassis_task|shoot_task)"
arm-none-eabi-size build/Debug/little_head_test.elf
```

Expected: all tests pass; exactly four strong unmangled task symbols are present; `application/initial` and `init_finished` have no references; firmware Flash/RAM usage is printed for review.

- [ ] **Step 10: Commit the cutover**

```powershell
git add sdk/robot_sdk.* sdk/device application components/bmi088 components/motor components/rc components/vision bsp Core/Src/main.cpp CMakeLists.txt
git commit -m "refactor: cut over applications to sdk messages"
```

---

### Task 9: Apply the approved NVIC table, layering checks, and runtime diagnostics

**Files:**
- Modify: `little_head_test.ioc`
- Modify: `Core/Src/gpio.c`
- Modify: `Core/Src/dma.c`
- Modify: `Core/Src/usart.c`
- Modify: `Core/Src/can.c`
- Modify: `Core/Src/tim.c`
- Modify: `USB_DEVICE/Target/usbd_conf.c`
- Modify: `sdk/robot_diagnostics.h`
- Modify: `sdk/robot_sdk.cpp`
- Modify: `application/ins/ins.cpp`
- Modify: `application/gimbal/gimbal.cpp`
- Modify: `application/chassis/chassis.cpp`
- Modify: `application/shoot/shoot.cpp`
- Create: `tests/check_layering.cmake`
- Create: `docs/verification/robot-sdk-bringup.md`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Add an automated layer-boundary test first**

The CMake script must fail if an `application/` source includes `bsp_*`, `can.h`, `tim.h`, `usart.h`, `usb_device.h`, `dr16.h`, `bmi088.h`, `motor_dji.h`, `motor_dm.h`, `vision.h`, `robot_sdk.h`, or anything under `sdk/device` or `sdk/motor`. It must catch flat and project-relative include paths. It must also fail if any `sdk/` source includes a path under `application/`.

```cmake
if(NOT DEFINED ROOT)
    message(FATAL_ERROR "ROOT is required")
endif()

file(GLOB_RECURSE APPLICATION_SOURCES
    "${ROOT}/application/*.h" "${ROOT}/application/*.cpp")
set(APP_FORBIDDEN
    "#[ \t]*include[ \t]*\"([^\"]*/)?(bsp_[^\"]+|can\\.h|tim\\.h|usart\\.h|usb_device\\.h|dr16\\.h|bmi088\\.h|motor_dji\\.h|motor_dm\\.h|vision\\.h|robot_sdk\\.h|sdk/(device|motor)/[^\"]+)\"")
foreach(SOURCE IN LISTS APPLICATION_SOURCES)
    file(READ "${SOURCE}" CONTENT)
    if(CONTENT MATCHES "${APP_FORBIDDEN}")
        message(FATAL_ERROR "application layer violation: ${SOURCE}")
    endif()
endforeach()

file(GLOB_RECURSE SDK_SOURCES "${ROOT}/sdk/*.h" "${ROOT}/sdk/*.cpp")
foreach(SOURCE IN LISTS SDK_SOURCES)
    file(READ "${SOURCE}" CONTENT)
    if(CONTENT MATCHES "#[ \t]*include[ \t]*\"[^\"]*application/")
        message(FATAL_ERROR "sdk layer violation: ${SOURCE}")
    endif()
endforeach()
```

Register it with CTest:

```cmake
add_test(
    NAME layering
    COMMAND ${CMAKE_COMMAND}
            -DROOT=${CMAKE_CURRENT_LIST_DIR}/..
            -P ${CMAKE_CURRENT_LIST_DIR}/check_layering.cmake
)
```

Run:

```powershell
ctest --test-dir build/host-tests -R layering --output-on-failure
```

Expected before cleanup: fail if any forbidden include remains. Remove the dependency, then require the test to pass.

- [ ] **Step 2: Apply the exact priority table to `.ioc` and generated sources**

Keep priority grouping 4 and subpriority 0 everywhere.

| Priority | Interrupts |
|---:|---|
| 4 | EXTI4, EXTI9_5, DMA2 Stream0, DMA2 Stream5 |
| 5 | CAN1 RX0/RX1, CAN2 RX0/RX1, TIM7, OTG_FS |
| 6 | USART3, DMA1 Stream1 |
| 7 | USART1, USART6, DMA2 Stream1/2/6/7 |
| 8 | EXTI3 |
| 10 | EXTI0 |
| 15 | SysTick, PendSV |

Do not add TIM2 in this iteration. Keep `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` at 5. Confirm that priority-4 BMI/SPI paths call no FreeRTOS/CMSIS FromISR API.

- [ ] **Step 3: Add fixed diagnostics without ISR logging**

Use volatile counters for CAN busy/error by bus/frame, USB busy/error, task overruns, TIM7 current/max execution microseconds, minimum observed task stack high-water marks, and minimum FreeRTOS heap. Initialize DWT measurement in `sdk_start()` and measure TIM7 with the BSP cycle API; this does not change publication timestamp behavior.

At 1 Hz, each task updates only its own stack-minimum slot. INS alone updates the global minimum heap value, avoiding competing read/modify/write operations between tasks. Each task increments only its own overrun counter; TIM7 owns its timing fields. Do not call `printf`, allocate memory, or format strings in any ISR.

- [ ] **Step 4: Write the on-target bring-up checklist**

The document must record concrete procedures and pass criteria for:

- four task release periods at 1 kHz;
- execution order INS, then Gimbal/CommandManager, then Chassis/Shoot;
- TIM7 worst case below 500 us;
- DR16 removal producing Disabled commands within 100 ms;
- stopping each motor target topic producing zero output within 100 ms;
- individual motor removal changing only its `online` field and forcing safe output;
- repeated USB busy intervals not extending TIM7;
- stack high-water and minimum heap readings;
- debugger inspection of all NVIC priority registers.

- [ ] **Step 5: Run final automated verification**

```powershell
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
cmake --build --preset Debug --clean-first --parallel
rg -n "init_finished|initial_task|freertos\.cpp" application bsp sdk Core CMakeLists.txt cmake/stm32cubemx/CMakeLists.txt
arm-none-eabi-nm -C build/Debug/little_head_test.elf | Select-String -Pattern "(ins_task|gimbal_task|chassis_task|shoot_task)"
arm-none-eabi-size build/Debug/little_head_test.elf
```

Expected: tests and cross-build pass; `rg` returns no matches; the symbol query reports four `T` task symbols and no `W` task symbols in the final ELF.

- [ ] **Step 6: Commit priorities and diagnostics**

```powershell
git add little_head_test.ioc Core/Src/gpio.c Core/Src/dma.c Core/Src/usart.c Core/Src/can.c Core/Src/tim.c USB_DEVICE/Target/usbd_conf.c sdk/robot_diagnostics.h sdk/robot_sdk.cpp application/ins/ins.cpp application/gimbal/gimbal.cpp application/chassis/chassis.cpp application/shoot/shoot.cpp tests docs/verification
git commit -m "chore: enforce irq priorities and runtime diagnostics"
```

---

### Task 10: Perform final hardware validation and record measured budgets

**Files:**
- Modify: `docs/verification/robot-sdk-bringup.md`

- [ ] **Step 1: Flash the clean Debug build and validate startup failures**

Confirm all four task handles are non-null, `g_robot_fatal_error == ROBOT_FATAL_NONE`, MessageCenter finalized with 16 topics, and no device absence causes a fatal halt.

- [ ] **Step 2: Execute every bring-up checklist item**

Record measured TIM7 maximum, each task's minimum stack high-water mark, minimum FreeRTOS heap, CAN/USB drop counters, and the observed timeout latencies. Do not reduce stack sizes in this refactor; use measurements for a later dedicated change.

- [ ] **Step 3: Re-run clean builds after hardware-derived corrections**

```powershell
ctest --test-dir build/host-tests --output-on-failure
cmake --build --preset Debug --clean-first --parallel
cmake --preset Release
cmake --build --preset Release --clean-first --parallel
arm-none-eabi-size build/Debug/little_head_test.elf build/Release/little_head_test.elf
git status --short
```

Expected: both firmware configurations and all host tests pass; only the intended verification record is uncommitted.

- [ ] **Step 4: Commit the measured validation record**

```powershell
git add docs/verification/robot-sdk-bringup.md
git commit -m "test: record robot sdk hardware validation"
```
