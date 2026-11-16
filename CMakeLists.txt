cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME little_head_test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_f32.c
#============================================================================//
    bsp/can/bsp_can.c
    bsp/tim/bsp_tim.c
    bsp/uart/bsp_uart.c
    bsp/usb/bsp_usb.c
    bsp/dwt/bsp_dwt.c
    bsp/led/bsp_led.c
    bsp/buzzer/bsp_buzzer.c
    bsp/imu_pwm/bsp_imu_pwm.c
#============================================================================//
    components/motor/motor_dji.c
    components/motor/motor_dm.c
    components/rc/dr16.c
    components/rc/vt13.c
    components/support/CRC8_CRC16.c
    components/referee/referee.c
    components/bmi088/third_party/bmi08a.c
    components/bmi088/third_party/bmi08g.c
    components/bmi088/third_party/bmi08xa.c
    components/bmi088/bmi088_interface.c
    components/bmi088/bmi088.c
    components/algorithm/filter/low_pass_filter.c
    components/algorithm/filter/quaternion_ekf.c
    components/algorithm/math_tools/math_tools.c
    components/algorithm/filter/kalman_filter.c
    components/algorithm/pid/pid.c
    # components/ist8310/ist8310driver.c
    # components/ist8310/ist8310driver_middleware.c
    components/vision/vision.c
#============================================================================//
    application/gimbal/gimbal.c
    application/shoot/shoot.c
    application/chassis/chassis.c
    application/initial/initial_task.c
    application/INS/INS_task.c
    application/gimbal/gimbal_task.c
    application/chassis/chassis_task.c
    application/shoot/shoot_task.c
    application/vision/vision_task.c
    application/command/command.c
    application/chassis/power_control/power_controller.c
#============================================================================//
    sdk/robot/robot_sdk.c
    sdk/message/message_center.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/CMSIS/DSP/Include
    bsp/can
    bsp/tim
    bsp/uart
    bsp/usb
    bsp/dwt
    bsp/led
    bsp/buzzer
    bsp/imu_pwm
    components/motor
    components/rc
    components/support
    components/referee
    components/bmi088
    components/bmi088/third_party
    components/algorithm/math_tools
    components/algorithm/filter
    components/algorithm/pid
    components/ist8310
    components/vision
    application/gimbal
    application/chassis
    application/initial
    application/shoot
    application/INS
    application/vision
    sdk/message
    application/command
    application/chassis/power_control
    sdk/robot
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_LOOPUNROLL
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
