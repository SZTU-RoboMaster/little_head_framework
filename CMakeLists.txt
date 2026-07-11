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
project(${CMAKE_PROJECT_NAME})
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
    bsp/can/bsp_can.cpp
    bsp/tim/bsp_tim.cpp
    components/motor/motor_dji.cpp
    components/motor/motor_dm.cpp
    bsp/uart/bsp_uart.cpp
    components/rc/dr16.cpp
    components/rc/vt13.cpp
    components/support/CRC8_CRC16.c
    components/referee/referee.cpp
    components/bmi088/third_party/bmi08a.c
    components/bmi088/third_party/bmi08g.c
    components/bmi088/third_party/bmi08xa.c
    components/bmi088/bmi088_interface.cpp
    components/bmi088/bmi088.cpp
    components/algorithm/math_tools/math_tools.cpp
    components/algorithm/kf/kalman_filter.c
    components/bmi088/quaternion_ekf.cpp
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    components/algorithm/pid/pid.cpp
    components/ist8310/ist8310driver.c
    components/ist8310/ist8310driver_middleware.c
    application/gimbal/gimbal.cpp
    application/shoot/shoot.cpp
    application/chassis/chassis.cpp
    application/initial/initial_task.cpp
    bsp/usb/bsp_usb.cpp
    components/vision/vision.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/CMSIS/DSP/Include
    bsp/can
    bsp/tim
    bsp/uart
    components/motor
    components/rc
    components/support
    components/referee
    components/bmi088
    components/bmi088/third_party
    components/algorithm/math_tools
    components/algorithm/kf
    components/algorithm/pid
    components/ist8310
    application/gimbal
    application/chassis
    application/initial
    application/shoot
    bsp/usb
    components/vision
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
