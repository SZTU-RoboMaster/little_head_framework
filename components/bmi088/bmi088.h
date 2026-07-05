/**
 * @file bmi088.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bmi088_interface.h"
#include "bmi08x.h"
#include "quaternion_ekf.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088数据结构体
 *
 */
struct Bmi088RxData
{
    float accel[3]; // 加速度计数据
    float temp;     // 温度数据
    float gyro[3];  // 陀螺仪数据
};

/**
 * @brief Specialized
 *
 */
class Bmi088
{
public:
    // 加速度计kf
    GravityKf gravity_kf_;
    // 四元数ekf
    QuaternionEkf quaternion_ekf_;

    // 数据更新标志位
    uint8_t update_flag_ = 0;

    // BMI088处理后的数据
    Bmi088RxData rx_data_;

    // z轴陀螺仪零飘值
    float gyro_bias_z_;

    int8_t init();

    void exti_read_callback(uint16_t gpio_pin);

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // bmi088设备结构体
    bmi08_dev bmi08dev_;
    // 加速度计中断配置
    bmi08_accel_int_channel_cfg accel_int_config_;
    // 陀螺仪中断配置
    bmi08_gyro_int_channel_cfg gyro_int_config_;
    // 加速度计源数据
    bmi08_sensor_data raw_accel_;
    // 陀螺仪源数据
    bmi08_sensor_data raw_gyro_;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
    int8_t enable_bmi08_interrupt();

    float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

    float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

    void calibrate_gyro_bias_z(volatile float gyro_z);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
