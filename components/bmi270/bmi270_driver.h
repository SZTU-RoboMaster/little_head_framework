/**
 * @file bmi270_driver.h
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
#include "bmi2_defs.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088数据结构体
 *
 */
struct Bmi270RxData
{
    float accel[3]; // 加速度计数据
    float gyro[3];  // 陀螺仪数据
    float temp;     // 温度数据
};

/**
 * @brief Specialized
 *
 */
class Bmi270
{
public:
    // BMI270处理后的数据
    Bmi270RxData rx_data_;

    int8_t init();

    void exti_read_callback(uint16_t gpio_pin);

protected:
    // 初始化相关常量

    // 常量

    // 内部变量


    // bmi270设备结构体
    bmi2_dev bmi270dev_;

    // 陀螺仪源数据
    bmi2_sens_data sensor_data_ = {{0}};
    // z轴陀螺仪零飘值
    float gyro_bias_z_;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
    int8_t enable_bmi2_interrupt();

    void calibrate_gyro_bias_z(volatile float gyro_z);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
