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
#include "bmi08_defs.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088数据结构体
 *
 */
typedef struct
{
    float accel[3]; // 加速度计数据
    float temp;     // 温度数据
    float gyro[3];  // 陀螺仪数据
} Bmi088RxData_t;

/**
 * @brief Specialized
 *
 */
typedef struct
{
    // BMI088处理后的数据
    Bmi088RxData_t rx_data;
    // 初始化相关常量

    // 常量

    // 内部变量

    // bmi088设备结构体
    struct bmi08_dev bmi08dev;
    // 加速度计中断配置
    struct bmi08_accel_int_channel_cfg accel_int_config;
    // 陀螺仪中断配置
    struct bmi08_gyro_int_channel_cfg gyro_int_config;
    // 加速度计源数据
    struct bmi08_sensor_data raw_accel;
    // 陀螺仪源数据
    struct bmi08_sensor_data raw_gyro;
    // z轴陀螺仪零飘值
    float gyro_bias_z;

    // 读变量

    // 写变量

    // 读写变量

} Bmi088_t;

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

int8_t Bmi088_Init(Bmi088_t *self);
void Bmi088_ExtiReadCallback(Bmi088_t *self, uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
