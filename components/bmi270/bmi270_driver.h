/**
 * @file bmi270_driver.h
 * @author anchengc
 * @brief BMI270驱动
 * @version 0.1
 * @date 2026-09-18 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

#include "bmi2_defs.h"

typedef struct
{
    float accel[3];
    float gyro[3];
    float temp;
} Bmi270RxData_t;

typedef struct
{
    Bmi270RxData_t rx_data;
    struct bmi2_dev bmi270dev;
    struct bmi2_sens_data sensor_data;
    float gyro_bias_z;
} Bmi270_t;

int8_t Bmi270_Init(Bmi270_t *self);
uint8_t Bmi270_ExtiReadCallback(Bmi270_t *self, uint16_t gpio_pin);

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
