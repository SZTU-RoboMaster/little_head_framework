/**
 * @file template.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-16 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_imu_pwm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param

 */
void imu_pwm_init()
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
