/**
 * @file bsp_buzzer.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-15 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_buzzer.h"

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
void BspBuzzer_Init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void BspBuzzer_On(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim1, psc);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);

}
void BspBuzzer_Off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
