/**
 * @file bsp_buzzer.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-15 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
void buzzer_init();

void buzzer_on(uint16_t psc, uint16_t pwm);

void buzzer_off(void);

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
