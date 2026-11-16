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
#ifdef __cplusplus
extern "C" {
#endif

void BspBuzzer_Init(void);

void BspBuzzer_On(uint16_t psc, uint16_t pwm);

void BspBuzzer_Off(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
