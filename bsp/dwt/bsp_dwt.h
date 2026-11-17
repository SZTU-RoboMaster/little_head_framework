/**
 * @file bsp_dwt.h
 * @author anchengc
 * @brief DWT
 * @version 0.1
 * @date 2026-07-12 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void BspDwt_Init(void);

void BspDwt_DelayUs(uint32_t us);

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SZTU-HJ **************************/
