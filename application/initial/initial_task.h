/**
 * @file initial_task.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-27 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
#define TASK_READY_INS (1U << 0)
#define TASK_READY_GIMBAL (1U << 1)
#define TASK_READY_CHASSIS (1U << 2)
#define TASK_READY_SHOOT (1U << 3)
#define TASK_READY_VISION (1U << 4)

#define TASK_READY_ALL                                                                             \
    (TASK_READY_INS | TASK_READY_GIMBAL | TASK_READY_CHASSIS | TASK_READY_SHOOT | TASK_READY_VISION)

/* Exported types ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
