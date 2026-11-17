//
// Created by hjr on 2026/11/17.
//

#include "board_comm_task.h"
/**
 * @file gimbal_task.c
 * @author hjr
 * @brief 板间通信任务
 * @version 0.1
 * @date 2026-11-18 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "board_comm_task.h"

#include "cmsis_os2.h"
#include "initial_task.h"

/* Private variables ---------------------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

void board_comm_task(void *argument)
{
    (void)argument;

    osThreadFlagsSet(defaultTaskHandle, TASK_READY_GIMBAL);
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;

        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
