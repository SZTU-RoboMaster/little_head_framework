/**
 * @file vision_task.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-19 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "vision_task.h"
#include "initial_task.h"
#include "cmsis_os2.h"

/* Private variables ---------------------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

void vision_task(void *argument)
{
    (void)argument;
    Vision_Init();
    osThreadFlagsSet(defaultTaskHandle, TASK_READY_VISION);

    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;
        Vision_Publish_New();
        Vision_Send();
        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
