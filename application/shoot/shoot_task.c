/**
 * @file shoot_task.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "shoot_task.h"
#include "initial_task.h"
#include "cmsis_os2.h"

/* Private variables ---------------------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

void shoot_task(void *argument)
{
    (void)argument;
    Shoot_Init();
    osThreadFlagsSet(defaultTaskHandle, TASK_READY_SHOOT);

    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;
        Shoot_Update_Input();
        Shoot_Update_Feedback();
        Shoot_Handle_Safety();
        Shoot_Set_Mode();
        Shoot_Update_Control_State();
        Shoot_Control();
        Shoot_Output();
        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
