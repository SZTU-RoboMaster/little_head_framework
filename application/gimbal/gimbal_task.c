/**
 * @file gimbal_task.c
 * @author anchengc
 * @brief 云台任务
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"
#include "command.h"
#include "initial_task.h"
#include "cmsis_os2.h"

/* Private variables ---------------------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

void gimbal_task(void *argument)
{
    (void)argument;
    Gimbal_Init();
    Command_Init();

    osThreadFlagsSet(defaultTaskHandle, TASK_READY_GIMBAL);
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;
        Command_Update();
        Gimbal_Update_Input();
        Gimbal_Update_Feedback();
        Gimbal_Handle_Safety();
        Gimbal_Set_Mode();
        Gimbal_Control();
        Gimbal_Calculate();
        Gimbal_Output();
        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
