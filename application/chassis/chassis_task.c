/**
 * @file chassis_task.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"
#include "initial_task.h"
#include "power_controller.h"
#include "cmsis_os2.h"

/* Private variables ---------------------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

void chassis_task(void *argument)
{
    (void)argument;
    Chassis_Init();
    PowerController_Init(POWER_CONTROL_ENABLE);

    osThreadFlagsSet(defaultTaskHandle, TASK_READY_CHASSIS);
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;
        Chassis_Update_Input();
        Chassis_Update_Feedback();
        Chassis_Handle_Safety();
        Chassis_Set_Mode();
        Chassis_Control();
        Chassis_Solve();
        Chassis_Output();
        PowerController_Update();
        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
