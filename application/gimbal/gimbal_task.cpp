/**
 * @file gimbal_task.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"

#include "command.h"
#include "initial_task.h"

#include "cmsis_os2.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Gimbal gimbal;
Command command;

extern osThreadId_t defaultTaskHandle;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param

 */
extern "C" void gimbal_task(void *argument)
{
    gimbal.init();
    command.init();

    osThreadFlagsSet(defaultTaskHandle, TASK_READY_GIMBAL);
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;

        command.update();

        gimbal.update_input();
        gimbal.update_feedback();
        gimbal.handle_safety();
        gimbal.set_mode();
        gimbal.control();
        gimbal.calculate();
        gimbal.output();

        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
