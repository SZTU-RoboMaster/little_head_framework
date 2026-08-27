/**
 * @file template.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "shoot_task.h"
#include "initial_task.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
Shoot shoot;

/* Private variables ---------------------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 */
extern "C" void shoot_task(void *argument)
{
    shoot.init();

    osThreadFlagsSet(defaultTaskHandle, TASK_READY_SHOOT);
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;

        shoot.update_input();
        shoot.update_feedback();
        shoot.handle_safety();
        shoot.set_mode();
        shoot.update_control_state();
        shoot.control();
        shoot.output();

        osDelayUntil(start + ticks);
    }
    /* USER CODE END shoot_task */
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
