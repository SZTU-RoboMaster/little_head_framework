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
#include "vision_task.h"
#include "initial_task.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Vision vision;

/* Private function declarations ---------------------------------------------*/
extern osThreadId_t defaultTaskHandle;

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param

 */
extern "C" void vision_task(void *argument)
{
    vision.init();

    osThreadFlagsSet(defaultTaskHandle, TASK_READY_VISION);
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;

        vision.update_tx();
        vision.send();

        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
