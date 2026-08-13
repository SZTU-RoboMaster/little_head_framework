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
#include "gimbal_task.h"
#include "cmsis_os.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Gimbal gimbal;

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

    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;

        gimbal.update_input();
        gimbal.update_feedback();
        gimbal.handle_safety();
        gimbal.set_mode();
        gimbal.update_control_state();
        gimbal.control();
        gimbal.calculate();
        gimbal.output();

        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
