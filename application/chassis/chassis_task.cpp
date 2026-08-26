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
#include "chassis_task.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Chassis chassis;
PowerController power_controller;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 */
extern "C" void chassis_task(void *argument)
{
    chassis.init();
    power_controller.init(POWER_CONTROL_ENABLE);

    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks++;

        chassis.update_input();
        chassis.update_feedback();
        chassis.handle_safety();
        chassis.set_mode();
        chassis.control();
        chassis.solve();
        chassis.output();

        power_controller.update();

        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
