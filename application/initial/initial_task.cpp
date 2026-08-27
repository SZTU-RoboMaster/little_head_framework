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
#include "initial_task.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern osThreadId_t defaultTaskHandle;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param
 */
extern "C" void StartDefaultTask(void *argument)
{
    osThreadFlagsWait(TASK_READY_ALL, osFlagsWaitAll, osWaitForever);

    robot_sdk_start();

    osThreadExit();
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
