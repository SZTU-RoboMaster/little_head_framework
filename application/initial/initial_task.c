/**
 * @file initial_task.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-27 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "initial_task.h"

#include "robot_sdk.h"

#include "cmsis_os2.h"

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
void StartDefaultTask(void *argument)
{
    osThreadFlagsWait(TASK_READY_ALL, osFlagsWaitAll, osWaitForever);

    RobotSdk_Start();

    osThreadExit();
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
