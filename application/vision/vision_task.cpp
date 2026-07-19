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
#include "INS_task.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Vision vision;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param

 */
extern "C" void vision_task(void *argument)
{
    uint32_t start = osKernelGetTickCount();
    uint32_t ticks = 0;
    while (1)
    {
        ticks ++;
        vision.tx_data_.pitch = ins.quaternion_ekf_.ins_.angle[1];
        vision.tx_data_.yaw = ins.quaternion_ekf_.ins_.angle[2];
        vision.tx_data_.pitch_vel = ins.bmi088_.rx_data_.gyro[1];
        vision.tx_data_.yaw_vel = ins.bmi088_.rx_data_.gyro[2];
        memcpy(vision.tx_data_.quaternion, ins.quaternion_ekf_.ins_.q, sizeof(ins.quaternion_ekf_.ins_.q));

        vision.send();

        osDelayUntil(start + ticks);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
