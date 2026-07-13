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
#include "INS_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
INS ins;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

extern "C" void INS_task(void *argument)
{
    ins.init();

    while (1)
    {
        osThreadFlagsWait(INS_DATA_READY_FLAG, osFlagsWaitAny, osWaitForever);

        ins.gravity_kf_.update(ins.bmi088_.rx_data_.gyro[0], ins.bmi088_.rx_data_.gyro[1],
                               ins.bmi088_.rx_data_.gyro[2], ins.bmi088_.rx_data_.accel[0],
                               ins.bmi088_.rx_data_.accel[1], ins.bmi088_.rx_data_.accel[2],
                               0.001f);
        ins.quaternion_ekf_.update(ins.bmi088_.rx_data_.gyro[0], ins.bmi088_.rx_data_.gyro[1],
                                   ins.bmi088_.rx_data_.gyro[2], ins.gravity_kf_.gravity_vec_[0],
                                   ins.gravity_kf_.gravity_vec_[1], ins.gravity_kf_.gravity_vec_[2],
                                   0.001f);
    }
}

/**
 * @brief
 *
 * @param

 */
void INS::init()
{
    bmi088_.init();

    gravity_kf_.init(1, 2000);
    quaternion_ekf_.init(10, 0.001, 1000000, 0.9996);
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
