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
#include <cstring>

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

        ins.update();
        ins.publish();
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

    publisher_ = MessageCenter::instance().advertise<InsMessage>(kInsTopicName);
}

void INS::update()
{
    gravity_kf_.update(bmi088_.rx_data_.gyro[0], bmi088_.rx_data_.gyro[1], bmi088_.rx_data_.gyro[2],
                       bmi088_.rx_data_.accel[0], bmi088_.rx_data_.accel[1],
                       bmi088_.rx_data_.accel[2], 0.001f);
    quaternion_ekf_.update(bmi088_.rx_data_.gyro[0], bmi088_.rx_data_.gyro[1],
                           bmi088_.rx_data_.gyro[2], gravity_kf_.gravity_vec_[0],
                           gravity_kf_.gravity_vec_[1], gravity_kf_.gravity_vec_[2], 0.001f);
}

void INS::publish()
{
    InsMessage msg;

    std::memcpy(msg.angle, quaternion_ekf_.ins_.angle, sizeof(msg.angle));
    std::memcpy(msg.gyro, bmi088_.rx_data_.gyro, sizeof(msg.gyro));
    std::memcpy(msg.acc, bmi088_.rx_data_.accel, sizeof(msg.acc));
    std::memcpy(msg.quaternion, quaternion_ekf_.ins_.q, sizeof(msg.quaternion));

    publisher_.publish(msg);
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
