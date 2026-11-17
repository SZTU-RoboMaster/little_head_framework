/**
 * @file INS_task.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "INS_task.h"

#include "bsp_imu_pwm.h"
#include "initial_task.h"
#include "math_tools.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Ins_t ins;

extern osThreadId_t defaultTaskHandle;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

void INS_task(void *argument)
{
    INS_Init();
    osThreadFlagsSet(defaultTaskHandle, TASK_READY_INS);

    while (1)
    {
        osThreadFlagsWait(INS_DATA_READY_FLAG, osFlagsWaitAny, osWaitForever);

        INS_Update();
        INS_Publish();

        INS_TempControl();
    }
}

/**
 * @brief
 *
 * @param

 */
void INS_Init(void)
{
    Bmi270_Init(&ins.bmi270);
    Pid_Init(&ins.imu_temp_pid, 1600.0f, 0.2f, 0.0f, 0.0f,
             4400.0f, 4500.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);

    GravityKf_Init(&ins.gravity_kf, 1, 2000);
    QuaternionEkf_Init(&ins.quaternion_ekf, 10, 0.001f, 1000000, 0.9996f);

    Create_Publisher(&ins.publisher, kInsTopicName, sizeof(Ins_Message_t));
}

void INS_Update(void)
{
    GravityKf_Update(&ins.gravity_kf, ins.bmi270.rx_data.gyro[0], ins.bmi270.rx_data.gyro[1],
                     ins.bmi270.rx_data.gyro[2], ins.bmi270.rx_data.accel[0],
                     ins.bmi270.rx_data.accel[1], ins.bmi270.rx_data.accel[2], 0.00125f);
    QuaternionEkf_Update(&ins.quaternion_ekf, ins.bmi270.rx_data.gyro[0],
                         ins.bmi270.rx_data.gyro[1], ins.bmi270.rx_data.gyro[2],
                         ins.gravity_kf.gravity_vec[0], ins.gravity_kf.gravity_vec[1],
                         ins.gravity_kf.gravity_vec[2], 0.00125f);
}

void INS_Publish(void)
{
    Ins_Message_t msg;

    memcpy(msg.angle, ins.quaternion_ekf.ins.angle, sizeof(msg.angle));
    memcpy(msg.gyro, ins.bmi270.rx_data.gyro, sizeof(msg.gyro));
    memcpy(msg.acc, ins.bmi270.rx_data.accel, sizeof(msg.acc));
    memcpy(msg.quaternion, ins.quaternion_ekf.ins.q, sizeof(msg.quaternion));

    Message_Publish(&ins.publisher, &msg, sizeof(msg));
}

void INS_TempControl(void)
{
    static uint8_t first_in = 1;
    if (first_in)
    {
        if (ins.bmi270.rx_data.temp > 43.0f)
        {
            first_in = 0;
        }
        BspImuPwm_Set(4999);
    }
    else
    {
        Pid_SetTarget(&ins.imu_temp_pid, 45.0f);
        Pid_SetFeedback(&ins.imu_temp_pid, ins.bmi270.rx_data.temp);
        Pid_Calculate(&ins.imu_temp_pid);
        int16_t out = (int16_t)clamp_float(Pid_GetOutput(&ins.imu_temp_pid), 0.0f, 4999.0f);
        BspImuPwm_Set(out);
    }
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
