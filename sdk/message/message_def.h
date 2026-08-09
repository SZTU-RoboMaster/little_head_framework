/**
 * @file template.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
inline constexpr char kInsTopicName[] = "/ins";

/* Exported types ------------------------------------------------------------*/
struct InsMessage
{
    float angle[3];
    float gyro[3];
    float acc[3];
    float quaternion[4];
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
