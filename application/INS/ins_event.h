/**
 * @file ins_event.h
 * @brief INS 数据就绪线程标志和线程句柄
 */

#pragma once

#include "cmsis_os2.h"

#define INS_DATA_READY_FLAG (1U << 0)

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t insTaskHandle;

#ifdef __cplusplus
}
#endif
