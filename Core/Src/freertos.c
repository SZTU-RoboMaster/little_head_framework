/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t insTaskHandle;
const osThreadAttr_t insTask_attributes = {
  .name = "insTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTask_attributes = {
  .name = "gimbalTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t shootTaskHandle;
const osThreadAttr_t shootTask_attributes = {
  .name = "shootTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
osThreadId_t visionTaskHandle;
const osThreadAttr_t visionTask_attributes = {
  .name = "visionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void INS_task(void * argument);
void gimbal_task(void * argument);
void chassis_task(void * argument);
void shoot_task(void * argument);
void vision_task(void * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  insTaskHandle = osThreadNew(INS_task, NULL, &insTask_attributes);
  gimbalTaskHandle = osThreadNew(gimbal_task, NULL, &gimbalTask_attributes);
  chassisTaskHandle = osThreadNew(chassis_task, NULL, &chassisTask_attributes);
  shootTaskHandle = osThreadNew(shoot_task, NULL, &shootTask_attributes);
  visionTaskHandle = osThreadNew(vision_task, NULL, &visionTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  osThreadExit();
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
__weak void INS_task(void * argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

__weak void gimbal_task(void * argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

__weak void chassis_task(void * argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

__weak void shoot_task(void * argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

__weak void vision_task(void * argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
}

/* USER CODE END Application */

