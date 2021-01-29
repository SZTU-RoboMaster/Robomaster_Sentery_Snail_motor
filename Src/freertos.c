/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <PWM_control.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_receive&send.h"
#include "DBUS_remote_control.h"
#include "LED_control.h"
#include "IMU_updata.h"
#include "MotorPID_test.h"
#include "DegCyclePID.h"
#include "DegCyclePID_3508.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "PWM_control.h"
#include "FIRE.h"
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

/* USER CODE END Variables */
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 256 * 4
};
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
  .name = "RemoteTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for Arm_Gimbal_Task */
osThreadId_t Arm_Gimbal_TaskHandle;
const osThreadAttr_t Arm_Gimbal_Task_attributes = {
  .name = "Arm_Gimbal_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ChassisTask_callback(void *argument);
void RemoteTask_callback(void *argument);
void Arm_Gimbal_Task_callback(void *argument);

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
  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(ChassisTask_callback, NULL, &ChassisTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(RemoteTask_callback, NULL, &RemoteTask_attributes);

  /* creation of Arm_Gimbal_Task */
  Arm_Gimbal_TaskHandle = osThreadNew(Arm_Gimbal_Task_callback, NULL, &Arm_Gimbal_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ChassisTask_callback */
/**
  * @brief  Function implementing the ChassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ChassisTask_callback */
void ChassisTask_callback(void *argument)
{
  /* USER CODE BEGIN ChassisTask_callback */
    /* Infinite loop */
    int i=1050;
    int j=0;

  for(;;)
  {
// //     MotorON(RC_data.rc.ch[0],RC_data.rc.ch[1],RC_data.rc.ch[2]);
   //// MotorSetDeg_3508(90);
               i++;
               j++;

      //         Fire_test(RC_data.rc.ch[1]);

     // else if(RC_data.rc.s[1]==RC_SW_MID){
     //     FLAG=0;
     //     Fire_test(1000);
     //     if(FLAG==1)
     //         count_i=0;
     //     else
     //         count_i=1;
     //     FLAG=1;
     // }
      //         Fire_Command();
     //       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,i);
     //       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,i);
     //       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,i);
             if(i>=1400){
                 i=1400  ;
             }
       osDelay(10);
  }
  /* USER CODE END ChassisTask_callback */
}

/* USER CODE BEGIN Header_RemoteTask_callback */
/**
* @brief Function implementing the RomoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteTask_callback */
void RemoteTask_callback(void *argument)
{
  /* USER CODE BEGIN RemoteTask_callback */
  /* Infinite loop */
  PWM_servo_control_init();
  for(;;)
  {

   //  set_servo_angle(servo_2,0.00f);
   //  set_servo_angle(servo_3,0.00f);
   //  set_servo_angle(servo_4,0.00f);
   //  set_servo_angle(servo_5,0.00f);
   //  set_servo_angle(servo_6,0.00f);
   //  set_servo_angle(servo_7,0.00f);
   //  set_servo_angle(servo_8,0.00f);
   //osDelay(2000);
   //  set_servo_angle(servo_2,180.00f);
   //  set_servo_angle(servo_3,180.00f);
   //  set_servo_angle(servo_4,180.00f);
   //  set_servo_angle(servo_5,180.00f);
   //  set_servo_angle(servo_6,180.00f);
   //  set_servo_angle(servo_7,180.00f);
   //  set_servo_angle(servo_8,180.00f);
   //  osDelay(2000);
  }
  /* USER CODE END RemoteTask_callback */
}

/* USER CODE BEGIN Header_Arm_Gimbal_Task_callback */
/**
* @brief Function implementing the Arm_Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Arm_Gimbal_Task_callback */
void Arm_Gimbal_Task_callback(void *argument)
{
  /* USER CODE BEGIN Arm_Gimbal_Task_callback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Arm_Gimbal_Task_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
