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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
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
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
uint32_t mainTaskBuffer[ 128 ];
osStaticThreadDef_t mainTaskControlBlock;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_mem = &mainTaskBuffer[0],
  .stack_size = sizeof(mainTaskBuffer),
  .cb_mem = &mainTaskControlBlock,
  .cb_size = sizeof(mainTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpuTask */
osThreadId_t mpuTaskHandle;
uint32_t mpuTaskBuffer[ 128 ];
osStaticThreadDef_t mpuTaskControlBlock;
const osThreadAttr_t mpuTask_attributes = {
  .name = "mpuTask",
  .stack_mem = &mpuTaskBuffer[0],
  .stack_size = sizeof(mpuTaskBuffer),
  .cb_mem = &mpuTaskControlBlock,
  .cb_size = sizeof(mpuTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rxQueue */
osMessageQueueId_t rxQueueHandle;
uint8_t rxQueueBuffer[ 5 * sizeof( uint8_t ) ];
osStaticMessageQDef_t rxQueueControlBlock;
const osMessageQueueAttr_t rxQueue_attributes = {
  .name = "rxQueue",
  .cb_mem = &rxQueueControlBlock,
  .cb_size = sizeof(rxQueueControlBlock),
  .mq_mem = &rxQueueBuffer,
  .mq_size = sizeof(rxQueueBuffer)
};
/* Definitions for txQueue */
osMessageQueueId_t txQueueHandle;
uint8_t myQueue02Buffer[ 13 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue02ControlBlock;
const osMessageQueueAttr_t txQueue_attributes = {
  .name = "txQueue",
  .cb_mem = &myQueue02ControlBlock,
  .cb_size = sizeof(myQueue02ControlBlock),
  .mq_mem = &myQueue02Buffer,
  .mq_size = sizeof(myQueue02Buffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMpuTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of rxQueue */
  rxQueueHandle = osMessageQueueNew (5, sizeof(uint8_t), &rxQueue_attributes);

  /* creation of txQueue */
  txQueueHandle = osMessageQueueNew (13, sizeof(uint16_t), &txQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartDefaultTask, NULL, &mainTask_attributes);

  /* creation of mpuTask */
  mpuTaskHandle = osThreadNew(StartMpuTask, NULL, &mpuTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

	  // leggo i comandi da seriale

	  // eseguo i comandi (avvio o fermo l'acquisizione dall'mpu)
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMpuTask */
/**
* @brief Function implementing the mpuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMpuTask */
void StartMpuTask(void *argument)
{
  /* USER CODE BEGIN StartMpuTask */
  /* Infinite loop */
  for(;;)
  {
	  // se sono qua è perché è scattato l'interrupt dell'mpu
	  // acquisisco i dati dall'mpu
	  // invio i dati alla seriale
  }
  /* USER CODE END StartMpuTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
