/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include "lvgl/lvgl.h"
#include "ui.h"
#include "can_driver.h"
#include "can_port.h"
#include "vehicle_state.h"
#include "motor_controller_can_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	FDCAN_RxHeaderTypeDef rxPacketHeader;
    uint8_t rxPacketData[8];
} CAN_RxPacketTypeDef;

typedef struct {
    FDCAN_TxHeaderTypeDef txPacketHeader;
    uint8_t txPacketData[8];
} CAN_TxPacketTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static can_driver_t *g_can;

/*
 *	Queue Definitions
 */
osMessageQueueId_t canQueueHandle;
const osMessageQueueAttr_t canQueueHandle_attributes = {
		.name = "canQueue"
};

/*
 *	OS thread Definitions
 */
osThreadId_t lvglTimerHandle;
const osThreadAttr_t lvglTimer_attributes = {
  .name = "lvglTimer",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 4 * 1024
};
/* Definitions for canRxTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t vehicleStateTaskHandle;
const osThreadAttr_t vehicleStateTask_attributes = {
    .name = "vehicleStateTask",
    .stack_size = 512 * 2,
    .priority = (osPriority_t) osPriorityAboveNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LVGLTimer(void *argument);
void StartCANTask(void *argument);
void StartVehicleStateTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	g_can = can_port_create();

	can_init(g_can);
	can_start(g_can);
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
	canQueueHandle = osMessageQueueNew(10,sizeof(can_frame_t),NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	lvglTimerHandle = osThreadNew(LVGLTimer, NULL, &lvglTimer_attributes);
	canTaskHandle = osThreadNew(StartCANTask, NULL, &canTask_attributes);
	vehicleStateTaskHandle = osThreadNew(StartVehicleStateTask, NULL, &vehicleStateTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Task responsible for simulating vehicle telemetry.
 * @details Generates mock motor data (angle, speed, frequency) and broadcasts
 * it over the CAN bus at a fixed interval of 10Hz.
 * @param  argument: Unused.
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	  /* USER CODE BEGIN StartDefaultTask */
	uint16_t speed = 0;
	uint16_t angle = 0;
	can_frame_t tx_frame;

	// Configure CAN frame
	tx_frame.id = 0x0A5;        // Set appropriate CAN ID
	tx_frame.dlc = 8;            // 8 bytes of data
	tx_frame.extended = false;    // Use standard 11-bit ID

  /* Infinite loop */
  for(;;)
  {
	  // Generate simulated motor data
	  speed = (speed + 5) % 1001;    // 0-1000 RPM
	  angle = (angle + 10) % 3600;    // 0-3599 (0.1 degree resolution)

	  // Fill CAN data bytes according to the motor data format
	  // Bytes 0-1: Motor angle
	  tx_frame.data[0] = (angle >> 8) & 0xFF;    // High byte
	  tx_frame.data[1] = angle & 0xFF;            // Low byte

	  // Bytes 2-3: Motor Speed
	  tx_frame.data[2] = speed & 0xFF;     // High byte
	  tx_frame.data[3] = (speed >> 8) & 0xFF;             // Low byte

	  // Bytes 4-5: Electrical Output Frequency
	  uint16_t frequency = (speed * 10) / 7;      // Example calculation
	  tx_frame.data[4] = (frequency >> 8) & 0xFF;
	  tx_frame.data[5] = frequency & 0xFF;


	  // Bytes 6-7: Delta Resolver Filtered
	  uint16_t delta_resolver = angle / 10;        // Example calculation
	  tx_frame.data[6] = (delta_resolver >> 8) & 0xFF;
	  tx_frame.data[7] = delta_resolver & 0xFF;

	  // Send the CAN frame
	  if (can_send(g_can, &tx_frame, 100) == CAN_OK) {
		  // Frame sent successfully
		  //TODO Add logging here for success
	  } else {
		  // Handle send error (optional)
		  //TODO Add Logging here for failure
	  }

	  osDelay(100);  // Send every 100ms (10Hz)
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* LVGL timer for tasks */
/**
 * @brief  Task handler for the LVGL graphics library.
 * @details Periodic task that updates UI labels with the latest vehicle state
 * (Speed and RPM) and executes the LVGL internal timer handler.
 * @note   Runs with a 20ms period (50Hz).
 * @param  argument: Unused.
 * @retval None
 */
void LVGLTimer(void *argument)
{
    for (;;)
    {
        lv_label_set_text_fmt(
            ui_Speed,
            "%2d km/h \t %2d RPM",
            VehicleState_GetSpeed(),
            VehicleState_GetRPM()
        );

        lv_timer_handler();
        osDelay(20);
    }
}

/**
 * @brief  Primary CAN bus listener task.
 * @details Blocks until a CAN frame is received via the hardware driver,
 * then pushes the frame into the @ref canQueueHandle for processing.
 * @param  argument: Unused.
 * @retval None
 */
void StartCANTask(void *argument)
{
    can_frame_t frame;

    for (;;)
    {
        if (can_receive(g_can, &frame, portMAX_DELAY) == CAN_OK)
        {
        	osMessageQueuePut(canQueueHandle, &frame,0,0);
        }
        osDelay(1);
    }
}

/**
 * @brief  Task for processing raw CAN data into vehicle state.
 * @details Consumes messages from @ref canQueueHandle. It parses motor controller
 * frames (ID: 0x0A5), calculates vehicle speed based on RPM, and updates
 * the global @ref vehicle_state module.
 * @param  argument: Unused.
 * @retval None
 */
void StartVehicleStateTask(void *argument)
{
    can_frame_t frame;
    uint16_t rpm;
    uint8_t speed;

    VehicleState_Init();

    //TODO Add more handlers for messages
    for (;;)
    {
        if (osMessageQueueGet(canQueueHandle, &frame, NULL, portMAX_DELAY) == osOK)
        {
            switch (frame.id)
            {
                case CAN_MC_RX_MOTOR_ID:
                    mc_process_motor_can(frame.data);

                    rpm = mc_get_motor_RPM();
                    VehicleState_SetRPM((uint8_t)rpm);

                    speed = (uint8_t)(rpm * 0.075861f);
                    VehicleState_SetSpeed(speed);
                    break;
                default:
                    break;
            }
        }
    }
}
/* USER CODE END Application */

