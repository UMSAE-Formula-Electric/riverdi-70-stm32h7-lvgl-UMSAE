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
#include "lvgl_port_touch.h"
#include "lvgl_port_display.h"
#include "ui.h"
#include "ui_events.h"
#include "can_driver.h"
#include "can_port.h"
#include "motor_controller_can_utils.h"
#include "bms_can_utils.h"
#include "can_utils.h"
#include "dashboard_ui.h"
#include "can_router.h"
#include "vehicle_state.h"
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

static can_driver_t *g_can1;
static can_driver_t *g_can2;

/*
 *	Queue Definitions
 */

/*
 *	OS thread Definitions
 */
osThreadId_t lvglTimerHandle;
const osThreadAttr_t lvglTimer_attributes = {
  .name = "lvglTimer",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 8 * 1024
};
/* Definitions for canRxTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t vehicleStateTaskHandle;
const osThreadAttr_t vehicleStateTask_attributes = {
    .name = "vehicleStateTask",
    .stack_size = 512 * 2,
    .priority = (osPriority_t) osPriorityBelowNormal,
};

/*
 *  CAN 2 OS Thread Definitions
 */
osThreadId_t can2TaskHandle;
const osThreadAttr_t can2Task_attributes = {
  .name = "can2Task",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityHigh,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LVGLTimer(void *argument);
void StartCANTask(void *argument);
void StartVehicleStateTask(void *argument);
void StartCAN2Task(void *argument);

static void ui_update_cb(lv_timer_t *timer)
{
    /* Read vehicle state – ensure thread-safe access (e.g., via a mutex) */
    uint16_t speed   = VehicleState_GetSpeed();
    uint16_t rpm     = VehicleState_GetRPM();
    uint8_t  brake   = VehicleState_GetBrake();
    uint16_t power   = VehicleState_GetWattage();
    uint8_t  throttle= VehicleState_GetThrottle();
    uint8_t  soc     = VehicleState_GetSOC();
    enum CAR_STATE state = VehicleState_GetState();

    /* Now it's safe to call LVGL API without explicit locks */
    DashboardUI_SetSpeed(speed);
    DashboardUI_SetRPM(rpm);
    DashboardUI_SetBrake(brake);
    DashboardUI_SetPower(power);
    DashboardUI_SetThrottle(throttle);
    DashboardUI_SetSOC(soc);
    DashboardUI_SetVehicleState(state);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); 

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

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
	//TODO log potential gains from this
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
    (void)xTask;

    /* Make sure debugger stops *exactly here* */
    __disable_irq();

    /* Force visibility in debugger watch window */
    volatile const char *task_name __attribute__((unused)) = pcTaskName;

    /* Optional: store fault info in global variables */
    static volatile const char *fault_task_name __attribute__((unused)) = 0;
    static volatile TaskHandle_t fault_task_handle __attribute__((unused)) = 0;

    fault_task_name = pcTaskName;
    fault_task_handle = xTask;

    (void)fault_task_name;
    (void)fault_task_handle;
    (void)task_name;

    /* Optional: break immediately into debugger */
    __BKPT(0);

    /* Optional: trap CPU if debugger is not attached */
    while (1)
    {
        __NOP();
    }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	g_can1 = can_port_create(1);
	if (g_can1 != NULL) {
		can_init(g_can1);
		can_start(g_can1);
	}

	g_can2 = can_port_create(2);
	if (g_can2 != NULL) {
		can_init(g_can2);
		can_start(g_can2);
	}

	UIEvents_Init();

	UIEvents_Init();

    /* Create LVGL timer to update dashboard UI */
    lv_timer_create(ui_update_cb, 50, NULL);
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
#ifdef DEBUG
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
#endif
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	lvglTimerHandle = osThreadNew(LVGLTimer, NULL, &lvglTimer_attributes);
	canTaskHandle = osThreadNew(StartCANTask, NULL, &canTask_attributes);
	vehicleStateTaskHandle = osThreadNew(StartVehicleStateTask, NULL, &vehicleStateTask_attributes);
    can2TaskHandle = osThreadNew(StartCAN2Task, NULL, &can2Task_attributes);

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
<<<<<<< HEAD
    UIEvents_PostTest("Default Task running!", sizeof("Default Task running!"));
=======
    UIEvents_PostTest("Default Task running!");
>>>>>>> 5581c79 (Created new UI toast notifications)
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
	  if (can_send(g_can1, &tx_frame, 100) == CAN_OK) {
		  // Frame sent successfully
		  //TODO Add logging here for success
	  } else {
		  // Handle send error (optional)
		  //TODO Add Logging here for failure
	  }

	  vTaskDelay(pdMS_TO_TICKS(100));  // Send every 100ms (10Hz)
  }
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* LVGL timer for tasks */
/**
 * @brief LVGL GUI update and tick handler task.
 * @details
 * Periodic FreeRTOS task responsible for:
 * - Pulling the latest values from the @ref vehicle_state module
 * - Updating dashboard UI elements (speed, RPM, SOC, power, etc.)
 * - Executing the LVGL internal task handler to process GUI timers,
 *   animations, and redraw events
 *
 * This task effectively bridges the application state layer and the
 * LVGL rendering system.
 *
 * @note Runs at a fixed 20 ms period (50 Hz) to maintain UI responsiveness.
 * @warning LVGL is not inherently thread-safe; all LVGL API calls should
 * be confined to this task or properly synchronized.
 *
 * @param[in] argument Unused task parameter.
 *
 * @retval None
 */
void LVGLTimer(void *argument)
{
	UIEvent_t evt;
    for (;;)
    {
        while (xQueueReceive(g_ui_event_queue, &evt, 0) == pdPASS) {
            UI_HandleEvent(&evt);
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Primary CAN bus listener task.
 * @details
 * Initializes the vehicle state subsystem and continuously waits
 * for incoming CAN frames on the CAN1 interface.
 *
 * Received frames are forwarded to the central CAN router for
 * message dispatch and subsystem processing.
 *
 * The task blocks indefinitely while waiting for CAN traffic and
 * yields periodically to the scheduler.
 *
 * @param[in] argument Unused task parameter.
 *
 * @retval None
 */
void StartCANTask(void *argument)
{
    can_frame_t frame;
    for (;;)
    {
        if (can_receive(g_can1, &frame, 15) == CAN_OK)
        {
        	CAN_router(&frame);

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Secondary CAN bus listener task.
 * @details
 * Continuously waits for incoming CAN frames on the CAN2 interface.
 * When a valid frame is received, the frame is forwarded to the
 * central CAN router for dispatch and processing.
 *
 * The task blocks indefinitely while waiting for CAN traffic and
 * yields periodically to the scheduler.nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn
 *
 * @param[in] argument Unused task parameter.
 *
 * @retval None
 */
void StartCAN2Task(void *argument)
{
    can_frame_t frame;

    for (;;)
    {
        if (can_receive(g_can2, &frame, 15) == CAN_OK)
        {
            CAN_router(&frame);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Vehicle state aggregation and computation task.
 * @details
 * Periodically samples subsystem-level telemetry (motor controller,
 * battery management system, and optional driver inputs) and produces
 * a consolidated vehicle state representation stored in the global
 * @ref vehicle_state module.
 *
 * Current responsibilities include:
 * - Converting motor RPM to vehicle speed
 * - Estimating instantaneous electrical power (V × I)
 * - Updating pack voltage, current, and state-of-charge (SOC)
 * - Tracking cell-level extrema (min/max voltage)
 *
 * @note This task currently performs direct polling of subsystem APIs
 * rather than consuming structured CAN messages from a queue. This
 * design may become difficult to maintain as additional signals are added.
 *
 * @warning The computation and aggregation logic is tightly coupled;
 * consider refactoring into a message-driven or staged processing
 * pipeline as system complexity increases.
 *
 * @param[in] argument Unused task parameter.
 *
 * @retval None
 */
void StartVehicleStateTask(void *argument)
{

    VehicleState_Init();


    //TODO This task will get out of hand quickly. The approach needs to change otherwise this will become difficult to maintain.
    for (;;)
    {
        uint16_t rpm = (uint16_t)mc_get_motor_RPM();
        uint16_t speed = (uint16_t)(rpm * 0.075861f);   // already computed helper

        VehicleState_SetRPM(rpm);
        VehicleState_SetSpeed(speed);

        // Power estimation (if you want crude instantaneous power)
        uint16_t v = mc_getBusVoltage();
        uint16_t i = mc_getBusCurrent();
        uint16_t power = v * i;

        VehicleState_SetWattage(power);
        VehicleState_SetPower(power); // optional scaling


        /* =========================
         * BMS DATA
         * ========================= */
        uint16_t packV = (uint16_t)(bms_getAverageVoltage() * 100); // scale consistency
        int packI = (int)(get_bms_current()); // signed current but stored uint in your API

        uint8_t soc = get_bms_estimated_state_of_charge();

        VehicleState_SetPackVoltage(packV);
        VehicleState_SetPackCurrent((uint16_t)packI);
        VehicleState_SetSOC(soc);

        VehicleState_SetCellMax(get_bms_high_voltage());
        VehicleState_SetCellMin(get_bms_low_voltage());


        /* =========================
         * DRIVER INPUTS (if available elsewhere)
         * ========================= */
        // VehicleState_SetThrottle(apps_getThrottle());
        // VehicleState_SetBrake(brake_getValue());


        /* =========================
         * SYSTEM STATE (optional)
         * ========================= */
        VehicleState_SetState(get_car_state());


        /* =========================
         * LOOP TIMING
         * ========================= */
        vTaskDelay(pdMS_TO_TICKS(20));

    }
}


/* USER CODE END Application */

