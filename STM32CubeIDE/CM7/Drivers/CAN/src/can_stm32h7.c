/*
 * can_stm32h7.c
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 *
 *  STM32H7 CAN Driver Implementation
 *  ==================================
 *  This file provides a CAN (Controller Area Network) driver for the STM32H7
 *  microcontroller family. It uses the STM32 HAL FDCAN peripheral in Classic
 *  CAN mode (not CAN FD) and integrates with FreeRTOS for thread-safe
 *  communication.
 *
 *  Design overview:
 *  - Each CAN bus (FDCAN1, FDCAN2) is managed by a separate context structure.
 *  - Incoming frames are received in an interrupt callback (HAL_FDCAN_RxFifo0Callback)
 *    and placed into a FreeRTOS queue (rxq).
 *  - Outgoing frames are placed into a transmit queue (txq) by application code.
 *    A dedicated FreeRTOS task (can_tx_task) dequeues frames and sends them using
 *    the HAL blocking API.
 *  - The driver exposes a common interface (can_driver_t) via a virtual function
 *    table (vtable) to allow the application to be hardware-agnostic.
 *  - All memory for queues is allocated statically to avoid runtime allocation
 *    failures and to meet real-time requirements.
 */


#include "can_driver.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>


/*
 * ----------------------------------------------------------------------------
 * Constants and Configuration
 * ----------------------------------------------------------------------------
 */
#define MAX_CAN_INTERFACES 2
#define RX_QUEUE_SIZE 32
#define TX_QUEUE_SIZE 16

static const char * const CAN1_TX_TASK_NAME = "can_tx_1";
static const char * const CAN2_TX_TASK_NAME = "can_tx_2";

TaskHandle_t can_tx_handle = NULL;

/*
 * ----------------------------------------------------------------------------
 * Data Structures
 * ----------------------------------------------------------------------------
 */
/**
 * @brief Per‑CAN‑bus context (private to this driver)
 *
 * Each instance holds the HAL handle, the FreeRTOS queues for RX and TX,
 * the static storage areas for those queues, and a flag indicating whether
 * the interface has been initialised and is active.
 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;   /* Pointer to the HAL FDCAN handle (e.g., &hfdcan1) */

    /* Queue handles created with xQueueCreateStatic() */
    QueueHandle_t rxq;             /* Queue for received frames (ISR -> task) */
    QueueHandle_t txq;             /* Queue for frames to be transmitted (application -> TX task) */

    StackType_t  tx_task_stack[1024];
    StaticTask_t tx_task_tcb;
    TaskHandle_t tx_task_handle;


    /* StaticQueue_t structures that hold the internal state of the queues.
     * These must be supplied when creating a queue statically. */
    StaticQueue_t rx_qbuf;
    StaticQueue_t tx_qbuf;

    /* Pre‑allocated memory areas for the queue data.
     * Each queue holds an array of can_frame_t structures. */
    can_frame_t rx_storage[RX_QUEUE_SIZE];
    can_frame_t tx_storage[TX_QUEUE_SIZE];

    /* Flag indicating whether this CAN interface is active and ready to be used. */
    volatile uint8_t is_active;
} h7_ctx_t;



/*
 * ----------------------------------------------------------------------------
 * Global Variables
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Array of context structures for the two CAN peripherals.
 *
 * Index 0 corresponds to FDCAN1, index 1 to FDCAN2.
 * They are zero‑initialised on startup. The `is_active` field is set
 * to 1 when the driver is created for that bus.
 */
static h7_ctx_t g_ctx[MAX_CAN_INTERFACES] = {0};


/*
 * ----------------------------------------------------------------------------
 * Private Helper Functions
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Validate and retrieve the driver's private context pointer.
 *
 * This function checks that the generic driver handle (self) and its
 * context pointer are non‑NULL. It uses configASSERT to catch invalid
 * usage during development.
 *
 * @param self  Pointer to the generic CAN driver interface.
 * @return      Pointer to the h7_ctx_t structure for the selected bus.
 */
static inline h7_ctx_t *validate_ctx(can_driver_t *self)
{
    configASSERT(self);
    configASSERT(self->ctx);

    h7_ctx_t *c = self->ctx;

    return c;
}

/*
 * ----------------------------------------------------------------------------
 * Interrupt Service Routine (Callback)
 * ----------------------------------------------------------------------------
 */

/**
 * @brief HAL FDCAN RX FIFO 0 callback.
 *
 * This function is called by the HAL interrupt handler when a new message
 * has been received into RX FIFO 0. It must determine which CAN peripheral
 * (FDCAN1 or FDCAN2) generated the interrupt, extract the frame, and place
 * it into the corresponding RX queue. Because this runs in an ISR context,
 * it uses the interrupt‑safe version of the queue send function
 * (xQueueSendFromISR) and may request a context switch if a higher‑priority
 * task was waiting for the data.
 *
 * @param hfdcan  HAL handle that triggered the callback.
 * @param flags   Interrupt flags (e.g., FDCAN_IT_RX_FIFO0_NEW_MESSAGE).
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    BaseType_t hpw = pdFALSE;
    h7_ctx_t *ctx = NULL;

    if (hfdcan->Instance == FDCAN1)
        ctx = &g_ctx[0];
    else if (hfdcan->Instance == FDCAN2)
        ctx = &g_ctx[1];

    if (!ctx || !ctx->is_active || !ctx->rxq)
        return;

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) == HAL_OK)
    {
        can_frame_t f;

        f.id = hdr.Identifier;
        f.dlc = (hdr.DataLength >> 16) & 0xF;
        f.extended = (hdr.IdType == FDCAN_EXTENDED_ID);

        memcpy(f.data, data, 8);

        if (xQueueSendFromISR(ctx->rxq, &f, &hpw) != pdPASS)
        {

            /* Make sure debugger stops *exactly here* */
            __disable_irq();

            /* Force visibility in debugger watch window */

            /* Optional: break immediately into debugger */
            __BKPT(0);

        }
    }

    portYIELD_FROM_ISR(hpw);
}


/*
 * ----------------------------------------------------------------------------
 * Transmit Task
 * ----------------------------------------------------------------------------
 */

/**
 * @brief FreeRTOS task responsible for sending CAN frames.
 *
 * This task is created once per CAN interface. It blocks on the transmit
 * queue (txq) and, for each dequeued frame, calls the HAL function to
 * place the message into the hardware TX FIFO. Because the HAL function
 * blocks until the message is queued in the peripheral, the task priority
 * should be chosen to balance real‑time requirements with other tasks.
 *
 * @param arg  Generic pointer cast to can_driver_t for this bus.
 */
void can_tx_task(void *arg)
{
    can_driver_t *drv = arg;
    h7_ctx_t *c = validate_ctx(drv);


    configASSERT(c && c->is_active && c->txq);

    can_frame_t f;

    for (;;)
    {
        // 1. Block indefinitely until application code queues a frame.
        // Zero CPU overhead while waiting.
        if (xQueueReceive(c->txq, &f, portMAX_DELAY) == pdPASS)
        {
            FDCAN_TxHeaderTypeDef hdr = {0};

            hdr.Identifier = f.id;
            hdr.IdType = f.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
            // Note: Ensure your data length conversion is correct for Classic CAN (0-8)
            hdr.DataLength = f.dlc << 16;

            // 2. Directly attempt to push to the hardware FIFO.
            // No polling, no delays.
            HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(c->hfdcan, &hdr, f.data);

            configASSERT(st == HAL_OK);
        }
        // 3. Removed the trailing vTaskDelay(1). The loop immediately
        // re-blocks on xQueueReceive if no more items are present.
    }
}

/*
 * ----------------------------------------------------------------------------
 * Driver Interface Implementation (Virtual Functions)
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Initialise the CAN driver's FreeRTOS resources.
 *
 * This function creates the static receive and transmit queues.
 * It does not configure or start the CAN hardware – that is done in
 * h7_start().
 *
 * @param self  Driver instance.
 * @return      CAN_OK on success.
 */
static can_status_t h7_init(can_driver_t *self)
{
    h7_ctx_t *c = validate_ctx(self);

    /* Create the RX queue using pre‑allocated memory */
    c->rxq = xQueueCreateStatic(
		RX_QUEUE_SIZE,
        sizeof(can_frame_t),
        (uint8_t *)c->rx_storage,
        &c->rx_qbuf
    );

    /* Create the TX queue using pre‑allocated memory */
    c->txq = xQueueCreateStatic(
		TX_QUEUE_SIZE,
        sizeof(can_frame_t),
        (uint8_t *)c->tx_storage,
        &c->tx_qbuf
    );

    configASSERT(c && c->is_active && c->rxq && c->txq);

    configASSERT(uxQueueSpacesAvailable(c->rxq) == RX_QUEUE_SIZE);
    configASSERT(uxQueueSpacesAvailable(c->txq) == TX_QUEUE_SIZE);

    return CAN_OK;
}

/**
 * @brief Start the CAN hardware and enable interrupt notifications.
 *
 * After calling h7_init(), this function starts the FDCAN peripheral
 * and activates the RX FIFO 0 new‑message interrupt. It also creates
 * the transmit task.
 *
 * @param self  Driver instance.
 * @return      CAN_OK if successful, CAN_ERR otherwise.
 */
static can_status_t h7_start(can_driver_t *self)
{
    h7_ctx_t *c = validate_ctx(self);

    configASSERT(c && c->is_active && c->rxq && c->txq);

    if (HAL_FDCAN_Start(c->hfdcan) != HAL_OK)
        return CAN_ERR;

    if (HAL_FDCAN_ActivateNotification(c->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        return CAN_ERR;

    // Dynamically choose name based on which instance context this is
    const char *task_name = (c->hfdcan->Instance == FDCAN1) ? CAN1_TX_TASK_NAME : CAN2_TX_TASK_NAME;


    // 2. Create the task statically (replace xTaskCreate with this)
    c->tx_task_handle = xTaskCreateStatic(
        can_tx_task,
        task_name,
        sizeof(c->tx_task_stack) / sizeof(StackType_t),
        self,
        tskIDLE_PRIORITY + 5,
        c->tx_task_stack,   // now in ctx — permanent lifetime
        &c->tx_task_tcb     // now in ctx — permanent lifetime
    );
    configASSERT(c->tx_task_handle);  // Verify creation succeeded

    return CAN_OK;
}

/**
 * @brief Queue a CAN frame for transmission (non‑blocking or with timeout).
 *
 * Places the frame into the transmit queue. The dedicated TX task will
 * actually send it to the hardware.
 *
 * @param self    Driver instance.
 * @param f       Pointer to the frame to send.
 * @param timeout Maximum time to wait in milliseconds. Use 0 for non‑blocking
 *                or portMAX_DELAY for indefinite wait.
 * @return        CAN_OK on success, CAN_TIMEOUT if the queue is full for the
 *                specified time.
 */
static can_status_t h7_send(can_driver_t *self,
                            const can_frame_t *f,
                            uint32_t timeout)
{
	h7_ctx_t *c = validate_ctx(self);

	configASSERT(c && c->is_active && c->rxq && c->txq);

	configASSERT(f);

    if (xQueueSend(c->txq, f, pdMS_TO_TICKS(timeout)) != pdPASS)
        return CAN_TIMEOUT;

    return CAN_OK;
}

/**
 * @brief Receive a CAN frame (blocking with timeout).
 *
 * Waits for a frame to arrive in the receive queue. The queue is filled
 * by the ISR callback.
 *
 * @param self    Driver instance.
 * @param f       Pointer to storage for the received frame.
 * @param timeout Maximum wait time in milliseconds.
 * @return        CAN_OK on success, CAN_TIMEOUT if no frame arrived within
 *                the timeout.
 */
static can_status_t h7_receive(can_driver_t *self,
                               can_frame_t *f,
                               uint32_t timeout)
{
	h7_ctx_t *c = validate_ctx(self);

	configASSERT(c->rxq);
	configASSERT(c->txq);

	configASSERT(f);

	/* Fill the output buffer with a known pattern (0xEE) before waiting.
     * This helps debugging in case the caller does not check the return value. */
	memset(f, 0xEE, sizeof(*f));

//	xQueueReceive(c->rxq, f, pdMS_TO_TICKS(timeout));

    if (xQueueReceive(c->rxq, f, pdMS_TO_TICKS(timeout)) != pdPASS)
        return CAN_TIMEOUT;

    return CAN_OK;
}

/*
 * ----------------------------------------------------------------------------
 * Virtual Function Table (VTable)
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Constant table of function pointers implementing the CAN driver
 *        operations.
 *
 * This vtable is shared by all instances of the H7 driver. Each instance's
 * can_driver_t contains a pointer to this same vtable, enabling the
 * application to call driver functions without knowing the underlying
 * hardware details.
 */
static const can_vtable_t vtbl =
{
    .init    = h7_init,
    .start   = h7_start,
    .send    = h7_send,
    .receive = h7_receive
};

/*
 * ----------------------------------------------------------------------------
 * Public Function: Driver Factory
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Create and return a CAN driver instance for the given bus number.
 *
 * This is the only public function exposed by this file. It initialises
 * the appropriate context structure (h7_ctx_t) and returns a can_driver_t
 * that binds the virtual table and the context.
 *
 * @param can_bus_number  Bus index: 1 for FDCAN1, 2 for FDCAN2.
 * @return                A can_driver_t structure. If the bus number is
 *                        invalid, the vptr is set to NULL.
 */
can_driver_t can_stm32h7_create(uint8_t can_bus_number)
{

	configASSERT(can_bus_number >= 1);
	configASSERT(can_bus_number <= MAX_CAN_INTERFACES);

    uint8_t idx = can_bus_number - 1;

    if (idx >= MAX_CAN_INTERFACES) {
        return (can_driver_t){ .vptr = NULL, .ctx = NULL };
    }

    g_ctx[idx].hfdcan = (can_bus_number == 1) ? &hfdcan1 : &hfdcan2;
    g_ctx[idx].is_active = 1;

    return (can_driver_t){
        .vptr = &vtbl,
        .ctx  = &g_ctx[idx]
    };
}
