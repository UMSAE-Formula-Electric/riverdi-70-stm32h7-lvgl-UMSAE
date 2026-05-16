/*
 * can_stm32h7.c
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */


#include "can_driver.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

#define MAX_CAN_INTERFACES 2

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    QueueHandle_t rxq;
    QueueHandle_t txq;
    uint8_t is_active;
} h7_ctx_t;

static h7_ctx_t g_ctx[MAX_CAN_INTERFACES] = {0};

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    BaseType_t hpw = pdFALSE;
    h7_ctx_t *ctx = NULL;

    if (hfdcan->Instance == FDCAN1) {
        ctx = &g_ctx[0];
    } else if (hfdcan->Instance == FDCAN2) {
        ctx = &g_ctx[1];
    }

    if (ctx == NULL || !ctx->is_active) return;

    if (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef hdr;
        uint8_t data[8];

       if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) == HAL_OK){
           can_frame_t f;

           f.id = hdr.Identifier;
           f.dlc = (hdr.DataLength >> 16) & 0xF;
           f.extended = (hdr.IdType == FDCAN_EXTENDED_ID);

           memcpy(f.data, data, 8);

           xQueueSendFromISR(ctx->rxq, &f, &hpw);
       }
    }

    portYIELD_FROM_ISR(hpw);
}

static void can_tx_task(void *arg)
{
    can_driver_t *drv = arg;
    h7_ctx_t *c = drv->ctx;

    can_frame_t f;

    for (;;)
    {
        if (xQueueReceive(c->txq, &f, portMAX_DELAY))
        {
            FDCAN_TxHeaderTypeDef hdr = {0};

            hdr.Identifier = f.id;
            hdr.IdType = f.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
            hdr.TxFrameType = FDCAN_DATA_FRAME;
            hdr.DataLength = f.dlc << 16;
            hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            hdr.BitRateSwitch = FDCAN_BRS_OFF;
            hdr.FDFormat = FDCAN_CLASSIC_CAN;
            hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            hdr.MessageMarker = 0;

            HAL_FDCAN_AddMessageToTxFifoQ(c->hfdcan, &hdr, f.data);
        }
    }
}

static can_status_t h7_init(can_driver_t *self)
{
    h7_ctx_t *c = self->ctx;

    c->rxq = xQueueCreate(32, sizeof(can_frame_t));
    c->txq = xQueueCreate(32, sizeof(can_frame_t));

    configASSERT(c->rxq);
    configASSERT(c->txq);

    return CAN_OK;
}

static can_status_t h7_start(can_driver_t *self)
{
    h7_ctx_t *c = self->ctx;

    if (HAL_FDCAN_Start(c->hfdcan) != HAL_OK)
        return CAN_ERR;

    if (HAL_FDCAN_ActivateNotification(
            c->hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0) != HAL_OK)
        return CAN_ERR;

    char task_name[16];
    if (c->hfdcan->Instance == FDCAN1) {
        snprintf(task_name, sizeof(task_name), "can_tx_1");
    } else {
        snprintf(task_name, sizeof(task_name), "can_tx_2");
    }

    xTaskCreate(can_tx_task, task_name, 512, self, 3, NULL);
    return CAN_OK;
}

static can_status_t h7_send(can_driver_t *self,
                            const can_frame_t *f,
                            uint32_t timeout)
{
    h7_ctx_t *c = self->ctx;

    if (xQueueSend(c->txq, f, pdMS_TO_TICKS(timeout)) != pdPASS)
        return CAN_TIMEOUT;

    return CAN_OK;
}

static can_status_t h7_receive(can_driver_t *self,
                               can_frame_t *f,
                               uint32_t timeout)
{
    h7_ctx_t *c = self->ctx;

    if (xQueueReceive(c->rxq, f, pdMS_TO_TICKS(timeout)) != pdPASS)
        return CAN_TIMEOUT;

    return CAN_OK;
}

/* vtable */
static const can_vtable_t vtbl =
{
    .init    = h7_init,
    .start   = h7_start,
    .send    = h7_send,
    .receive = h7_receive
};

can_driver_t can_stm32h7_create(uint8_t can_bus_number)
{
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
