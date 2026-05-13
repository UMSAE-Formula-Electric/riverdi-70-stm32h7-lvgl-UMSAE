/*
 * can_stm32h7.c
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */


#include "can_driver.h"
#include "fdcan.h"     // HAL include ONLY here
#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;

    QueueHandle_t rxq;
    QueueHandle_t txq;

} h7_ctx_t;


static h7_ctx_t *g_ctx = NULL;   // set during create()

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t flags)
{
    BaseType_t hpw = pdFALSE;

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

           xQueueSendFromISR(g_ctx->rxq, &f, &hpw);

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
            hdr.DataLength = f.dlc << 16;

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

    g_ctx = c;

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

    xTaskCreate(can_tx_task, "can_tx", 512, self, 3, NULL);
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


can_driver_t can_stm32h7_create(void)
{
    static h7_ctx_t ctx;

    ctx.hfdcan = &hfdcan1;

    return (can_driver_t){
        .vptr = &vtbl,
        .ctx  = &ctx
    };
}
