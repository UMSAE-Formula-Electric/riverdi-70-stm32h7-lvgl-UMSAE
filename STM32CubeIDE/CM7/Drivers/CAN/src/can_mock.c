/*
 * can_mock.c
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */
#include "can_driver.h"

typedef struct
{
    can_frame_t last_tx;
} mock_ctx_t;

static can_status_t mock_send(can_driver_t *d,
                              const can_frame_t *f,
                              uint32_t t)
{
    mock_ctx_t *c = d->ctx;
    c->last_tx = *f;
    return CAN_OK;
}

static can_status_t mock_receive(can_driver_t *d,
                                 can_frame_t *f,
                                 uint32_t t)
{
    return CAN_TIMEOUT;
}
