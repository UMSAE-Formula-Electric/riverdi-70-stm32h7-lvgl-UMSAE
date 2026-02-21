/*
 * can_driver.h
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */

#ifndef DRIVERS_CAN_INC_CAN_DRIVER_H_
#define DRIVERS_CAN_INC_CAN_DRIVER_H_

#include "can_types.h"

typedef struct can_driver can_driver_t;

typedef struct
{
	/**
	 * Initializes the CAN driver.
	 *
	 * This function performs any necessary initialization of the CAN hardware
	 * and driver internal state. It should be called before any other driver
	 * functions.
	 *
	 * @param self Pointer to the CAN driver instance
	 * @return Status code indicating success or failure of the initialization
	 */
	can_status_t (*init)(can_driver_t *self);

	/**
	 * Starts the CAN communication.
	 *
	 * This function enables the CAN controller, allowing it to participate
	 * in bus communication. After successful start, the driver can send and
	 * receive CAN frames.
	 *
	 * @param self Pointer to the CAN driver instance
	 * @return Status code indicating success or failure of the start operation
	 */
	can_status_t (*start)(can_driver_t *self);

	/**
	 * Transmits a CAN frame on the bus.
	 *
	 * This function sends a CAN frame and waits for the transmission to complete
	 * or until the specified timeout expires.
	 *
	 * @param self Pointer to the CAN driver instance
	 * @param frame Pointer to the CAN frame to be transmitted
	 * @param timeout_ms Maximum time to wait for transmission completion in milliseconds
	 * @return Status code indicating success, timeout, or failure of the transmission
	 */
	can_status_t (*send)(can_driver_t *self,
	                     const can_frame_t *frame,
	                     uint32_t timeout_ms);

	/**
	 * Receives a CAN frame from the bus.
	 *
	 * This function waits for a CAN frame to be received or until the specified
	 * timeout expires. The received frame data is copied into the provided frame
	 * structure.
	 *
	 * @param self Pointer to the CAN driver instance
	 * @param frame Pointer to the structure where the received frame will be stored
	 * @param timeout_ms Maximum time to wait for reception in milliseconds
	 * @return Status code indicating success, timeout, or failure of the reception
	 */
	can_status_t (*receive)(can_driver_t *self,
	                        can_frame_t *frame,
	                        uint32_t timeout_ms);

} can_vtable_t;


struct can_driver
{
    const can_vtable_t *vptr;
    void *ctx;     // driver private state
};


/* helper wrappers (nicer to call) */

static inline can_status_t can_init(can_driver_t *d)
{ return d->vptr->init(d); }


static inline can_status_t can_start(can_driver_t *d)
{ return d->vptr->start(d); }

static inline can_status_t can_send(can_driver_t *d,
                                    const can_frame_t *f,
                                    uint32_t t)
{ return d->vptr->send(d, f, t); }

static inline can_status_t can_receive(can_driver_t *d,
                                       can_frame_t *f,
                                       uint32_t t)
{ return d->vptr->receive(d, f, t); }

#endif /* DRIVERS_CAN_INC_CAN_DRIVER_H_ */
