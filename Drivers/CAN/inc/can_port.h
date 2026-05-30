/*
 * can_port.h
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */

#ifndef DRIVERS_CAN_INC_CAN_PORT_H_
#define DRIVERS_CAN_INC_CAN_PORT_H_

#include "can_driver.h"

can_driver_t *can_port_create(uint8_t can_bus_number);

#endif /* DRIVERS_CAN_INC_CAN_PORT_H_ */
