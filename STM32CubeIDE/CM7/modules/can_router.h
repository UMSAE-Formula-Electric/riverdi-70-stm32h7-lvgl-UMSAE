/*
 * can_router.h
 *
 *  Created on: May 16, 2026
 *      Author: mason
 */

#ifndef MODULES_CAN_ROUTER_H_
#define MODULES_CAN_ROUTER_H_

#include "can_driver.h"

void CAN_router(const can_frame_t *frame);

#endif /* MODULES_CAN_ROUTER_H_ */
