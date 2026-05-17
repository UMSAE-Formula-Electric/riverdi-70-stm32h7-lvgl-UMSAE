/**
 * @file can_router.h
 * @brief CAN message routing interface.
 * @details
 * Provides the interface for routing received CAN frames to the
 * appropriate subsystem or message handler based on the CAN identifier
 * and message contents.
 *
 * The router acts as a central dispatch layer between the CAN driver
 * and higher-level application modules.
 */

#ifndef MODULES_CAN_ROUTER_H_
#define MODULES_CAN_ROUTER_H_

#include "can_driver.h"

/**
 * @brief Routes a received CAN frame to the appropriate handler.
 * @details
 * Processes an incoming CAN frame and dispatches it to the correct
 * module, state machine, or message parser based on the frame ID
 * and associated payload data.
 *
 * This function is intended to be called whenever a new CAN message
 * is received from the CAN driver layer.
 *
 * @param[in] frame Pointer to the received CAN frame.
 */
void CAN_router(const can_frame_t *frame);

#endif /* MODULES_CAN_ROUTER_H_ */
