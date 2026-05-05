/*
 * can_utils.c
 *
 *  Created on: Apr 23, 2024
 *      Author: tonyz
 */

#include <stdint.h>
#include "can_utils.h"
#include "can_driver.h"
#include "logger.h"

static can_driver_t *g_can_drv = NULL;

void can_util_init(can_driver_t *drv){
	g_can_drv = drv;
}

/**
  * @brief  send a can message, delays until sent confirmed.
  * @param  hcan: where x can be 1 or 2 to select the CAN peripheral.
  * @param  data: bytes of data to send, max length 8.
  * @param	length: length of data to send. (Length is 1 indexed for some reason, 7 means 8), likely a bug
  * @param 	dest: destination ID ??? Austin sucks, don't understand CAN at all
  * @param	canRTR: is request for transmission, 1 for request, 0 for data
  * @param	isExtended: is the ID and extended address, 0 for standard, 1 for extended
  * @retval 0 on success, 1 if timeout, 2 hcan not init, 3 length too long
  */
uint8_t sendCan(uint32_t dest, const uint8_t *data, uint32_t length, uint32_t canRTR, uint8_t isExtended) {
    if (g_can_drv == NULL) return 2; // Driver not initialized
    if (length > 8) return 3;        // Length too long

    can_frame_t frame;
    frame.id = dest;
    frame.dlc = length;
    frame.extended = isExtended;

    // Safety: clear and copy data
    memset(frame.data, 0, 8);
    memcpy(frame.data, data, length);

    // can_send handles the queueing/threading internally in can_stm32h7.c
    can_status_t status = can_send(g_can_drv, &frame, 100);

    if (status == CAN_TIMEOUT) {
        logMessage("Error: CAN Send Timeout\r\n", false);
        return 1;
    } else if (status != CAN_OK) {
        logMessage("Error: CAN Driver Failure\r\n", false);
        return 4;
    }

    return 0; // Success
}
