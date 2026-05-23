/*
 * freertos_heap.c
 *
 *  Created on: May 18, 2026
 *      Author: mason
 */
#include "FreeRTOS.h"

/* Place FreeRTOS heap into D2 SRAM */

uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
