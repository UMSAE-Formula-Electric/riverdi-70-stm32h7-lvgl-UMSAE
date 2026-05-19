/*
 * freertos_heap.c
 *
 *  Created on: May 18, 2026
 *      Author: mason
 */
#include "FreeRTOS.h"

/* Place FreeRTOS heap into D2 SRAM */
__attribute__((section(".ram_d2")))
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
