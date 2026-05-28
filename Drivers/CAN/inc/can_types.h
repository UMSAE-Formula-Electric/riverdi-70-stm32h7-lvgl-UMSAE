/*
 * can_types.c
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */


#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     extended;
} can_frame_t;

typedef enum
{
    CAN_OK = 0,
    CAN_ERR,
    CAN_TIMEOUT
} can_status_t;
