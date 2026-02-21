/*
 * can_port.c
 *
 *  Created on: Feb 20, 2026
 *      Author: mason
 */


#include "can_port.h"

#if defined(TARGET_STM32H7)
extern can_driver_t can_stm32h7_create(void);
#define CREATE() can_stm32h7_create()

#elif defined(TARGET_STM32F4)
extern can_driver_t can_stm32f4_create(void);
#define CREATE() can_stm32f4_create()

#elif defined(TARGET_MOCK)
extern can_driver_t can_mock_create(void);
#define CREATE() can_mock_create()

#endif

can_driver_t *can_port_create(void)
{
    static can_driver_t driver;
    driver = CREATE();
    return &driver;
}
