#include "can_port.h"
#include <stddef.h>

#define MAX_CAN_PORTS 2

#if defined(TARGET_STM32H7)
extern can_driver_t can_stm32h7_create(uint8_t can_bus_number);
#define CREATE(bus) can_stm32h7_create(bus)

#elif defined(TARGET_STM32F4)
extern can_driver_t can_stm32f4_create(uint8_t can_bus_number);
#define CREATE(bus) can_stm32f4_create(bus)

#elif defined(TARGET_MOCK)
extern can_driver_t can_mock_create(uint8_t can_bus_number);
#define CREATE(bus) can_mock_create(bus)

#endif

can_driver_t *can_port_create(uint8_t can_bus_number)
{
    static can_driver_t drivers[MAX_CAN_PORTS];

    uint8_t idx = can_bus_number - 1;

    if (idx >= MAX_CAN_PORTS) {
        return NULL;
    }

    drivers[idx] = CREATE(can_bus_number);

    return &drivers[idx];
}
