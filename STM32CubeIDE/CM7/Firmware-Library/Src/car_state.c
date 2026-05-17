#include <stdbool.h>
#include "car_state.h"
#include "FreeRTOS.h"
#include "task.h"



static volatile CAR_STATE_t current_state = IDLE;

static bool is_valid_transition(CAR_STATE_t from, CAR_STATE_t to)
{
    if (from == to) return true;

    switch (from)
    {
        case IDLE:
            return (to == TRACTIVE_SYSTEM_ACTIVE || to == ERROR_STATE);

        case TRACTIVE_SYSTEM_ACTIVE:
            return (to == READY_TO_DRIVE || to == ERROR_STATE);

        case READY_TO_DRIVE:
            return (to == IDLE || to == ERROR_STATE);

        case CHARGING_SYSTEM_ACTIVE:
            return (to == IDLE || to == ERROR_STATE);

        case ERROR_STATE:
            return false; // latch until reset

        default:
            return false;
    }
}

void car_state_init(void)
{
    taskENTER_CRITICAL();
    current_state = IDLE;
    taskEXIT_CRITICAL();
}

CAR_STATE_t car_state_get(void)
{
    CAR_STATE_t s;
    taskENTER_CRITICAL();
    s = current_state;
    taskEXIT_CRITICAL();
    return s;
}

bool car_state_request(CAR_STATE_t new_state)
{
    bool ok = false;

    taskENTER_CRITICAL();
    if (is_valid_transition(current_state, new_state))
    {
        current_state = new_state;
        ok = true;
    }
    taskEXIT_CRITICAL();

    return ok;
}

void car_state_enter_error(void)
{
    taskENTER_CRITICAL();
    current_state = ERROR_STATE;
    taskEXIT_CRITICAL();
}
