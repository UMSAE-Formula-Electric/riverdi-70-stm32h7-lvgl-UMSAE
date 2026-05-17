#ifndef NEWVCU_CAR_STATE_H
#define NEWVCU_CAR_STATE_H

#include <stdbool.h>

typedef enum CAR_STATE{
    IDLE = 0,
    TRACTIVE_SYSTEM_ACTIVE,
    READY_TO_DRIVE,
    ERROR_STATE,
    CHARGING_SYSTEM_ACTIVE,
    NUM_CAR_STATES

}CAR_STATE_t;


/**
 * @brief Initialize the car state machine.
 */
void car_state_init(void);

/**
 * @brief Request a state transition.
 * @return true if transition was accepted, false if rejected.
 */
bool car_state_request(CAR_STATE_t new_state);

/**
 * @brief Get current system state (atomic read).
 */
CAR_STATE_t car_state_get(void);

/**
 * @brief Force error state (bypasses rules).
 * @note Only for fault handlers / safety shutdown.
 */
void car_state_enter_error(void);



#endif //NEWVCU_CAR_STATE_H
