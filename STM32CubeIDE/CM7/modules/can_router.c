/*
 * can_router.c
 *
 *  Created on: May 16, 2026
 *      Author: mason
 */


#include "can_router.h"
#include "can_utils.h"
#include "motor_controller_can_utils.h"
#include "bms_can_utils.h"
#include "car_state.h"

static void handle_startup_state_msg(uint8_t msg)
{
    switch ((enum STARTUP_STATUS_NOTIFY_MSG)msg)
    {
        case CAN_GO_IDLE_REQ:
            car_state_request(IDLE);
            break;

        case CAN_ACB_TSA_ACK:
        	car_state_request(TRACTIVE_SYSTEM_ACTIVE);
        case CAN_ACB_RTD_ACK:
            car_state_request(READY_TO_DRIVE);
            break;

        case CAN_ACB_TSA_NACK:
        case CAN_ACB_RTD_NACK:
            break;

        case CAN_NO_SAFETY_LOOP_SET:
            car_state_request(ERROR_STATE);
            break;

        case CAN_NO_SAFETY_LOOP_CLEAR:
            car_state_request(IDLE);
            break;

        default:
            break;
    }
}

void CAN_router(const can_frame_t *frame)
{
    /* ---------------- STATE TRANSITION / CONTROL PLANE ---------------- */
    switch (frame->id)
    {
        case CAN_VCU_SET_ACB_STATE_ID:
            handle_startup_state_msg(frame->data[0]);
            return;

        default:
            break;
    }

    /* ---------------- MOTOR CONTROLLER RANGE ---------------- */
    switch (frame->id)
    {
        case CAN_MC_RX_TEMP1_ID:
        case CAN_MC_RX_TEMP2_ID:
        case CAN_MC_RX_TEMP3_ID:
        case CAN_MC_RX_ANALOG_INPUTS_VOLTAGE:
        case CAN_MC_RX_DIGITAL_INPUT_STATUS:
        case CAN_MC_RX_MOTOR_ID:
        case CAN_MC_RX_CURRENT_ID:
        case CAN_MC_RX_VOLT_ID:
        case CAN_MC_RX_FAULT_ID:
        case CAN_MC_RX_INTERNAL_VOLTAGES:
        case CAN_MC_RX_INTERNAL_STATES:
        case CAN_MC_RX_TORQUE_TIMER_INFO:
        case CAN_MC_RX_MODULATION_INDEX:
        case CAN_MC_RX_FIRMWARE_INFO:
        case CAN_MC_RX_DIAGNOSTIC_DATA:
        case CAN_MC_RX_HIGHSPEED:
        case CAN_MC_RX_TORQUE_CAPABILITY:
            MotorController_Process(frame->id, frame->data);
            break;

        /* ---------------- BMS ---------------- */
        case CAN_BMS_STATE_OF_CHARGE:
            BMS_Process(frame->id, frame->data);
            break;

        /* ---------------- IGNORE OR EXTEND ---------------- */
        default:
            break;
    }
}
