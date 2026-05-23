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
            set_car_state(IDLE);
            break;

        case CAN_ACB_TSA_ACK:
        	set_car_state(TRACTIVE_SYSTEM_ACTIVE);
        	break;
        case CAN_ACB_RTD_ACK:
            set_car_state(READY_TO_DRIVE);
            break;
        default:
            break;
    }
}

void CAN_router(const can_frame_t *frame)
{
    switch (frame->id)
    {
        /* ---------------- MOTOR CONTROLLER RANGE ---------------- */
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
        	MotorController_Process(frame->id,frame->data);
            break;

        /* ---------------- BMS ---------------- */
        case CAN_BMS_STATE_OF_CHARGE:
        	BMS_Process(frame->id,frame->data);
            break;


        case CAN_ACU_TO_VCU_ID:
			handle_startup_state_msg(frame->data[0]);
        	break;
        /* ---------------- IGNORE OR EXTEND ---------------- */
        default:
            break;
    }
}
