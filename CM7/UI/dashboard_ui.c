/*
 * dashboard_ui.c
 *
 *  Created on: May 12, 2026
 *      Author: mason
 */


#include "dashboard_ui.h"

#include "ui.h"
#include "lvgl/lvgl.h"

void DashboardUI_Init(void)
{
    DashboardUI_SetVehicleState(IDLE);
}

void DashboardUI_SetSpeed(uint16_t speed_kmh)
{
    lv_label_set_text_fmt(
		ui_SpeedBaseTag,
        "%u",
        speed_kmh
    );
}

void DashboardUI_SetRPM(uint16_t rpm)
{
    lv_label_set_text_fmt(
		ui_MotorBaseTag,
        "%u",
        rpm
    );
}

void DashboardUI_SetThrottle(uint8_t throttle_percent)
{
    lv_label_set_text_fmt(
		ui_ThrottleBaseTag,
        "%u%%",
        throttle_percent
    );
}

void DashboardUI_SetBrake(uint8_t brake_percent)
{
    lv_label_set_text_fmt(
		ui_BrakeBaseTag,
        "%u%%",
        brake_percent
    );
}

void DashboardUI_SetPower(uint8_t power){
	lv_label_set_text_fmt(
		ui_PowerBaseTag,
		"%u",
		power
	);
}

void DashboardUI_SetSOC(uint8_t soc_percent)
{
    lv_label_set_text_fmt(
		ui_SOCBaseTag,
        "%u%%",
        soc_percent
    );
}

void DashboardUI_SetTempGauge(uint16_t temp_percent){
	lv_bar_set_value(
		ui_TemperatureBar,
		temp_percent,
		LV_ANIM_ON
	);
}

void DashboardUI_SetDegree(uint8_t temprature){
	lv_label_set_text_fmt(
			ui_TemperatureBaseTag,
			"%u°C",
			temprature);
}

void DashboardUI_SetVehicleState(enum CAR_STATE state)
{
    const char *text = "";
    lv_color_t color = lv_color_white();

    switch(state)
    {
        case IDLE:
        {
            text = "IDLE";

            color = lv_palette_main(LV_PALETTE_GREY);

            break;
        }

        case TRACTIVE_SYSTEM_ACTIVE:
        {
            text = "TRACTIVE SYSTEM ACTIVE";

            color = lv_palette_main(LV_PALETTE_GREEN);

            break;
        }

        case READY_TO_DRIVE:
        {
            text = "READY TO DRIVE";

            color = lv_palette_main(LV_PALETTE_BLUE);

            break;
        }

        case ERROR_STATE:
        {
            text = "ERROR";

            color = lv_palette_main(LV_PALETTE_RED);

            break;
        }

        case CHARGING_SYSTEM_ACTIVE:
        {
            text = "CHARGING";

            color = lv_palette_main(LV_PALETTE_YELLOW);

            break;
        }

        default:
        {
            text = "UNKNOWN";

            color = lv_palette_main(LV_PALETTE_RED);

            break;
        }
    }

    lv_label_set_text(
        ui_VehicleStateText,
        text
    );

    lv_obj_set_style_text_color(
        ui_VehicleStateText,
        color,
        LV_PART_MAIN | LV_STATE_DEFAULT
    );
}
