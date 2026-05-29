/*
 * dashboard_ui.c
 *
 *  Created on: May 12, 2026
 *      Author: mason
 */


#include "dashboard_ui.h"


#include "lvgl/lvgl.h"
#include "ui.h"
#include "ui_task.h"
#include "ui_theme.h"

void DashboardUI_Init(void)
{
	ASSERT_LVGL_TASK();
	DashboardUI_ApplyTheme(UITheme_Get());
    DashboardUI_SetVehicleState(IDLE);
}

void DashboardUI_ApplyTheme(const UITheme_t *t)
{
	ASSERT_LVGL_TASK();
    // Screen background
    lv_obj_set_style_bg_color(ui_DriverDisplay, t->colors.bg_primary, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_DriverDisplay, LV_OPA_COVER, LV_PART_MAIN);

    // Vehicle state labels
    lv_obj_set_style_text_font(ui_VehicleStateLabel, t->fonts.lg, LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_VehicleStateLabel, t->colors.text_secondary, LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_VehicleStateText, t->fonts.lg, LV_PART_MAIN);

    // Stat widgets — title / value / unit
    typedef struct { lv_obj_t *title; lv_obj_t *value; lv_obj_t *unit; } StatRef_t;
    StatRef_t stats[] = {
        { ui_PowerTag,          ui_PowerBaseTag,    ui_PowerSubTextTag    },
        { ui_SpeedTag,          ui_SpeedBaseTag,    ui_SpeedSubTextTag    },
        { ui_MotorTag,          ui_MotorBaseTag,    ui_MotorSubTextTag    },
        { ui_ThrottleTag,       ui_ThrottleBaseTag, ui_ThrottleSubTextTag },
        { ui_StateofChargeTag,  ui_SOCBaseTag,      ui_BatterySubTextTag  },
        { ui_BrakeTag,          ui_BrakeBaseTag,    ui_BrakeSubTextTag    },
    };

    for (size_t i = 0; i < sizeof(stats) / sizeof(stats[0]); i++) {
        lv_obj_set_style_text_color(stats[i].title, t->colors.text_secondary, LV_PART_MAIN);
        lv_obj_set_style_text_font(stats[i].title,  t->fonts.md,              LV_PART_MAIN);

        lv_obj_set_style_text_color(stats[i].value, t->colors.text_primary,   LV_PART_MAIN);
        lv_obj_set_style_text_font(stats[i].value,  t->fonts.value_sm,        LV_PART_MAIN);

        lv_obj_set_style_text_color(stats[i].unit,  t->colors.text_secondary, LV_PART_MAIN);
        lv_obj_set_style_text_font(stats[i].unit,   t->fonts.sm,              LV_PART_MAIN);
    }

    // Speed gets the large value font
    lv_obj_set_style_text_font(ui_SpeedBaseTag, t->fonts.value_lg, LV_PART_MAIN);

    // Temperature bar
    lv_obj_set_style_text_color(ui_TemperatureTag,      t->colors.text_secondary, LV_PART_MAIN);
    lv_obj_set_style_text_color(ui_TemperatureBaseTag,  t->colors.text_primary,   LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_TemperatureBar,        t->colors.bar_background, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_TemperatureBar,        t->colors.bar_indicator,  LV_PART_INDICATOR);
}
void DashboardUI_SetSpeed(uint16_t speed_kmh)
{
	ASSERT_LVGL_TASK();
    lv_label_set_text_fmt(
		ui_SpeedBaseTag,
        "%u",
        speed_kmh
    );
}

void DashboardUI_SetRPM(uint16_t rpm)
{
	ASSERT_LVGL_TASK();
    lv_label_set_text_fmt(
		ui_MotorBaseTag,
        "%u",
        rpm
    );
}

void DashboardUI_SetThrottle(uint8_t throttle_percent)
{
	ASSERT_LVGL_TASK();
    lv_label_set_text_fmt(
		ui_ThrottleBaseTag,
        "%u%%",
        throttle_percent
    );
}

void DashboardUI_SetBrake(uint8_t brake_percent)
{
	ASSERT_LVGL_TASK();
    lv_label_set_text_fmt(
		ui_BrakeBaseTag,
        "%u%%",
        brake_percent
    );
}

void DashboardUI_SetPower(uint16_t power){
	ASSERT_LVGL_TASK();
	lv_label_set_text_fmt(
		ui_PowerBaseTag,
		"%u",
		power
	);
}

void DashboardUI_SetSOC(uint8_t soc_percent)
{
	ASSERT_LVGL_TASK();
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
	ASSERT_LVGL_TASK();
	lv_label_set_text_fmt(
			ui_TemperatureBaseTag,
			"%u°C",
			temprature);
}

void DashboardUI_SetVehicleState(enum CAR_STATE state){
	ASSERT_LVGL_TASK();
    const UITheme_t *t = UITheme_Get();
    const char *text   = "UNKNOWN";
    lv_color_t color   = t->states.state_unknown;

    switch (state)
    {
        case IDLE:
            text  = "IDLE";
            color = t->states.state_idle;
            break;
        case TRACTIVE_SYSTEM_ACTIVE:
            text  = "TRACTIVE SYSTEM ACTIVE";
            color = t->states.state_tractive;
            break;
        case READY_TO_DRIVE:
            text  = "READY TO DRIVE";
            color = t->states.state_ready;
            break;
        case ERROR_STATE:
            text  = "ERROR";
            color = t->states.state_error;
            break;
        case CHARGING_SYSTEM_ACTIVE:
            text  = "CHARGING";
            color = t->states.state_charging;
            break;
        default:
            break;
    }

    lv_label_set_text(ui_VehicleStateText, text);
    lv_obj_set_style_text_color(ui_VehicleStateText, color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
}

