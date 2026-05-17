/*
 * dashboard_ui.h
 *
 *  Created on: May 12, 2026
 *      Author: mason
 */

#ifndef DASHBOARD_UI_H
#define DASHBOARD_UI_H

#include <stdint.h>
#include <stdbool.h>

#include "car_state.h"

void DashboardUI_Init(void);

void DashboardUI_SetSpeed(uint16_t speed_kmh);
void DashboardUI_SetRPM(uint16_t rpm);
void DashboardUI_SetThrottle(uint8_t throttle_percent);
void DashboardUI_SetBrake(uint8_t brake_percent);
void DashboardUI_SetPower(uint16_t power);
void DashboardUI_SetSOC(uint8_t soc_percent);
void DashboardUI_SetDegree(uint8_t temprature);
void DashboardUI_SetTempGauge(uint16_t temp_percent);

void DashboardUI_SetVehicleState(enum CAR_STATE state);

#endif /* UI_DASHBOARD_UI_H_ */
