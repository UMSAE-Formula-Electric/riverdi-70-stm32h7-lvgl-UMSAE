/*
 * vehicle_state.h
 *
 * Created on: Apr 25, 2026
 * Author: mason
 */

#ifndef INC_VEHICLE_STATE_H_
#define INC_VEHICLE_STATE_H_

#include <stdint.h>

typedef struct vehicleState vehicleState_t;

/* Init */
void VehicleState_Init(void);

/* Pointer Access */
vehicleState_t * VehicleState_GetPtr(void);
const vehicleState_t * VehicleState_GetConstPtr(void);

/* Full Copy */
void VehicleState_GetSnapshot(vehicleState_t *dest);

/* ===== Handlers / Update Functions ===== */

/* Motor / Drive */
void VehicleState_SetRPM(uint8_t rpm);
void VehicleState_SetSpeed(uint8_t speed);
void VehicleState_SetPower(uint8_t power);

/* Driver Inputs */
void VehicleState_SetThrottle(uint8_t throttle);
void VehicleState_SetBrake(uint8_t brake);

/* System */
void VehicleState_SetState(uint8_t state);

/* Battery */
void VehicleState_SetPackVoltage(uint8_t voltage);
void VehicleState_SetPackCurrent(uint8_t current);
void VehicleState_SetSOC(uint8_t soc);
void VehicleState_SetWattage(uint8_t wattage);
void VehicleState_SetCellMax(uint8_t cellmax);
void VehicleState_SetCellMin(uint8_t cellmin);

/* ===== Read Accessors ===== */
uint8_t VehicleState_GetRPM(void);
uint8_t VehicleState_GetSpeed(void);
uint8_t VehicleState_GetPower(void);
uint8_t VehicleState_GetThrottle(void);
uint8_t VehicleState_GetBrake(void);
uint8_t VehicleState_GetState(void);
uint8_t VehicleState_GetPackVoltage(void);
uint8_t VehicleState_GetPackCurrent(void);
uint8_t VehicleState_GetSOC(void);
uint8_t VehicleState_GetWattage(void);
uint8_t VehicleState_GetCellMax(void);
uint8_t VehicleState_GetCellMin(void);

#endif
