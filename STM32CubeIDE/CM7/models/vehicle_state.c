/*
 * vehicle_state.c
 *
 * Created on: Apr 25, 2026
 * Author: mason
 */

#include <stdint.h>
#include <string.h>
#include "vehicle_state.h"

struct vehicleState {
    uint16_t rpm;
    uint16_t speed;

    int16_t powerKW;       // signed for regen
    uint8_t throttle;
    uint8_t brake;
    uint8_t state;

    uint16_t packVoltage;  // volts
    int16_t packCurrent;   // signed amps

    uint8_t SOC;

    int32_t wattage;       // actual watts
    uint16_t cellmax;
    uint16_t cellmin;
};

static vehicleState_t g_vehicleState;

/* ================= Init ================= */

void VehicleState_Init(void)
{
    memset(&g_vehicleState, 0, sizeof(g_vehicleState));
}

/* ================= Pointer Access ================= */

vehicleState_t * VehicleState_GetPtr(void)
{
    return &g_vehicleState;
}

const vehicleState_t * VehicleState_GetConstPtr(void)
{
    return &g_vehicleState;
}

void VehicleState_GetSnapshot(vehicleState_t *dest)
{
    if (dest != 0)
    {
        *dest = g_vehicleState;
    }
}

/* ================= Handlers ================= */

void VehicleState_SetRPM(uint16_t rpm)
{
    g_vehicleState.rpm = rpm;
}

void VehicleState_SetSpeed(uint16_t speed)
{
    g_vehicleState.speed = speed;
}

void VehicleState_SetPower(uint8_t power)
{
    g_vehicleState.powerKW = power;
}

void VehicleState_SetThrottle(uint8_t throttle)
{
    g_vehicleState.throttle = throttle;
}

void VehicleState_SetBrake(uint8_t brake)
{
    g_vehicleState.brake = brake;
}

void VehicleState_SetState(uint8_t state)
{
    g_vehicleState.state = state;
}

void VehicleState_SetPackVoltage(uint16_t voltage)
{
    g_vehicleState.packVoltage = voltage;
}

void VehicleState_SetPackCurrent(uint16_t current)
{
    g_vehicleState.packCurrent = current;
}

void VehicleState_SetSOC(uint8_t soc)
{
    g_vehicleState.SOC = soc;
}

void VehicleState_SetWattage(uint8_t wattage)
{
    g_vehicleState.wattage = wattage;
}

void VehicleState_SetCellMax(uint8_t cellmax)
{
    g_vehicleState.cellmax = cellmax;
}

void VehicleState_SetCellMin(uint8_t cellmin)
{
    g_vehicleState.cellmin = cellmin;
}

/* ================= Getters ================= */

uint16_t VehicleState_GetRPM(void)         { return g_vehicleState.rpm; }
uint16_t VehicleState_GetSpeed(void)       { return g_vehicleState.speed; }
uint16_t VehicleState_GetPower(void)       { return g_vehicleState.powerKW; }
uint16_t VehicleState_GetThrottle(void)    { return g_vehicleState.throttle; }
uint16_t VehicleState_GetBrake(void)       { return g_vehicleState.brake; }
uint16_t VehicleState_GetState(void)       { return g_vehicleState.state; }
uint16_t VehicleState_GetPackVoltage(void) { return g_vehicleState.packVoltage; }
uint16_t VehicleState_GetPackCurrent(void) { return g_vehicleState.packCurrent; }
uint16_t VehicleState_GetSOC(void)         { return g_vehicleState.SOC; }
uint32_t VehicleState_GetWattage(void)     { return g_vehicleState.wattage; }
uint16_t VehicleState_GetCellMax(void)     { return g_vehicleState.cellmax; }
uint16_t VehicleState_GetCellMin(void)     { return g_vehicleState.cellmin; }
