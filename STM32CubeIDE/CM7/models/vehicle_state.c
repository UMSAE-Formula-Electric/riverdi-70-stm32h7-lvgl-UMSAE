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
    uint8_t rpm;
    uint8_t speed;
    uint8_t power;
    uint8_t Throttle;
    uint8_t brake;
    uint8_t state;
    uint8_t packVoltage;
    uint8_t packCurrent;
    uint8_t SOC;
    uint8_t wattage;
    uint8_t cellmax;
    uint8_t cellmin;
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

void VehicleState_SetRPM(uint8_t rpm)
{
    g_vehicleState.rpm = rpm;
}

void VehicleState_SetSpeed(uint8_t speed)
{
    g_vehicleState.speed = speed;
}

void VehicleState_SetPower(uint8_t power)
{
    g_vehicleState.power = power;
}

void VehicleState_SetThrottle(uint8_t throttle)
{
    g_vehicleState.Throttle = throttle;
}

void VehicleState_SetBrake(uint8_t brake)
{
    g_vehicleState.brake = brake;
}

void VehicleState_SetState(uint8_t state)
{
    g_vehicleState.state = state;
}

void VehicleState_SetPackVoltage(uint8_t voltage)
{
    g_vehicleState.packVoltage = voltage;
}

void VehicleState_SetPackCurrent(uint8_t current)
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

uint8_t VehicleState_GetRPM(void)         { return g_vehicleState.rpm; }
uint8_t VehicleState_GetSpeed(void)       { return g_vehicleState.speed; }
uint8_t VehicleState_GetPower(void)       { return g_vehicleState.power; }
uint8_t VehicleState_GetThrottle(void)    { return g_vehicleState.Throttle; }
uint8_t VehicleState_GetBrake(void)       { return g_vehicleState.brake; }
uint8_t VehicleState_GetState(void)       { return g_vehicleState.state; }
uint8_t VehicleState_GetPackVoltage(void) { return g_vehicleState.packVoltage; }
uint8_t VehicleState_GetPackCurrent(void) { return g_vehicleState.packCurrent; }
uint8_t VehicleState_GetSOC(void)         { return g_vehicleState.SOC; }
uint8_t VehicleState_GetWattage(void)     { return g_vehicleState.wattage; }
uint8_t VehicleState_GetCellMax(void)     { return g_vehicleState.cellmax; }
uint8_t VehicleState_GetCellMin(void)     { return g_vehicleState.cellmin; }
