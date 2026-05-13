/*
 * vehicle_state.h
 *
 * Created on: Apr 25, 2026
 * Author: mason
 */

#ifndef INC_VEHICLE_STATE_H_
#define INC_VEHICLE_STATE_H_

#include <stdint.h>

/**
 * @brief Opaque handle to the vehicle state structure.
 * @details The implementation details are hidden in the source file to
 * prevent direct member access from other modules.
 */
typedef struct vehicleState vehicleState_t;

/* Init */
/**
 * @brief Initializes the vehicle state memory and default values.
 * @note  Should be called before any tasks attempt to read/write state.
 */
void VehicleState_Init(void);

/* Pointer Access */
/**
 * @brief  Returns a mutable pointer to the global vehicle state.
 * @return vehicleState_t* Pointer to the internal state object.
 */
vehicleState_t * VehicleState_GetPtr(void);
/**
 * @brief  Returns a read-only pointer to the global vehicle state.
 * @return const vehicleState_t* Constant pointer to the internal state object.
 */
const vehicleState_t * VehicleState_GetConstPtr(void);

/* Full Copy */
void VehicleState_GetSnapshot(vehicleState_t *dest);

/* ===== Handlers / Update Functions ===== */

/** @name Motor / Drive Setters */
/** @{ */
void VehicleState_SetRPM(uint16_t rpm);
void VehicleState_SetSpeed(uint16_t speed);
void VehicleState_SetPower(uint8_t power);
/** @} */

/** @name Driver Input Setters */
/** @{ */
void VehicleState_SetThrottle(uint8_t throttle);
void VehicleState_SetBrake(uint8_t brake);
/** @} */

/** @name System Setters */
/** @{ */
void VehicleState_SetState(uint8_t state);
/** @} */

/** @name Battery Metric Setters */
/** @{ */
void VehicleState_SetPackVoltage(uint8_t voltage);
void VehicleState_SetPackCurrent(uint8_t current);
void VehicleState_SetSOC(uint8_t soc);
void VehicleState_SetWattage(uint8_t wattage);
void VehicleState_SetCellMax(uint8_t cellmax);
void VehicleState_SetCellMin(uint8_t cellmin);
/** @} */

/* ===== Read Accessors ===== */
/**
 * @brief Retrieves the current motor RPM.
 * @return uint8_t Current RPM value.
 */
uint8_t VehicleState_GetRPM(void);

/**
 * @brief Retrieves the current vehicle speed.
 * @return uint8_t Speed in km/h (or configured units).
 */
uint8_t VehicleState_GetSpeed(void);

/**
 * @brief Retrieves the current power output.
 * @return uint8_t Power percentage or scaled value.
 */
uint8_t VehicleState_GetPower(void);

uint8_t VehicleState_GetThrottle(void);
uint8_t VehicleState_GetBrake(void);
uint8_t VehicleState_GetState(void);
uint8_t VehicleState_GetPackVoltage(void);
uint8_t VehicleState_GetPackCurrent(void);

/**
 * @brief Retrieves the Battery State of Charge (SOC).
 * @return uint8_t Percentage (0-100).
 */
uint8_t VehicleState_GetSOC(void);

uint8_t VehicleState_GetWattage(void);
uint8_t VehicleState_GetCellMax(void);
uint8_t VehicleState_GetCellMin(void);

#endif
