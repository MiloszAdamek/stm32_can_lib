// slave_driver.h
#if defined(DEVICE_IS_MASTER)

#ifndef INC_SLAVE_DRIVER_H_
#define INC_SLAVE_DRIVER_H_

#include "can_odrive.h" // Dla enumów stanów i trybów
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Definicja logicznych ról dla każdego silnika w robocie.
 */
typedef enum
{
    FLYWHEEL, // Koło zamachowe
    // MOTOR_LEFT,
    // MOTOR_RIGHT,
    DRIVE_WHEEL,
    NUM_SLAVES // Liczba Slave'ów
} SlaveRole_t;

/**
 * @brief Struktura przechowująca stan wewnętrzny Slave'a.
 */
typedef struct
{
    uint16_t axis_error;
    uint16_t axis_state;
    uint16_t occurred_faults;
    float pos_estimate;
    float vel_estimate;
    uint16_t motor_power_watt;
    uint32_t last_heartbeat_tick;
} SlaveState_t;

/**
 * @brief Inicjalizuje sterowniki wszystkich slave'ów i komunikację CAN.
 * @param hcan_void Wskaźnik do uchwytu peryferium CAN.
 */
void SlaveDriver_Init(void *hcan_void);

// --- FUNKCJE DO WYSYŁANIA KOMEND (AKCJE) ---

/**
 * @brief Resetuje wybranego Slave'a
 * @param role Rola silnika (np. FLYWHEEL).
 */
void SlaveDriver_Reboot(SlaveRole_t role);

/**
 * @brief Ustawia docelową prędkość dla wybranego silnika.
 * @param role Rola silnika (np. FLYWHEEL).
 * @param velocity Prędkość w [RPM].
 */
void SlaveDriver_SetVelocity(SlaveRole_t role, float velocity);

/**
 * @brief Ustawia docelowy moment obrotowy dla wybranego silnika.
 * @param role Rola silnika.
 * @param torque Moment w [A].
 */
void SlaveDriver_SetTorque(SlaveRole_t role, float torque);

/**
 * @brief Zmienia stan osi dla wybranego silnika.
 * @param role Rola silnika.
 * @param state Docelowy stan (np. AXIS_STATE_CLOSED_LOOP_CONTROL).
 */
void SlaveDriver_SetState(SlaveRole_t role, ODrive_Axis_State_t state);

/**
 * @brief Wysyła komendę kasowania błędów do wybranego silnika.
 * @param role Rola silnika.
 */
void SlaveDriver_ClearErrors(SlaveRole_t role);

// --- FUNKCJE DO ODCZYTYWANIA STANU (ZAPYTANIA) ---

// /**
//  * @brief Zwraca ostatnią znaną prędkość silnika.
//  * @param role Rola silnika.
//  * @return Prędkość w [RPM].
//  */
// float SlaveDriver_GetVelocity(SlaveRole_t role);

// /**
//  * @brief Zwraca ostatnią znaną pozycję enkodera.
//  * @param role Rola silnika.
//  * @return Pozycja (w obrotach).
//  */
// float SlaveDriver_GetPosition(SlaveRole_t role);

// /**
//  * @brief Zwraca ostatni znany stan osi.
//  * @param role Rola silnika.
//  * @return Stan z enum ODrive_Axis_State_t.
//  */
// uint32_t SlaveDriver_GetState(SlaveRole_t role);

// /**
//  * @brief Zwraca ostatni znany kod błędu osi.
//  * @param role Rola silnika.
//  * @return Kod błędu.
//  */
// uint32_t SlaveDriver_GetError(SlaveRole_t role);

/**
 * @brief Sprawdza, czy dany slave jest online.
 * @param role Rola silnika.
 * @param timeout_ms Czas w ms, po którym slave jest uznawany za offline.
 * @return true, jeśli slave jest online.
 */
bool SlaveDriver_IsOnline(SlaveRole_t role, uint32_t timeout_ms);

/**
 * @brief Ustawia częstotliwość wysyłania ramek heartbeat
 * @param role Rola silnika.
 * @param heartbeat_freq Częstotliwość wysyłania ramki heartbeat w Hz
 * @param telemetry_freq Częstotliwość wysyłania ramki telemetrii w Hz
 */
void SlaveDriver_SetHeartbeatFreq(SlaveRole_t role, uint16_t heartbeat_freq, uint16_t telemetry_freq);

/**
 * @brief Pobiera w sposób bezpieczny pełną kopię stanu slave'a.
 */
bool SlaveDriver_GetStateCopy_RTOS(SlaveRole_t role, SlaveState_t *pStateCopy);

// /**
//  * @brief Analiza błędów slave'a
//  */
// void SlaveDriver_CheckErrors(SlaveRole_t role);

// /**
//  * @brief Analiza błędów slave'a
//  */
void SlaveDriver_InitESCs(void);

void SlaveDriver_StopAll(void);

/**
 * @brief Znajduje rolę slave'a na podstawie jego CAN Node ID.
 * @param node_id CAN Node ID slave'a.
 * @return Rola slave'a (SlaveRole_t) lub NUM_SLAVES jeśli nie znaleziono.
 */
SlaveRole_t GetSlaveRoleFromId(uint8_t node_id);

/**
 * @brief Ustawia limity prędkości i momentu
 * @param role Rola silnika.
 * @param speed_limit maksymalna prędkość w rpm
 * @param torque_limit maksymalny moment w Nm
 */
void SlaveDriver_SetLimits(SlaveRole_t role, uint16_t speed_limit, uint16_t torque_limit);

#endif /* INC_SLAVE_DRIVER_H_ */

#if defined(DEVICE_IS_MASTER)
