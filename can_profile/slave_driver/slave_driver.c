// slave_driver.c

#if defined(DEVICE_IS_MASTER)

#include "slave_driver.h"
#include "can_wrapper.h"
#include "can_interface.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

// =================================================================================
// ===           				KONFIGURACJA 								     ===
// =================================================================================
const uint8_t SLAVE_NODE_IDS[NUM_SLAVES] = {
    [FLYWHEEL] = 1, // Koło zamachowe ma CAN ID 1
    // [MOTOR_LEFT]  = 2,
    // [MOTOR_RIGHT] = 3,
    [DRIVE_WHEEL] = 2};
// =================================================================================

// Przechowuje stan Slave'ów
static volatile SlaveState_t g_slave_states[NUM_SLAVES];

extern TaskHandle_t task_slave_monitor_handle;

// Callback do odbioru ramek od Slave'ów
static void internal_rx_callback(const CAN_Wrapper_RxHeader_t *pHeader, const uint8_t *pData)
{
    SlaveRole_t role = GetSlaveRoleFromId(CAN_ID_NODE(pHeader->Identifier));
    if (role == NUM_SLAVES) return;

    volatile SlaveState_t *state = &g_slave_states[role];
    switch (CAN_ID_COMMAND(pHeader->Identifier))
    {
    case CAN_CMD_HEARTBEAT:
    {
        CAN_Heartbeat_t heartbeat = CAN_DecodeHeartbeat(pData);
        state->axis_state = heartbeat.state;
        state->axis_error = heartbeat.faults;
        memcpy((void *)&state->occurred_faults, pData + 4, sizeof(state->occurred_faults));
        memcpy((void *)&state->motor_power_watt, pData + 6, sizeof(state->motor_power_watt));
        state->last_heartbeat_tick = HAL_GetTick();

        if (task_slave_monitor_handle != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(task_slave_monitor_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        break;
    }
    case CAN_CMD_ENCODER_FEEDBACK:
    {
        CAN_TelemetrySnapshot_t encoder = CAN_DecodeEncoderFeedback(pData);
        state->pos_estimate = encoder.pos_rev;
        state->vel_estimate = encoder.vel_rpm;
        break;
    }
    }
}

// --- Publiczne API ---

void SlaveDriver_Init(void *hcan_void)
{
    if (hcan_void == NULL)
    {
        Error_Handler();
    }

    CAN_Wrapper_Init(hcan_void);

    CAN_Master_Init();
    CAN_Master_SetRxCallback(internal_rx_callback);
}

void SlaveDriver_Reboot(SlaveRole_t role)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_Reboot(SLAVE_NODE_IDS[role]);
    }
};

void SlaveDriver_SetVelocity(SlaveRole_t role, float velocity)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_Velocity(SLAVE_NODE_IDS[role], velocity);
    }
}

void SlaveDriver_SetTorque(SlaveRole_t role, float torque)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_Torque(SLAVE_NODE_IDS[role], torque);
    }
}

void SlaveDriver_SetCurrentLimit(SlaveRole_t role, float current_limit_A)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_CurrentLimit(SLAVE_NODE_IDS[role], current_limit_A);
    }
}

void SlaveDriver_SetState(SlaveRole_t role, CAN_StateRequest_t state)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_State(SLAVE_NODE_IDS[role], state);
    }
}

void SlaveDriver_ClearErrors(SlaveRole_t role)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_Clear_Errors(SLAVE_NODE_IDS[role]);
    }
}

// uint32_t SlaveDriver_GetError(SlaveRole_t role) {
//     if (role < NUM_SLAVES) {
//         return g_slave_states[role].axis_error;
//     }
//     return 0;
// }

void SlaveDriver_SetHeartbeatFreq(SlaveRole_t role, uint16_t heartbeat_freq, uint16_t telemetry_freq)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_FeedbackRates(SLAVE_NODE_IDS[role], heartbeat_freq, telemetry_freq);
    }
}

void SlaveDriver_SetWatchdogTimeout(SlaveRole_t role, uint16_t timeout_ms)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_WatchdogTimeout(SLAVE_NODE_IDS[role], timeout_ms);
    }
}

bool SlaveDriver_IsOnline(SlaveRole_t role, uint32_t timeout_ms)
{
    if (role < NUM_SLAVES)
    {
        return (HAL_GetTick() - g_slave_states[role].last_heartbeat_tick < timeout_ms);
    }
    return false;
}

bool SlaveDriver_GetStateCopy_RTOS(SlaveRole_t role, SlaveState_t *pStateCopy)
{
    if (role >= NUM_SLAVES || pStateCopy == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    memcpy(pStateCopy, (void *)&g_slave_states[role], sizeof(SlaveState_t));
    taskEXIT_CRITICAL();

    return true;
}

void SlaveDriver_InitESCs(void)
{
    for (int role = 0; role < NUM_SLAVES; role++)
    {
        SlaveDriver_ClearErrors(role);
        SlaveDriver_SetHeartbeatFreq(role, 50, 50); // Ustawienie częstotliwości wysyłania ramek heartbeat i telemetry
        SlaveDriver_SetState(role, CAN_REQUEST_STOP);
    }
}

void SlaveDriver_StopAll(void)
{
    for (int role = 0; role < NUM_SLAVES; role++)
    {
        SlaveDriver_SetState(role, CAN_REQUEST_STOP);
    }
}

/**
 * @brief Znajduje rolę slave'a na podstawie jego CAN Node ID.
 */
SlaveRole_t GetSlaveRoleFromId(uint8_t node_id)
{
    for (int i = 0; i < NUM_SLAVES; i++)
    {
        if (SLAVE_NODE_IDS[i] == node_id)
        {
            return (SlaveRole_t)i; // Zwraca rolę (indeks), np. FLYWHEEL (0)
        }
    }
    return NUM_SLAVES; // Zwraca wartość oznaczającą błąd (nie znaleziono ID)
}

#endif // DEVICE_IS_MASTER
