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
    // [FLYWHEEL] = 1, // Koło zamachowe ma CAN ID 1
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
    uint32_t node_id = pHeader->Identifier >> 5;
    uint32_t cmd_id = pHeader->Identifier & 0x1F;

    for (int role = 0; role < NUM_SLAVES; role++)
    {
        if (SLAVE_NODE_IDS[role] == node_id)
        {
            switch (cmd_id)
            {
            case ODRIVE_HEARTBEAT_MESSAGE:
                memcpy((void *)&g_slave_states[role].axis_state, &pData[0], sizeof(uint16_t));
                memcpy((void *)&g_slave_states[role].axis_error, &pData[2], sizeof(uint16_t));
                memcpy((void *)&g_slave_states[role].occurred_faults, &pData[4], sizeof(uint16_t));
                memcpy((void *)&g_slave_states[role].motor_power_watt, &pData[6], sizeof(uint16_t));
                g_slave_states[role].last_heartbeat_tick = HAL_GetTick();

                // printf("CAN RX, Node ID: %d Heartbeat - State: %u, Error: 0x%X, Occurred fault: 0x%X, Motor Power: %u W\r\n",
                //     (int)node_id,
                //     g_slave_states[role].axis_state,
                //     g_slave_states[role].axis_error,
                //     g_slave_states[role].occurred_faults,
                //     g_slave_states[role].motor_power_watt);

                // Powiadom zadanie, że otrzymano nowy heartbeat do sprawdzenia
                if (task_slave_monitor_handle != NULL)
                {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    vTaskNotifyGiveFromISR(task_slave_monitor_handle, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Jeśli zadanie monitorujące ma wyższy priorytet, przełącz kontekst
                }

                break;
            case ODRIVE_GET_ENCODER_ESTIMATES:
                memcpy((void *)&g_slave_states[role].pos_estimate, &pData[0], sizeof(float)); // Odczyt pozycji w Rev
                memcpy((void *)&g_slave_states[role].vel_estimate, &pData[4], sizeof(float)); // Odczyt prędkości w Rev/s

                float vel_rpm = g_slave_states[role].vel_estimate;
                float pos_deg = g_slave_states[role].pos_estimate;

                // printf("CAN RX, Node ID: %d Velocity -> %.2f RPM\r\n", (int)node_id, vel_rpm);
                // printf("CAN RX, Node ID: %d Position -> %.2f Rev\r\n", (int)node_id, pos_deg);

                break;
            }
            return;
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

void SlaveDriver_SetState(SlaveRole_t role, ODrive_Axis_State_t state)
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

// float SlaveDriver_GetVelocity(SlaveRole_t role) {
//     if (role < NUM_SLAVES) {
//         CAN_Master_Request_Encoder_Estimates(SLAVE_NODE_IDS[role]);
//         return g_slave_states[role].vel_estimate;
//     }
//     return 0.0f;
// }

// float SlaveDriver_GetPosition(SlaveRole_t role) {
//     if (role < NUM_SLAVES) {
//         CAN_Master_Request_Encoder_Estimates(SLAVE_NODE_IDS[role]);
//         return g_slave_states[role].pos_estimate;
//     }
//     return 0.0f;
// }

// uint32_t SlaveDriver_GetState(SlaveRole_t role) {
//     if (role < NUM_SLAVES) {
//         return g_slave_states[role].axis_state;
//     }
//     return AXIS_STATE_UNDEFINED;
// }

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
        CAN_Master_Send_HeartbeatFreq(SLAVE_NODE_IDS[role], heartbeat_freq, telemetry_freq);
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
        // SlaveDriver_SetLimits(role, 3000, 100);
        SlaveDriver_SetState(role, AXIS_STATE_IDLE);
    }
}

void SlaveDriver_StopAll(void)
{
    for (int role = 0; role < NUM_SLAVES; role++)
    {
        SlaveDriver_SetState(role, AXIS_STATE_IDLE);
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

void SlaveDriver_SetLimits(SlaveRole_t role, uint16_t speed_limit, uint16_t torque_limit)
{
    if (role < NUM_SLAVES)
    {
        CAN_Master_Send_Limits(SLAVE_NODE_IDS[role], speed_limit, torque_limit);
    }
}

#endif // DEVICE_IS_MASTER
