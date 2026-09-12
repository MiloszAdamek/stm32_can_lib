// can_master.c

// --- Implementacja API dla trybu MASTER ---

#if defined(DEVICE_IS_MASTER)

#include "can_interface.h"
#include "can_wrapper.h"
#include <string.h>
#include <stdio.h>
#include "math.h"

// --- Zmienne globalne modułu ---
static CAN_Master_Rx_Callback_t g_master_rx_callback = NULL;

/**
 * @brief Centralny callback dla tego modułu, przekazywany do wrappera.
 *        Rozdziela odebrane wiadomości do logiki Mastera lub Slave'a.
 */
static void odrive_rx_callback(const CAN_Wrapper_RxHeader_t *pHeader, const uint8_t *pData)
{
    if (g_master_rx_callback != NULL)
    {
        g_master_rx_callback(pHeader, pData);
    }
}

void CAN_Master_Init(void)
{
    CAN_Wrapper_RegisterRxCallback(odrive_rx_callback);
    CAN_Wrapper_ConfigFilter_AcceptAll();
    if (CAN_Wrapper_Start() != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Master_SetRxCallback(CAN_Master_Rx_Callback_t callback)
{
    g_master_rx_callback = callback;
}

void CAN_Master_Send_State(uint8_t target_node_id, int32_t state)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_SET_AXIS_REQUESTED_STATE);
    send_can_frame(id, (uint8_t *)&state, sizeof(state));
}

void CAN_Master_Send_Torque(uint8_t target_node_id, float torque)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_SET_INPUT_TORQUE);
    send_can_frame(id, (uint8_t *)&torque, sizeof(torque));
}

void CAN_Master_Send_Velocity(uint8_t target_node_id, float velocity)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_SET_INPUT_VEL);
    send_can_frame(id, (uint8_t *)&velocity, sizeof(velocity));
}

void CAN_Master_Send_Modes(uint8_t target_node_id, int32_t control_mode, int32_t input_mode)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_SET_CONTROLLER_MODES);
    uint8_t data[8];
    memcpy(data, &control_mode, 4);
    memcpy(data + 4, &input_mode, 4);
    send_can_frame(id, data, 8);
}

void CAN_Master_Send_Reboot(uint8_t target_node_id)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_REBOOT);
    send_can_frame(id, NULL, 0);
}

void CAN_Master_Send_Clear_Errors(uint8_t target_node_id)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_CLEAR_ERRORS);
    send_can_frame(id, NULL, 0);
}

void CAN_Master_Request_Encoder_Estimates(uint8_t target_node_id)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_GET_ENCODER_ESTIMATES);
    send_can_frame(id, NULL, 0);
}

void CAN_Master_Request_IQ(uint8_t target_node_id)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_GET_IQ);
    send_can_frame(id, NULL, 0);
}

void CAN_Master_Send_HeartbeatFreq(uint8_t target_node_id, uint16_t heartbeat_freq, uint16_t telemetry_freq)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_SET_HEARTBEAT_FREQ);
    uint8_t data[4];
    memcpy(data, &heartbeat_freq, 2);
    memcpy(data + 2, &telemetry_freq, 2);
    send_can_frame(id, data, 4);
}

void CAN_Master_Send_Limits(uint8_t target_node_id, uint16_t speed_limit, uint16_t torque_limit)
{
    uint32_t id = ODRIVE_MAKE_CAN_ID(target_node_id, ODRIVE_SET_LIMITS);
    uint8_t data[4];
    memcpy(data, &speed_limit, 2);
    memcpy(data + 2, &torque_limit, 2);
    send_can_frame(id, data, 4);
}

#endif // DEVICE_IS_MASTER
