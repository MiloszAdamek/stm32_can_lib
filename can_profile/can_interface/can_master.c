// can_master.c

// --- Implementacja API dla trybu MASTER ---

#if defined(DEVICE_IS_MASTER)

#include "can_interface.h"
#include "can_wrapper.h"
#include <math.h>

// --- Zmienne globalne modułu ---
static CAN_Master_Rx_Callback_t g_master_rx_callback = NULL;
static CAN_Master_Node_t *g_nodes;
static size_t g_node_count;

CAN_Master_Node_t *CAN_Master_FindNode(uint32_t node_id)
{
    for (size_t i = 0; i < g_node_count; ++i)
        if (g_nodes[i].node_id == node_id) return &g_nodes[i];
    return NULL;
}

static void update_feedback(const CAN_Wrapper_RxHeader_t *header, const uint8_t *data)
{
    if (header->DataLength != 8) return;
    CAN_Master_Node_t *node = CAN_Master_FindNode(CAN_ID_NODE(header->Identifier));
    if (!node) return;

    CAN_Master_Feedback_t *feedback = &node->feedback;
    switch (CAN_ID_COMMAND(header->Identifier)) {
    case CAN_CMD_HEARTBEAT: {
        CAN_Heartbeat_t heartbeat = CAN_DecodeHeartbeat(data);
        feedback->state = heartbeat.state;
        feedback->faults = heartbeat.faults;
        feedback->heartbeat_ms = HAL_GetTick();
        feedback->heartbeat_seen = true;
        break;
    }
    case CAN_CMD_ENCODER_FEEDBACK: {
        CAN_TelemetrySnapshot_t encoder = CAN_DecodeEncoderFeedback(data);
        if (!isfinite(encoder.pos_rev) || !isfinite(encoder.vel_rpm)) return;
        feedback->encoder = encoder;
        feedback->encoder_ms = HAL_GetTick();
        feedback->encoder_valid = true;
        break;
    }
    }
}

/**
 * @brief Centralny callback dla tego modułu, przekazywany do wrappera.
 *        Rozdziela odebrane wiadomości do logiki Mastera lub Slave'a.
 */
static void master_receive(const CAN_Wrapper_RxHeader_t *pHeader, const uint8_t *pData)
{
    update_feedback(pHeader, pData);
    if (g_master_rx_callback != NULL)
    {
        g_master_rx_callback(pHeader, pData);
    }
}

void CAN_Master_Init(void)
{
    g_nodes = NULL;
    g_node_count = 0;
    CAN_Wrapper_RegisterRxCallback(master_receive);
    CAN_Wrapper_ConfigFilter_AcceptAll();
    if (CAN_Wrapper_Start() != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef CAN_Master_Open(FDCAN_HandleTypeDef *bus, CAN_Master_Node_t *nodes, size_t count)
{
    if (!bus || !nodes || !count || count > CAN_NODE_ID_MAX + 1) return HAL_ERROR;
    uint64_t used_nodes = 0;
    for (size_t i = 0; i < count; ++i) {
        uint32_t id = nodes[i].node_id;
        if (id > CAN_NODE_ID_MAX || (used_nodes & (UINT64_C(1) << id))) return HAL_ERROR;
        used_nodes |= UINT64_C(1) << id;
    }

    for (size_t i = 0; i < count; ++i)
        nodes[i] = (CAN_Master_Node_t){.node_id = nodes[i].node_id};
    g_nodes = nodes;
    g_node_count = count;
    CAN_Wrapper_Init(bus);
    CAN_Wrapper_SetBusOffRecovery(false);
    CAN_Wrapper_RegisterRxCallback(master_receive);
    HAL_StatusTypeDef status = HAL_ERROR;
    if (CAN_Wrapper_ConfigFilter(0, 0) == HAL_OK &&
        HAL_FDCAN_ConfigGlobalFilter(bus, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK)
        status = CAN_Wrapper_Start();
    if (status != HAL_OK) {
        CAN_Wrapper_Init(NULL);
        g_nodes = NULL;
        g_node_count = 0;
    }
    return status;
}

bool CAN_Master_GetFeedback(uint32_t node_id, CAN_Master_Feedback_t *out, uint32_t max_encoder_age_ms)
{
    CAN_Master_Node_t *node = CAN_Master_FindNode(node_id);
    if (!node || !out) return false;
    *out = node->feedback;
    if (HAL_GetTick() - out->encoder_ms > max_encoder_age_ms) out->encoder_valid = false;
    return true;
}

void CAN_Master_SetRxCallback(CAN_Master_Rx_Callback_t callback)
{
    g_master_rx_callback = callback;
}

static HAL_StatusTypeDef send_command(uint8_t node, CAN_Command_t command, const void *data, uint8_t size)
{
    if (node > CAN_NODE_ID_MAX || (g_node_count && !CAN_Master_FindNode(node))) return HAL_ERROR;
    return send_can_frame(CAN_MAKE_ID(node, command), data, size);
}

HAL_StatusTypeDef CAN_Master_Send_State(uint8_t target_node_id, int32_t state)
{
    return send_command(target_node_id, CAN_CMD_REQUEST_STATE, &state, sizeof(state));
}

HAL_StatusTypeDef CAN_Master_Send_Torque(uint8_t target_node_id, float torque)
{
    if (!isfinite(torque)) return HAL_ERROR;
    return send_command(target_node_id, CAN_CMD_SET_TORQUE, &torque, sizeof(torque));
}

HAL_StatusTypeDef CAN_Master_Send_Velocity(uint8_t target_node_id, float velocity)
{
    if (!isfinite(velocity)) return HAL_ERROR;
    return send_command(target_node_id, CAN_CMD_SET_VELOCITY, &velocity, sizeof(velocity));
}

HAL_StatusTypeDef CAN_Master_Send_WatchdogTimeout(uint8_t target_node_id, uint16_t timeout_ms)
{
    return send_command(target_node_id, CAN_CMD_SET_WATCHDOG_TIMEOUT, &timeout_ms, sizeof(timeout_ms));
}

HAL_StatusTypeDef CAN_Master_Send_Reboot(uint8_t target_node_id)
{
    CAN_Master_Node_t *node = CAN_Master_FindNode(target_node_id);
    HAL_StatusTypeDef status = send_command(target_node_id, CAN_CMD_REBOOT, NULL, 0);
    if (node && status == HAL_OK) node->feedback = (CAN_Master_Feedback_t){0};
    return status;
}

HAL_StatusTypeDef CAN_Master_Send_Clear_Errors(uint8_t target_node_id)
{
    return send_command(target_node_id, CAN_CMD_CLEAR_FAULTS, NULL, 0);
}

HAL_StatusTypeDef CAN_Master_Send_FeedbackRates(uint8_t target_node_id, uint16_t heartbeat_hz, uint16_t encoder_hz)
{
    uint16_t rates[] = {heartbeat_hz, encoder_hz};
    return send_command(target_node_id, CAN_CMD_SET_FEEDBACK_RATES, rates, sizeof(rates));
}

HAL_StatusTypeDef CAN_Master_Send_CurrentLimit(uint8_t target_node_id, float current_limit_A)
{
    return send_command(target_node_id, CAN_CMD_SET_CURRENT_LIMIT, &current_limit_A, sizeof(current_limit_A));
}

const char *CAN_Master_FaultName(unsigned bit)
{
    static const char *const names[] = {
        "Driver fault", "Driver gate fault", "Driver overcurrent",
        "Driver overtemperature", "Driver undervoltage", "Motor overcurrent",
        "Motor overtemperature", "MOSFET overtemperature", "VBUS overvoltage",
        "VBUS undervoltage", "Encoder fault", "Control loop fault",
        "Invalid measurements", "CAN communication timeout",
    };
    return bit < sizeof(names) / sizeof(names[0]) ? names[bit] : "Unknown fault";
}

#endif // DEVICE_IS_MASTER
