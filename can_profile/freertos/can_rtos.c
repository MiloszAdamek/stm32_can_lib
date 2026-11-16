#include "can_rtos.h"
#include "task.h"

static struct {
    CAN_Master_Node_t *nodes;
    size_t count;
    CAN_RTOS_Config_t config;
    CAN_RTOS_TaskConfig_t task;
    uint64_t configuring, configured, armed, pending_stop;
} service;

static uint64_t node_bit(uint8_t node_id)
{
    return node_id <= CAN_NODE_ID_MAX ? UINT64_C(1) << node_id : 0;
}

static bool feedback_online(const CAN_Master_Feedback_t *feedback, uint32_t timeout_ms)
{
    return feedback->heartbeat_seen && HAL_GetTick() - feedback->heartbeat_ms <= timeout_ms;
}

static bool feedback_faulted(const CAN_Master_Feedback_t *feedback, uint32_t timeout_ms)
{
    return !feedback_online(feedback, timeout_ms) || feedback->faults ||
           feedback->state == CAN_FOC_FAULT || !CAN_FOC_IsKnown(feedback->state);
}

HAL_StatusTypeDef CAN_RTOS_Open(FDCAN_HandleTypeDef *bus, CAN_Master_Node_t *nodes,
                               size_t count, CAN_RTOS_Config_t config)
{
    service.count = 0;
    service.configuring = service.configured = service.armed = service.pending_stop = 0;
    HAL_StatusTypeDef status = CAN_Master_Open(bus, nodes, count);
    if (status == HAL_OK) {
        service.nodes = nodes;
        service.count = count;
        service.config = config;
    }
    return status;
}

static HAL_StatusTypeDef stop(uint8_t node_id)
{
    uint64_t bit = node_bit(node_id);
    service.armed &= ~bit;
    HAL_StatusTypeDef status = CAN_Master_Send_State(node_id, CAN_REQUEST_STOP);
    if (status == HAL_OK) service.pending_stop &= ~bit;
    else service.pending_stop |= bit;
    return status;
}

HAL_StatusTypeDef CAN_RTOS_Stop(uint8_t node_id)
{
    taskENTER_CRITICAL();
    HAL_StatusTypeDef status = stop(node_id);
    taskEXIT_CRITICAL();
    return status;
}

HAL_StatusTypeDef CAN_RTOS_Configure(uint8_t node_id, float current_limit_A)
{
    CAN_Master_Node_t *node = CAN_Master_FindNode(node_id);
    if (!node) return HAL_ERROR;
    uint64_t bit = node_bit(node_id);

    taskENTER_CRITICAL();
    if ((service.armed | service.configuring) & bit) {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    service.configuring |= bit;
    service.configured &= ~bit;
    HAL_StatusTypeDef status = stop(node_id);
    taskEXIT_CRITICAL();

    const TickType_t delay = pdMS_TO_TICKS(2) ? pdMS_TO_TICKS(2) : 1;
    if (status == HAL_OK) {
        vTaskDelay(delay);
        taskENTER_CRITICAL();
        status = CAN_Master_Send_FeedbackRates(node_id, service.config.heartbeat_hz,
                                               service.config.encoder_hz);
        taskEXIT_CRITICAL();
    }
    if (status == HAL_OK) vTaskDelay(delay);

    taskENTER_CRITICAL();
    if (status == HAL_OK)
        status = CAN_Master_Send_CurrentLimit(node_id, current_limit_A);
    if (status == HAL_OK) {
        node->feedback.heartbeat_seen = false;
        service.configured |= bit;
    }
    service.configuring &= ~bit;
    taskEXIT_CRITICAL();
    return status;
}

HAL_StatusTypeDef CAN_RTOS_SetCurrentLimit(uint8_t node_id, float amps)
{
    taskENTER_CRITICAL();
    HAL_StatusTypeDef status = ((service.armed | service.configuring) & node_bit(node_id))
        ? HAL_BUSY : CAN_Master_Send_CurrentLimit(node_id, amps);
    taskEXIT_CRITICAL();
    return status;
}

HAL_StatusTypeDef CAN_RTOS_Reboot(uint8_t node_id)
{
    uint64_t bit = node_bit(node_id);
    taskENTER_CRITICAL();
    HAL_StatusTypeDef status = ((service.armed | service.configuring) & bit)
        ? HAL_BUSY : CAN_Master_Send_Reboot(node_id);
    if (status == HAL_OK) service.configured &= ~bit;
    taskEXIT_CRITICAL();
    return status;
}

static bool ready(uint8_t node_id)
{
    uint64_t bit = node_bit(node_id);
    if (!(service.configured & bit) || (service.pending_stop & bit))
        return false;

    const CAN_Master_Feedback_t *feedback = &CAN_Master_FindNode(node_id)->feedback;
    return !feedback_faulted(feedback, service.config.heartbeat_timeout_ms) &&
           (feedback->state == CAN_FOC_IDLE || CAN_FOC_IsClosedLoop(feedback->state));
}

HAL_StatusTypeDef CAN_RTOS_Arm(uint8_t node_id)
{
    taskENTER_CRITICAL();
    HAL_StatusTypeDef status = ready(node_id) ? CAN_Master_Send_State(node_id, CAN_REQUEST_START) : HAL_ERROR;
    if (status == HAL_OK) service.armed |= node_bit(node_id);
    taskEXIT_CRITICAL();
    return status;
}

uint64_t CAN_RTOS_ArmedNodes(void)
{
    taskENTER_CRITICAL();
    uint64_t armed = service.armed;
    taskEXIT_CRITICAL();
    return armed;
}

bool CAN_RTOS_IsConfigured(uint8_t node_id)
{
    taskENTER_CRITICAL();
    bool configured = (service.configured & node_bit(node_id)) != 0;
    taskEXIT_CRITICAL();
    return configured;
}

bool CAN_RTOS_IsReady(uint8_t node_id)
{
    taskENTER_CRITICAL();
    bool is_ready = ready(node_id);
    taskEXIT_CRITICAL();
    return is_ready;
}

bool CAN_RTOS_IsOnline(uint8_t node_id, uint32_t timeout_ms)
{
    const CAN_Master_Node_t *node = CAN_Master_FindNode(node_id);
    taskENTER_CRITICAL();
    bool online = node && feedback_online(&node->feedback, timeout_ms);
    taskEXIT_CRITICAL();
    return online;
}

bool CAN_RTOS_HasFault(uint8_t node_id, uint32_t timeout_ms)
{
    const CAN_Master_Node_t *node = CAN_Master_FindNode(node_id);
    taskENTER_CRITICAL();
    bool fault = !node || feedback_faulted(&node->feedback, timeout_ms);
    taskEXIT_CRITICAL();
    return fault;
}

static void monitor_task(void *unused)
{
    (void)unused;
    for (size_t i = 0; i < service.count; ++i) {
        uint8_t id = service.nodes[i].node_id;
        if (CAN_RTOS_IsConfigured(id)) continue;
        HAL_StatusTypeDef status = CAN_RTOS_Configure(id, service.config.current_limit_A);
        if (service.task.on_setup) service.task.on_setup(id, status);
    }

    uint32_t started_ms = HAL_GetTick();
    bool supervise = false;
    for (;;) {
        supervise = supervise || HAL_GetTick() - started_ms >= service.config.heartbeat_timeout_ms;
        bool report_faults = supervise && service.task.on_faults;
        uint64_t faults = 0;
        for (size_t i = 0; i < service.count; ++i) {
            const CAN_Master_Node_t *node = &service.nodes[i];
            uint64_t bit = node_bit(node->node_id);
            taskENTER_CRITICAL();
            if (service.pending_stop & bit) stop(node->node_id);
            if (report_faults && feedback_faulted(&node->feedback, service.config.heartbeat_timeout_ms))
                faults |= bit;
            taskEXIT_CRITICAL();
        }
        if (report_faults) service.task.on_faults(faults);
        vTaskDelay(service.task.period);
    }
}

BaseType_t CAN_RTOS_StartTask(CAN_RTOS_TaskConfig_t task)
{
    service.task = task;
    return xTaskCreate(monitor_task, "CAN Motors", task.stack_depth, NULL, task.priority, NULL);
}
