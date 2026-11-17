#ifndef STM32_CAN_RTOS_H
#define STM32_CAN_RTOS_H

#include "can_interface.h"
#include "FreeRTOS.h"

typedef struct {
    uint16_t heartbeat_hz, encoder_hz;
    float current_limit_A;
    uint32_t heartbeat_timeout_ms;
} CAN_RTOS_Config_t;

typedef struct {
    TickType_t period; /* Nonzero task polling interval. */
    UBaseType_t priority;
    configSTACK_DEPTH_TYPE stack_depth;
    void (*on_setup)(uint8_t node_id, HAL_StatusTypeDef status);
    void (*on_faults)(uint64_t fault_nodes); /* One bit per CAN node ID. */
} CAN_RTOS_TaskConfig_t;

/* Open once after initializing the board HAL. Keep the node table alive.
 * StartTask configures the nodes and runs supervision in a single task. */
HAL_StatusTypeDef CAN_RTOS_Open(FDCAN_HandleTypeDef *bus, CAN_Master_Node_t *nodes,
                               size_t count, CAN_RTOS_Config_t config);
BaseType_t CAN_RTOS_StartTask(CAN_RTOS_TaskConfig_t task);

/* Configure runs in task context and delays briefly between setup frames. */
HAL_StatusTypeDef CAN_RTOS_Configure(uint8_t node_id, float current_limit_A);
HAL_StatusTypeDef CAN_RTOS_SetCurrentLimit(uint8_t node_id, float amps);
HAL_StatusTypeDef CAN_RTOS_Reboot(uint8_t node_id);
HAL_StatusTypeDef CAN_RTOS_Arm(uint8_t node_id);
HAL_StatusTypeDef CAN_RTOS_Stop(uint8_t node_id);
uint64_t CAN_RTOS_ArmedNodes(void);
bool CAN_RTOS_IsConfigured(uint8_t node_id);
bool CAN_RTOS_IsReady(uint8_t node_id);
bool CAN_RTOS_IsOnline(uint8_t node_id, uint32_t timeout_ms);
bool CAN_RTOS_HasFault(uint8_t node_id, uint32_t timeout_ms);

#endif
