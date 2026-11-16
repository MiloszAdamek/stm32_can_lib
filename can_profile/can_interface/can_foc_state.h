#ifndef STM32_CAN_FOC_STATE_H
#define STM32_CAN_FOC_STATE_H

#include <stdbool.h>
#include <stdint.h>

/* Native FOC heartbeat values. State requests use CAN_REQUEST_*. */
typedef enum {
    CAN_FOC_IDLE = 0,
    CAN_FOC_ALIGNMENT = 1,
    CAN_FOC_TORQUE = 2,
    CAN_FOC_RUN = 3,
    CAN_FOC_SPEED = 4,
    CAN_FOC_POSITION = 5,
    CAN_FOC_FAULT = 6,
} CAN_FOC_State_t;

static inline bool CAN_FOC_IsKnown(uint16_t state)
{
    return state <= CAN_FOC_FAULT;
}

static inline bool CAN_FOC_IsClosedLoop(uint16_t state)
{
    return state >= CAN_FOC_TORQUE && state <= CAN_FOC_POSITION;
}

#endif
