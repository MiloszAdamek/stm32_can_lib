// can_interface.c

#include "can_interface.h"
#include "can_wrapper.h"
#include <string.h>

CAN_Heartbeat_t CAN_DecodeHeartbeat(const uint8_t *data)
{
    CAN_Heartbeat_t heartbeat;
    memcpy(&heartbeat.state, data, sizeof(heartbeat.state));
    memcpy(&heartbeat.faults, data + sizeof(heartbeat.state), sizeof(heartbeat.faults));
    return heartbeat;
}

CAN_TelemetrySnapshot_t CAN_DecodeEncoderFeedback(const uint8_t *data)
{
    CAN_TelemetrySnapshot_t encoder;
    memcpy(&encoder.pos_rev, data, sizeof(encoder.pos_rev));
    memcpy(&encoder.vel_rpm, data + sizeof(encoder.pos_rev), sizeof(encoder.vel_rpm));
    return encoder;
}

/**
 * @brief Wewnętrzna funkcja do wysyłania ramek CAN za pomocą wrappera.
 */
HAL_StatusTypeDef send_can_frame(uint32_t can_id, const void *data, uint8_t len)
{
    if (can_id > 0x7ff || len > 8 || (len && !data))
        return HAL_ERROR;

    CAN_Wrapper_TxHeader_t header = {can_id, len};
    uint8_t payload[8] = {0};
    if (len) memcpy(payload, data, len);
    return CAN_Wrapper_Transmit(&header, payload);
}

/**
 * @brief Funkcja do załączenia terminatora 120Ohm
 * @param enable, 1 - terminator aktywny, 2 - terminator wyłaczany
 */
void enableTerminator(bool enable)
{
    if (enable)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // Włącz
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Wyłącz
    }
}
