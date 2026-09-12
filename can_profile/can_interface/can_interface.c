// can_interface.c

#include "can_interface.h"
#include "can_wrapper.h"
#include <string.h>
#include <stdio.h>
#include "math.h"

// --- Zmienne globalne modułu ---

// #if defined(DEVICE_IS_SLAVE)
//     extern MCI_Handle_t *pMCI[NBR_OF_MOTORS];
//     static uint8_t g_my_axis_id = 0;
//     static void CAN_Slave_Process_Rx_Message(uint32_t cmd_id, uint8_t *data);
//     extern volatile uint16_t g_heartbeat_period_ms;
//     extern volatile uint16_t g_telemetry_period_ms;
// #endif

// #if defined(DEVICE_IS_MASTER)
//     static CAN_Master_Rx_Callback_t g_master_rx_callback = NULL;
// #endif

/**
 * @brief Wewnętrzna funkcja do wysyłania ramek CAN za pomocą wrappera.
 */
static inline void send_can_frame(uint32_t can_id, uint8_t *data, uint8_t len)
{
    CAN_Wrapper_TxHeader_t TxHeader;
    if (len > 8)
        return;

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = len;

    if (CAN_Wrapper_Transmit(&TxHeader, data) != HAL_OK)
    {
        // Obsluga bledu transmisji
        // Error_Handler();
    }
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
