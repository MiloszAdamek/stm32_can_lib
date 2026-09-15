// can_interface.c

#include "can_interface.h"
#include "can_wrapper.h"

/**
 * @brief Wewnętrzna funkcja do wysyłania ramek CAN za pomocą wrappera.
 */
inline void send_can_frame(uint32_t can_id, uint8_t *data, uint8_t len)
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
