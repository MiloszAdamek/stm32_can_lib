// can_wrapper.c

#include "can_wrapper.h"
#include "main.h"
#include <stdio.h>

// Zmienne prywatne modułu
static void *hcan_ptr = NULL;
static CAN_RxCallback_t g_rx_callback = NULL;

void CAN_Wrapper_Init(void *hcan_void)
{
    hcan_ptr = hcan_void;
}

void CAN_Wrapper_RegisterRxCallback(CAN_RxCallback_t callback)
{
    g_rx_callback = callback;
}

void CAN_Wrapper_ConfigFilter_ODrive(uint32_t node_id)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = (node_id << 5); // ID do porównania
    sFilterConfig.FilterID2 = (0x3F << 5);    // Maska na bity Node ID (bity 10..5)

    if (HAL_FDCAN_ConfigFilter((FDCAN_HandleTypeDef *)hcan_ptr, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Wrapper_ConfigFilter_AcceptAll(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x0;
    sFilterConfig.FilterID2 = 0x0; // Maska 0 = akceptuj wszystko

    if (HAL_FDCAN_ConfigFilter((FDCAN_HandleTypeDef *)hcan_ptr, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef CAN_Wrapper_Start(void)
{
    if (HAL_FDCAN_Start((FDCAN_HandleTypeDef *)hcan_ptr) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_FDCAN_ActivateNotification((FDCAN_HandleTypeDef *)hcan_ptr, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

HAL_StatusTypeDef CAN_Wrapper_Transmit(CAN_Wrapper_TxHeader_t *pHeader, uint8_t *pData)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)hcan_ptr;
    FDCAN_ProtocolStatusTypeDef ProtocolStatus;

    // 1. Check for "Bus Off" state (Caused by Motor Noise)
    // -----------------------------------------------------
    HAL_FDCAN_GetProtocolStatus(hfdcan, &ProtocolStatus);

    if (ProtocolStatus.BusOff)
    {
        // The hardware has disconnected itself to save the bus.
        // We must manually tell it to try rejoining.
        // Clearing the INIT bit in CCCR exits the initialization mode.
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);

        // Return Error so we don't try to queue a packet into a dead peripheral
        return HAL_ERROR;
    }

    // 2. Check FIFO Level NON-BLOCKING
    // -----------------------------------------------------
    // Replaced 'while' with 'if'. If FIFO is full, we DROP the packet.
    // It is better to lose one balancing packet than to freeze the CPU.
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        // Zakładamy zawsze Standard ID i Data Frame
        TxHeader.Identifier = pHeader->Identifier;
        TxHeader.IdType = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;

        TxHeader.DataLength = pHeader->DataLength;

        // Stałe parametry dla trybu Classic CAN
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker = 0;

        return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pData);
    }
    else
    {
        // FIFO is full. This usually means noise is preventing transmission
        // and retries are clogging the pipe.
        // Return BUSY so the main loop knows we failed, but KEEPS RUNNING.
        return HAL_BUSY;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hcan_ptr != hfdcan)
        return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            if (g_rx_callback && RxHeader.IdType == FDCAN_STANDARD_ID && RxHeader.RxFrameType == FDCAN_DATA_FRAME)
            {
                CAN_Wrapper_RxHeader_t wrapperHeader;
                wrapperHeader.Identifier = RxHeader.Identifier;
                // HAL_FDCAN_GetRxMessage zwraca już zdekodowaną wartość.
                wrapperHeader.DataLength = RxHeader.DataLength;

                // Wywołaj callback wyższego poziomu (np. odrive_rx_callback)
                g_rx_callback(&wrapperHeader, RxData);
            }
        }
    }
}
