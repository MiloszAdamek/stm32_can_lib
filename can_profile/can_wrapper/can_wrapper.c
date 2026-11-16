// can_wrapper.c

#include "can_wrapper.h"
#include "can_interface.h"
#include <stdio.h>

// Zmienne prywatne modułu
static void *hcan_ptr = NULL;
static CAN_RxCallback_t g_rx_callback = NULL;
static bool recover_bus_off = true;

void CAN_Wrapper_Init(void *hcan_void)
{
    hcan_ptr = hcan_void;
    recover_bus_off = true;
}

void CAN_Wrapper_SetBusOffRecovery(bool enabled)
{
    recover_bus_off = enabled;
}

void CAN_Wrapper_RegisterRxCallback(CAN_RxCallback_t callback)
{
    g_rx_callback = callback;
}

HAL_StatusTypeDef CAN_Wrapper_ConfigFilter(uint32_t id, uint32_t mask)
{
    if (!hcan_ptr) return HAL_ERROR;
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = id;
    sFilterConfig.FilterID2 = mask;
    return HAL_FDCAN_ConfigFilter((FDCAN_HandleTypeDef *)hcan_ptr, &sFilterConfig);
}

void CAN_Wrapper_ConfigNodeFilter(uint32_t node_id)
{
    if (CAN_Wrapper_ConfigFilter(CAN_MAKE_ID(node_id, 0), CAN_MAKE_ID(CAN_NODE_ID_MAX, 0)) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_Wrapper_ConfigFilter_AcceptAll(void)
{
    if (CAN_Wrapper_ConfigFilter(0, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef CAN_Wrapper_Start(void)
{
    FDCAN_HandleTypeDef *bus = hcan_ptr;
    if (!bus || HAL_FDCAN_Start(bus) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef result = HAL_FDCAN_ActivateNotification(bus, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    if (result != HAL_OK) HAL_FDCAN_Stop(bus);
    return result;
}

HAL_StatusTypeDef CAN_Wrapper_Transmit(CAN_Wrapper_TxHeader_t *pHeader, uint8_t *pData)
{
    if (!hcan_ptr || !pHeader || pHeader->DataLength > 8) return HAL_ERROR;
    FDCAN_TxHeaderTypeDef TxHeader;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)hcan_ptr;
    FDCAN_ProtocolStatusTypeDef ProtocolStatus;

    // 1. Sprawdzenie stanu Bus-Off
    if (HAL_FDCAN_GetProtocolStatus(hfdcan, &ProtocolStatus) != HAL_OK) return HAL_ERROR;
    if (ProtocolStatus.BusOff)
    {
        // Pełna synchronizacja stanu HAL zamiast samego czyszczenia bitu
        if (recover_bus_off)
        {
            HAL_FDCAN_Stop(hfdcan);
            HAL_FDCAN_Start(hfdcan);
        }
        return HAL_ERROR;
    }

    // 2. Transmisja Non-Blocking
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        TxHeader.Identifier = pHeader->Identifier;
        TxHeader.IdType = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;
        
        // Classic CAN DLC constants are consecutive codes, shifted on some HAL versions.
        TxHeader.DataLength = pHeader->DataLength * FDCAN_DLC_BYTES_1;

        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker = 0;

        return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pData);
    }
    else
    {
        return HAL_BUSY;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!hcan_ptr || hcan_ptr != hfdcan)
        return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[64]; // HAL copies the payload before we validate the header.

        // Pętla drenująca - opróżnia całe FIFO z oczekujących ramek falownika
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
        {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
            {
                if (g_rx_callback &&
                    RxHeader.IdType == FDCAN_STANDARD_ID && 
                    RxHeader.Identifier <= 0x7ff &&
                    RxHeader.RxFrameType == FDCAN_DATA_FRAME &&
                    RxHeader.FDFormat == FDCAN_CLASSIC_CAN &&
                    RxHeader.DataLength <= FDCAN_DLC_BYTES_8 &&
                    RxHeader.DataLength % FDCAN_DLC_BYTES_1 == 0)
                {
                    CAN_Wrapper_RxHeader_t wrapperHeader;
                    wrapperHeader.Identifier = RxHeader.Identifier;
                    wrapperHeader.DataLength = RxHeader.DataLength / FDCAN_DLC_BYTES_1;

                    // Legacy slave callbacks use an eight-byte buffer even for short frames.
                    for (uint8_t i = wrapperHeader.DataLength; i < 8; ++i) RxData[i] = 0;

                    g_rx_callback(&wrapperHeader, RxData);
                }
            }
            else
            {
                break;
            }
        }
    }
}
