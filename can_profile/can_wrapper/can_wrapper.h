// can_wrapper.h

#ifndef INC_CAN_WRAPPER_H_
#define INC_CAN_WRAPPER_H_

#include "main.h"
#include <stdint.h>

typedef struct {
    uint32_t Identifier; // Tylko standardowe ID (11-bit)
    uint8_t  DataLength;
} CAN_Wrapper_TxHeader_t;

typedef struct {
    uint32_t Identifier; // Tylko standardowe ID (11-bit)
    uint8_t  DataLength;
} CAN_Wrapper_RxHeader_t;

// Wskaźnik na funkcję zwrotną (callback) dla odebranych wiadomości
typedef void (*CAN_RxCallback_t)(const CAN_Wrapper_RxHeader_t* pHeader, const uint8_t* pData);

/**
 * @brief Inicjalizuje wrapper, przechowując wskaźnik do właściwego uchwytu HAL.
 * @param hcan_void Wskaźnik na uchwyt FDCAN_HandleTypeDef.
 */
void CAN_Wrapper_Init(void* hcan_void);

/**
 * @brief Rejestruje funkcję zwrotną, która będzie wywoływana po otrzymaniu nowej wiadomości.
 */
void CAN_Wrapper_RegisterRxCallback(CAN_RxCallback_t callback);

/**
 * @brief Konfiguruje filtr CAN. Dla ODrive potrzebujemy filtrowania po Node ID.
 * @param node_id ID węzła (0-63), dla którego ramki mają być akceptowane.
 */
void CAN_Wrapper_ConfigFilter_ODrive(uint32_t node_id);

/**
 * @brief Konfiguruje filtr CAN tak, aby akceptował wszystkie ramki.
 */
void CAN_Wrapper_ConfigFilter_AcceptAll(void);

/**
 * @brief Uruchamia peryferium CAN i aktywuje przerwania.
 */
HAL_StatusTypeDef CAN_Wrapper_Start(void);

/**
 * @brief Wysyła ramkę CAN.
 * @param pHeader Wskaźnik do generycznego nagłówka TX.
 * @param pData Wskaźnik do bufora danych.
 */
HAL_StatusTypeDef CAN_Wrapper_Transmit(CAN_Wrapper_TxHeader_t* pHeader, uint8_t* pData);


#endif /* INC_CAN_WRAPPER_H_ */
