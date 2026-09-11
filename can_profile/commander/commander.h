#ifndef INC_COMMANDER_H_
#define INC_COMMANDER_H_

#include "main.h" // Potrzebne dla UART_HandleTypeDef

/**
 * @brief Inicjalizuje moduł Commandera.
 * @param huart_void Wskaźnik na uchwyt UART_HandleTypeDef (np. &huart2).
 */
void Commander_Init(void* huart_void);

/**
 * @brief Funkcja do cyklicznego wywoływania w pętli głównej.
 *        Sprawdza, czy nadeszła nowa komenda i ją przetwarza.
 */
void Commander_Process(void);

#endif /* INC_COMMANDER_H_ */
