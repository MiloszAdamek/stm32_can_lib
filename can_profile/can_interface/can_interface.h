// can_odrive.h

#ifndef INC_CAN_ODRIVE_H_
#define INC_CAN_ODRIVE_H_

#include <stdint.h>
#include <stdbool.h>
#include "can_wrapper.h"

// =================================================================================
// INSTRUKCJA KONFIGURACJI
// =================================================================================
// Aby wybrać tryb pracy modułu, odkomentuj jedną z poniższych linii
// w swoim pliku main.c (lub w opcjach kompilatora, np. -DDEVICE_IS_MASTER)
// PRZED dołączeniem tego pliku.
//
// Przykład w main.c:
// #define DEVICE_IS_MASTER
// #include "can_odrive.h"
//
// #define DEVICE_IS_MASTER
// #define DEVICE_IS_SLAVE
// =================================================================================

/**
 * @brief Makro do tworzenia standardowego 11-bitowego identyfikatora ramki CAN
 *        zgodnie ze schematem ODrive.
 * @param node_id ID osi/węzła (6 bitów, 0-63).
 * @param cmd_id ID komendy (5 bitów, 0-31).
 */
#define ODRIVE_MAKE_CAN_ID(node_id, cmd_id) (((uint32_t)(node_id) << 5) | (uint32_t)(cmd_id))

/**
 * @brief Definicje identyfikatorów komend protokołu ODrive CAN.
 */
typedef enum
{
    ODRIVE_HEARTBEAT_MESSAGE = 0x01,
    ODRIVE_GET_MOTOR_ERROR = 0x03,
    ODRIVE_GET_ENCODER_ERROR = 0x04,
    ODRIVE_SET_AXIS_REQUESTED_STATE = 0x07,
    ODRIVE_SET_LIMITS = 0x08,
    ODRIVE_GET_ENCODER_ESTIMATES = 0x09,
    ODRIVE_SET_CONTROLLER_MODES = 0x0B,
    ODRIVE_SET_INPUT_POS = 0x0C,
    ODRIVE_SET_INPUT_VEL = 0x0D,
    ODRIVE_SET_INPUT_TORQUE = 0x0E,
    ODRIVE_SET_HEARTBEAT_FREQ = 0x0F,
    ODRIVE_GET_IQ = 0x14,
    ODRIVE_REBOOT = 0x16,
    ODRIVE_CLEAR_ERRORS = 0x18,
} ODrive_CAN_Commands_t;

/**
 * @anchor fault_codes
 * @name Fault codes
 * The symbols below define the codes associated to the faults that the
 * Motor Control subsystem can raise.
 * @{ */
#define MC_NO_ERROR ((uint16_t)0x0000)   /**< @brief No error. */
#define MC_NO_FAULTS ((uint16_t)0x0000)  /**< @brief No error. */
#define MC_DURATION ((uint16_t)0x0001)   /**< @brief Error: FOC rate to high. */
#define MC_OVER_VOLT ((uint16_t)0x0002)  /**< @brief Error: Software over voltage. */
#define MC_UNDER_VOLT ((uint16_t)0x0004) /**< @brief Error: Software under voltage. */
#define MC_OVER_TEMP ((uint16_t)0x0008)  /**< @brief Error: Software over temperature. */
#define MC_START_UP ((uint16_t)0x0010)   /**< @brief Error: Startup failed. */
#define MC_SPEED_FDBK ((uint16_t)0x0020) /**< @brief Error: Speed feedback. */
#define MC_OVER_CURR ((uint16_t)0x0040)  /**< @brief Error: Emergency input (Over current). */
#define MC_SW_ERROR ((uint16_t)0x0080)   /**< @brief Software Error. */
#define MC_DP_FAULT ((uint16_t)0x0400)   /**< @brief Error Driver protection fault. */

/**
 * @brief Definicje stanów osi ODrive.
 */
typedef enum
{
    AXIS_STATE_UNDEFINED = 0,
    AXIS_STATE_IDLE = 1,
    AXIS_STATE_STARTUP_SEQUENCE = 2,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    AXIS_STATE_MOTOR_CALIBRATION = 4,
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
} ODrive_Axis_State_t;

/**
 * @brief Definicje trybów sterowania ODrive.
 */
typedef enum
{
    CONTROL_MODE_VOLTAGE_CONTROL = 0,
    CONTROL_MODE_TORQUE_CONTROL = 1,
    CONTROL_MODE_VELOCITY_CONTROL = 2,
    CONTROL_MODE_POSITION_CONTROL = 3,
} ODrive_Control_Mode_t;

/**
 * @brief Definicja wskaźnika na funkcję zwrotną (callback) dla trybu MASTER.
 *        Funkcja ta będzie wywoływana za każdym razem, gdy MASTER odbierze
 *        ramkę CAN (np. heartbeat lub odpowiedź na żądanie).
 * @param pHeader Wskaźnik do generycznej struktury nagłówka odebranej ramki.
 * @param pData Wskaźnik do bufora z danymi odebranej ramki (do 8 bajtów).
 */
typedef void (*CAN_Master_Rx_Callback_t)(const CAN_Wrapper_RxHeader_t *pHeader, const uint8_t *pData);

/**
 * @brief Funkcja do załączenia terminatora 120Ohm
 * @param enable, 1 - terminator aktywny, 2 - terminator wyłaczany
 */
void enableTerminator(bool enable);

/**
 * @brief Funkcja do wysyłania ramki CAN
 * @param can_id ID ramki CAN
 * @param data Wskaźnik do bufora z danymi
 * @param len Długość danych
 */
static inline void send_can_frame(uint32_t can_id, uint8_t *data, uint8_t len);

// --- API dla trybu SLAVE ---
#if defined(DEVICE_IS_SLAVE)

/**
 * @brief (SLAVE) Inicjalizuje peryferium CAN w trybie Slave.
 *        Konfiguruje filtr, aby akceptować tylko ramki zaadresowane do tego węzła.
 * @param hcam_void uchwyt do hfdcan/hcan
 * @param my_axis_id ID tej osi (węzła), wartość od 0 do 63.
 */
void CAN_Slave_Init(void *hcan_void, uint8_t my_axis_id);

void CAN_Slave_Heartbeat(void);

void CAN_Slave_Telemetry(void);

#endif // DEVICE_IS_SLAVE

// --- API dla trybu MASTER ---
#if defined(DEVICE_IS_MASTER)

/**
 * @brief (MASTER) Inicjalizuje peryferium CAN w trybie Master.
 *        Konfiguruje filtr tak, aby akceptował wszystkie przychodzące ramki.
 */
void CAN_Master_Init(void);

/**
 * @brief (MASTER) Rejestruje funkcję zwrotną (callback) do obsługi odebranych ramek.
 * @param callback Wskaźnik do funkcji, która będzie wywoływana po odebraniu ramki.
 */
void CAN_Master_SetRxCallback(CAN_Master_Rx_Callback_t callback);

/**
 * @brief (MASTER) Ustawia stan docelowej osi.
 * @param target_node_id ID węzła docelowego.
 * @param state Wartość stanu z enum ODrive_Axis_State_t.
 */
void CAN_Master_Send_State(uint8_t target_node_id, int32_t state);

/**
 * @brief (MASTER) Ustawia docelowy moment obrotowy.
 * @param target_node_id ID węzła docelowego.
 * @param torque Moment obrotowy w [A].
 */
void CAN_Master_Send_Torque(uint8_t target_node_id, float torque);

/**
 * @brief (MASTER) Ustawia docelową prędkość.
 * @param target_node_id ID węzła docelowego.
 * @param velocity Prędkość w [RPM].
 */
void CAN_Master_Send_Velocity(uint8_t target_node_id, float velocity);

/**
 * @brief (MASTER) Ustawia tryby sterowania.
 * @param target_node_id ID węzła docelowego.
 * @param control_mode Tryb sterowania (z enum ODrive_Control_Mode_t).
 * @param input_mode Tryb wejścia (zazwyczaj 1 - INPUT_MODE_PASSTHROUGH).
 */
void CAN_Master_Send_Modes(uint8_t target_node_id, int32_t control_mode, int32_t input_mode);

/**
 * @brief (MASTER) Wysyła komendę restartu do węzła.
 * @param target_node_id ID węzła docelowego.
 */
void CAN_Master_Send_Reboot(uint8_t target_node_id);

/**
 * @brief (MASTER) Wysyła komendę kasowania błędów do węzła.
 * @param target_node_id ID węzła docelowego.
 */
void CAN_Master_Send_Clear_Errors(uint8_t target_node_id);

/**
 * @brief (MASTER) Wysyła żądanie o estymaty z enkodera (pozycja, prędkość).
 * @param target_node_id ID węzła docelowego.
 */
void CAN_Master_Request_Encoder_Estimates(uint8_t target_node_id);

/**
 * @brief (MASTER) Wysyła żądanie o wartości prądów Iq (zadany i mierzony).
 * @param target_node_id ID węzła docelowego.
 */
void CAN_Master_Request_IQ(uint8_t target_node_id);

/**
 * @brief (MASTER) Ustawia parametr Slave'a - częstotliwosc wysyłania ramek heartbeat i telemetry
 * @param target_node_id ID węzła docelowego.
 */
void CAN_Master_Send_HeartbeatFreq(uint8_t target_node_id, uint16_t heartbeat_freq, uint16_t telemetry_freq);

/**
 * @brief (MASTER) Ustawia parametr Slave'a - limit predkosci i momentu
 * @param target_node_id ID węzła docelowego.
 */
void CAN_Master_Send_Limits(uint8_t target_node_id, uint16_t speed_limit, uint16_t torque_limit);

#endif // DEVICE_IS_MASTER

#endif /* INC_CAN_ODRIVE_H_ */
