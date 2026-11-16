#ifndef STM32_CAN_INTERFACE_H
#define STM32_CAN_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "can_foc_state.h"
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
// #include "can_interface.h"
//
// #define DEVICE_IS_MASTER
// #define DEVICE_IS_SLAVE
// =================================================================================

typedef struct {
    float pos_rev;
    float vel_rpm;
} CAN_TelemetrySnapshot_t;

typedef struct {
    uint16_t state;
    uint16_t faults;
} CAN_Heartbeat_t;

/* Call with the payload of the corresponding Classic CAN frame. */
CAN_Heartbeat_t CAN_DecodeHeartbeat(const uint8_t *data);
CAN_TelemetrySnapshot_t CAN_DecodeEncoderFeedback(const uint8_t *data);

/* Standard 11-bit identifier with a 6-bit node and a 5-bit command. */
#define CAN_NODE_ID_MAX 0x3fu
#define CAN_COMMAND_BITS 5u
#define CAN_MAKE_ID(node, command) (((uint32_t)(node) << CAN_COMMAND_BITS) | (uint32_t)(command))
#define CAN_ID_NODE(id) ((uint32_t)(id) >> CAN_COMMAND_BITS)
#define CAN_ID_COMMAND(id) ((uint32_t)(id) & ((1u << CAN_COMMAND_BITS) - 1u))

typedef enum
{
    CAN_CMD_HEARTBEAT = 0x01,
    CAN_CMD_REQUEST_STATE = 0x07,
    CAN_CMD_SET_CURRENT_LIMIT = 0x08,
    CAN_CMD_ENCODER_FEEDBACK = 0x09,
    CAN_CMD_SET_VELOCITY = 0x0D,
    CAN_CMD_SET_TORQUE = 0x0E,
    CAN_CMD_SET_FEEDBACK_RATES = 0x0F,
    CAN_CMD_REBOOT = 0x16,
    CAN_CMD_SET_WATCHDOG_TIMEOUT = 0x17,
    CAN_CMD_CLEAR_FAULTS = 0x18,
} CAN_Command_t;

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

// Kody błędów naszego sterownika

#define MOTOR_FAULT_NONE                    0U

/* Bit 0–4: power stage */
#define MOTOR_FAULT_DRIVER                  (1UL << 0)
#define MOTOR_FAULT_DRIVER_GATE             (1UL << 1)
#define MOTOR_FAULT_DRIVER_OVERCURRENT      (1UL << 2)
#define MOTOR_FAULT_DRIVER_OVERTEMP         (1UL << 3)
#define MOTOR_FAULT_DRIVER_UNDERVOLTAGE     (1UL << 4)

/* Bit 5–12: control loop */
#define MOTOR_FAULT_OVERCURRENT             (1UL << 5)
#define MOTOR_FAULT_MOTOR_OVERTEMP          (1UL << 6)
#define MOTOR_FAULT_MOSFET_OVERTEMP         (1UL << 7)
#define MOTOR_FAULT_VBUS_OVERVOLTAGE        (1UL << 8)
#define MOTOR_FAULT_VBUS_UNDERVOLTAGE       (1UL << 9)
#define MOTOR_FAULT_ENCODER                 (1UL << 10)
#define MOTOR_FAULT_CONTROL                 (1UL << 11)
#define MOTOR_FAULT_INVALID_MEASUREMENTS    (1UL << 12)


/* State request payload values. Heartbeats carry CAN_FOC_State_t. */
typedef enum
{
    CAN_REQUEST_STOP = 1,
    CAN_REQUEST_START = 8,
} CAN_StateRequest_t;

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
HAL_StatusTypeDef send_can_frame(uint32_t can_id, const void *data, uint8_t len);

// --- API dla trybu SLAVE ---
#if defined(DEVICE_IS_SLAVE)


/// @brief Struktura z callbackami do obsługi zdarzeń CAN w trybie SLAVE związanych ze sterowaniem silnikiem. Umożliwia integrację z własnym kodem sterowania silnikiem.
typedef struct
{
    void (*start)(void);
    void (*stop)(void);

    void (*set_speed)(float rpm);
    void (*set_torque)(float iq);

    void (*reboot)(void);

    void (*get_heartbeat)(
        uint16_t *axis_state,
        uint16_t *axis_error);

    void (*get_telemetry)(
        float *position_rev,
        float *velocity_rpm);

    void (*clear_faults)(void);

    void (*get_faults)(uint16_t *faults);

    void (*set_speed_limit)(float speed_limit_rpm);

    void (*set_torque_limit_A)(float torque_limit_A);

    void (*set_torque_limit_Nm)(float torque_limit_Nm);

    void (*set_watchdog_timeout)(uint16_t timeout_ms);
} CAN_Slave_Callbacks_t;

/**
 * @brief (SLAVE) Inicjalizuje peryferium CAN w trybie Slave.
 *        Konfiguruje filtr, aby akceptować tylko ramki zaadresowane do tego węzła.
 * @param hcam_void uchwyt do hfdcan/hcan
 * @param my_axis_id ID tej osi (węzła), wartość od 0 do 63.
 * @param callbacks Wskaźnik do struktury z callbackami.
 */
void CAN_Slave_Init(void *hcan_void, uint8_t my_axis_id, const CAN_Slave_Callbacks_t *callbacks);

void CAN_Slave_Heartbeat(void);

void CAN_Slave_Telemetry(void);

void CAN_Slave_PrintLastCommand(void);

/**
 * @brief (SLAVE) Rejestruje callbacki do obsługi zdarzeń CAN związanych ze sterowaniem silnikiem.
 * @param callbacks Wskaźnik do struktury z callbackami.
 */
void CAN_Slave_RegisterCallbacks(const CAN_Slave_Callbacks_t *callbacks);

#endif // DEVICE_IS_SLAVE

// --- API dla trybu MASTER ---
#if defined(DEVICE_IS_MASTER)

typedef struct {
    CAN_TelemetrySnapshot_t encoder;
    uint32_t heartbeat_ms, encoder_ms;
    uint16_t state, faults; /* Raw heartbeat fields. See CAN_FOC_State_t. */
    bool heartbeat_seen, encoder_valid;
} CAN_Master_Feedback_t;

typedef struct {
    uint32_t node_id;
    /* The library manages feedback. */
    CAN_Master_Feedback_t feedback;
} CAN_Master_Node_t;

/* Configure the bus timing and message RAM before opening. Keep nodes alive
 * until CAN stops and exclude the receive ISR when calling from task context.
 * One bus is supported. */
HAL_StatusTypeDef CAN_Master_Open(FDCAN_HandleTypeDef *bus, CAN_Master_Node_t *nodes, size_t count);
/* Access the caller-owned node record while excluding the receive ISR. */
CAN_Master_Node_t *CAN_Master_FindNode(uint32_t node_id);
bool CAN_Master_GetFeedback(uint32_t node_id, CAN_Master_Feedback_t *out, uint32_t max_encoder_age_ms);
const char *CAN_Master_FaultName(unsigned bit);

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
 * @param state Wartość stanu z enum CAN_StateRequest_t.
 */
HAL_StatusTypeDef CAN_Master_Send_State(uint8_t target_node_id, int32_t state);

/**
 * @brief (MASTER) Ustawia docelowy moment obrotowy.
 * @param target_node_id ID węzła docelowego.
 * @param torque Surowa wartość zadana w jednostkach używanych przez dany ESC.
 */
HAL_StatusTypeDef CAN_Master_Send_Torque(uint8_t target_node_id, float torque);

/**
 * @brief (MASTER) Ustawia docelową prędkość.
 * @param target_node_id ID węzła docelowego.
 * @param velocity Prędkość w [RPM].
 */
HAL_StatusTypeDef CAN_Master_Send_Velocity(uint8_t target_node_id, float velocity);

/**
 * @brief (MASTER) Wysyła komendę restartu do węzła.
 * @param target_node_id ID węzła docelowego.
 */
HAL_StatusTypeDef CAN_Master_Send_Reboot(uint8_t target_node_id);
HAL_StatusTypeDef CAN_Master_Send_WatchdogTimeout(uint8_t target_node_id, uint16_t timeout_ms);

/**
 * @brief (MASTER) Wysyła komendę kasowania błędów do węzła.
 * @param target_node_id ID węzła docelowego.
 */
HAL_StatusTypeDef CAN_Master_Send_Clear_Errors(uint8_t target_node_id);

/**
 * @brief (MASTER) Ustawia parametr Slave'a - częstotliwosc wysyłania ramek heartbeat i telemetry
 * @param target_node_id ID węzła docelowego.
 */
HAL_StatusTypeDef CAN_Master_Send_FeedbackRates(uint8_t target_node_id, uint16_t heartbeat_hz, uint16_t encoder_hz);

/**
 * @brief (MASTER) Ustawia limit prądu w amperach.
 * @param target_node_id ID węzła docelowego.
 * @param current_limit_A Limit prądu w [A].
 */
HAL_StatusTypeDef CAN_Master_Send_CurrentLimit(uint8_t target_node_id, float current_limit_A);

#endif // DEVICE_IS_MASTER

#endif /* STM32_CAN_INTERFACE_H */
