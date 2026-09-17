// can_slave.c

#if defined(DEVICE_IS_SLAVE)

// --- Implementacja API dla trybu SLAVE ---

#include "can_interface.h"
#include "can_wrapper.h"
#include <string.h>
#include <stdio.h>

static uint8_t g_my_axis_id = 0;

static CAN_Slave_Callbacks_t g_callbacks;

static void CAN_Slave_Process_Rx_Message(uint32_t cmd_id, uint8_t *data);

volatile bool g_can_heartbeat_flag      = false;
volatile bool g_can_telemetry_flag      = false;
volatile bool g_can_cmd_received_flag   = false;
static uint8_t g_last_cmd_payload[8] = {0};

uint32_t g_last_cmd_id = -1; // Zmienna do debugowania ostatniej komendy CAN

#define ISR_FREQ_HZ 10000U          // 10 kHz

#define HEARTBEAT_FREQ_HZ   10U     // Default 10 Hz
#define TELEMETRY_FREQ_HZ   100U    // Default 100 Hz

#define PERIOD_TICKS(freq_hz)  (ISR_FREQ_HZ / (freq_hz))

volatile uint32_t g_heartbeat_period_ticks = PERIOD_TICKS(HEARTBEAT_FREQ_HZ);
volatile uint32_t g_telemetry_period_ticks = PERIOD_TICKS(TELEMETRY_FREQ_HZ);

/**
 * @brief Callback do odbierania wiadomości od mastera, przekazywany do wrappera CAN.
 */
static void odrive_rx_callback(const CAN_Wrapper_RxHeader_t *pHeader, const uint8_t *pData)
{
    uint32_t axis_id = pHeader->Identifier >> 5;
    if (axis_id == g_my_axis_id)
    {
        uint32_t cmd_id = pHeader->Identifier & 0x1F;
        CAN_Slave_Process_Rx_Message(cmd_id, (uint8_t *)pData);
    }
}

void CAN_Slave_RegisterCallbacks(const CAN_Slave_Callbacks_t *callbacks)
{
    g_callbacks = *callbacks;
}

void CAN_Slave_Init(void *hcan_void, uint8_t my_axis_id, const CAN_Slave_Callbacks_t *callbacks)
{
    if (hcan_void == NULL)
    {
        Error_Handler();
    }

    if (callbacks == NULL)
    {
        Error_Handler();
    }

    CAN_Wrapper_Init(hcan_void);

    g_my_axis_id = my_axis_id & 0x3F;

    CAN_Slave_RegisterCallbacks(callbacks);

    CAN_Wrapper_RegisterRxCallback(odrive_rx_callback);
    CAN_Wrapper_ConfigFilter_ODrive(g_my_axis_id);
    if (CAN_Wrapper_Start() != HAL_OK)
    {
        Error_Handler();
    }
}

static void CAN_Slave_Process_Rx_Message(uint32_t cmd_id, uint8_t *data)
{
    switch (cmd_id){
        
        case ODRIVE_SET_AXIS_REQUESTED_STATE:
        {
            uint32_t state;
            memcpy(&state, data, sizeof(uint32_t));
            if (state == AXIS_STATE_CLOSED_LOOP_CONTROL)
            {
                // Domyślnie uruchomienie w trybie prędkości z zerową prędkością w rpm
                if (g_callbacks.start)
                {
                    g_callbacks.start();
                }
                if (g_callbacks.set_speed)
                {
                    g_callbacks.set_speed(0.0f);
                }
            }
            else if (state == AXIS_STATE_IDLE)
            {
                if (g_callbacks.stop)
                {
                    g_callbacks.stop();
                }
            }
            break;
        }
        case ODRIVE_SET_INPUT_VEL:
        {
            float vel_rpm;
            memcpy(&vel_rpm, data, sizeof(float));
            // Automatyczne przełączenie na tryb regulacji pedkości
            if (g_callbacks.set_speed)
            {
                // printf("CAN: Set speed to %.2f RPM\r\n", vel_rpm);
                g_callbacks.set_speed(vel_rpm);
            }
            break;
        }
        case ODRIVE_SET_INPUT_TORQUE:
        {
            float torque_A;
            memcpy(&torque_A, data, sizeof(float));
            // printf("CAN: Set torque to %.2f A\r\n", torque_A);
            // Automatyczne przełączenie na tryb regulacji momentu
            // Domyślnie torque w A - do ustalenia czy w Nm czy w A
            if (g_callbacks.set_torque)
            {
                g_callbacks.set_torque(torque_A);
            }
            break;
        }
        // case ODRIVE_GET_ENCODER_ESTIMATES:
        // {
        //     uint8_t tx_data[8];
        //     float pos_estimate = (float)MC_GetElAngledppMotor1();
        //     float vel_estimate = MC_GetAverageMecSpeedMotor1_F();
        //     memcpy(&tx_data[0], &pos_estimate, sizeof(float));
        //     memcpy(&tx_data[4], &vel_estimate, sizeof(float));
        //     uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_GET_ENCODER_ESTIMATES);
        //     send_can_frame(id, tx_data, 8);
        //     break;
        // }
        // case ODRIVE_GET_IQ:
        // {
        //     uint8_t tx_data[8];
        //     qd_t iqd_measured = MC_GetIqdMotor1();
        //     qd_t iqd_setpoint = MC_GetIqdrefMotor1();
        //     float iq_setpoint_val = (float)iqd_setpoint.q;
        //     float iq_measured_val = (float)iqd_measured.q;
        //     memcpy(&tx_data[0], &iq_setpoint_val, sizeof(float));
        //     memcpy(&tx_data[4], &iq_measured_val, sizeof(float));
        //     uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_GET_IQ);
        //     send_can_frame(id, tx_data, 8);
        //     break;
        // }
        // case ODRIVE_CLEAR_ERRORS:
        // {
        //     MC_AcknowledgeFaultMotor1();
        //     break;
        // }
        case ODRIVE_REBOOT:
        {
            if (g_callbacks.reboot)
            {
                g_callbacks.reboot();
            }
            break;
        }
        case ODRIVE_SET_HEARTBEAT_FREQ:
        {
            uint16_t new_heartbeat_freq_hz;
            uint16_t new_telemetry_freq_hz;

            memcpy(&new_heartbeat_freq_hz, &data[0], sizeof(uint16_t));
            memcpy(&new_telemetry_freq_hz, &data[2], sizeof(uint16_t));

            if (new_heartbeat_freq_hz > 0 && new_heartbeat_freq_hz <= ISR_FREQ_HZ)
            {
                g_heartbeat_period_ticks = PERIOD_TICKS(new_heartbeat_freq_hz);
            }
            else
            {
                g_heartbeat_period_ticks = 0;
            }

            if (new_telemetry_freq_hz > 0  && new_telemetry_freq_hz <= ISR_FREQ_HZ)
            {
                g_telemetry_period_ticks = PERIOD_TICKS(new_telemetry_freq_hz);
            }
            else
            {
                g_telemetry_period_ticks = 0;
            }

            break;
        }
        default:
            break;
    }
    g_last_cmd_id = cmd_id; // Zapisz ostatnią komendę do debugowania
    memcpy(g_last_cmd_payload, data, 8); // Zapisz ostatni payload do debugowania
    g_can_cmd_received_flag = true; // Ustaw flagę, że otrzymano komendę CAN
}

// Funkcja do printowania w while(1) ostatniej komendy CAN, przydatne do debugowania
void CAN_Slave_PrintLastCommand(void)
{
    switch (g_last_cmd_id)
    {
        case ODRIVE_SET_AXIS_REQUESTED_STATE:
        {
            uint32_t state;
            memcpy(&state, g_last_cmd_payload, sizeof(uint32_t));
            printf("CAN CMD: SET_AXIS_STATE | State: %lu\r\n", (unsigned long)state);
            break;
        }
        case ODRIVE_SET_INPUT_VEL:
        {
            float vel_rpm;
            memcpy(&vel_rpm, g_last_cmd_payload, sizeof(float));
            printf("CAN CMD: SET_INPUT_VEL | Vel: %.2f RPM\r\n", vel_rpm);
            break;
        }
        case ODRIVE_SET_INPUT_TORQUE:
        {
            float torque_A;
            memcpy(&torque_A, g_last_cmd_payload, sizeof(float));
            printf("CAN CMD: SET_INPUT_TORQUE | Torque: %.2f A\r\n", torque_A);
            break;
        }
        case ODRIVE_REBOOT:
        {
            printf("CAN CMD: ODRIVE_REBOOT\r\n");
            break;
        }
        case ODRIVE_SET_HEARTBEAT_FREQ:
        {
            uint16_t hb_freq, tel_freq;
            memcpy(&hb_freq, &g_last_cmd_payload[0], sizeof(uint16_t));
            memcpy(&tel_freq, &g_last_cmd_payload[2], sizeof(uint16_t));
            printf("CAN CMD: SET_HEARTBEAT_FREQ | Heartbeat: %u Hz, Telemetry: %u Hz\r\n", hb_freq, tel_freq);
            break;
        }
        default:
        {
            if (g_last_cmd_id != (uint32_t)-1) {
                printf("CAN CMD: Unknown (0x%02X)\r\n", (unsigned int)g_last_cmd_id);
            }
            break;
        }
    }
}

void CAN_Slave_Heartbeat(void)
{
    uint8_t tx_data[8];

    // uint16_t axis_error = MC_GetCurrentFaultsMotor1();       // faults that are currently active.
    // uint16_t occurred_faults = MC_GetOccurredFaultsMotor1(); // faults that have occurred on Motor 1 since its state machine moved to the #FAULT_NOW state.
    // uint16_t current_state = MC_GetSTMStateMotor1();
    // uint16_t axis_state = (current_state == RUN) ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;
    // uint16_t motor_power_int = (uint16_t)MC_GetAveragePowerMotor1_F();

    uint16_t axis_error = 0;
    uint16_t axis_state = 0;

    if (g_callbacks.get_heartbeat)
    {
        g_callbacks.get_heartbeat(&axis_state, &axis_error);
    }

    memcpy(&tx_data[0], &axis_state, sizeof(uint16_t));
    memcpy(&tx_data[2], &axis_error, sizeof(uint16_t));
    // memcpy(&tx_data[4], &occurred_faults, sizeof(uint16_t));
    // memcpy(&tx_data[6], &motor_power_int, sizeof(uint16_t));

    uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_HEARTBEAT_MESSAGE);
    send_can_frame(id, tx_data, 8);
}

void CAN_Slave_Telemetry(void)
{
    uint8_t tx_data[8];

    float current_angle_rev = 0.0f;
    float current_velocity_rpm = 0.0f;

    if (g_callbacks.get_telemetry)
    {
        g_callbacks.get_telemetry(&current_angle_rev, &current_velocity_rpm);
    }

    memcpy(&tx_data[0], &current_angle_rev, sizeof(float));
    memcpy(&tx_data[4], &current_velocity_rpm, sizeof(float));

    uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_GET_ENCODER_ESTIMATES);
    send_can_frame(id, tx_data, 8);
}

#endif // DEVICE_IS_SLAVE