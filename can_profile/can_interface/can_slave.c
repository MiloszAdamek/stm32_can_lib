// can_slave.c

#if defined(DEVICE_IS_SLAVE)

// --- Implementacja API dla trybu SLAVE ---

#include "can_interface.h"
#include "can_wrapper.h"
#include <string.h>
#include <stdio.h>
#include "math.h"

#include "motor_control.h"

// #include "mc_api.h"
// #include "mc_interface.h"
// #include "mc_configuration_registers.h"

static uint8_t g_my_axis_id = 0;
static void CAN_Slave_Process_Rx_Message(uint32_t cmd_id, uint8_t *data);
extern volatile uint16_t g_heartbeat_period_ms;
extern volatile uint16_t g_telemetry_period_ms;

/**
 * @brief Centralny callback dla tego modułu, przekazywany do wrappera.
 *        Rozdziela odebrane wiadomości do logiki Mastera lub Slave'a.
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

void CAN_Slave_Init(void *hcan_void, uint8_t my_axis_id)
{
    if (hcan_void == NULL)
    {
        Error_Handler();
    }

    CAN_Wrapper_Init(hcan_void);

    g_my_axis_id = my_axis_id & 0x3F;

    CAN_Wrapper_RegisterRxCallback(odrive_rx_callback);
    CAN_Wrapper_ConfigFilter_ODrive(g_my_axis_id);
    if (CAN_Wrapper_Start() != HAL_OK)
    {
        Error_Handler();
    }
}

static void CAN_Slave_Process_Rx_Message(uint32_t cmd_id, uint8_t *data)
{
    switch (cmd_id)
    {

    case ODRIVE_SET_AXIS_REQUESTED_STATE:
    {
        uint32_t state;
        memcpy(&state, data, sizeof(uint32_t));
        if (state == AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            // Domyślnie uruchomienie w trybie prędkości z zerową prędkością w rpm
            MotorControl_Start();
            MotorControl_SetSpeed(0.0f);
        }
        else if (state == AXIS_STATE_IDLE)
        {
            MotorControl_Stop();
        }
        break;
    }
    case ODRIVE_SET_INPUT_VEL:
    {
        float vel_rpm;
        memcpy(&vel_rpm, data, sizeof(float));
        // Automatyczne przełączenie na tryb regulacji pedkości
        MotorControl_SetSpeed(vel_rpm);
        break;
    }
    case ODRIVE_SET_INPUT_TORQUE:
    {
        float torque_A;
        memcpy(&torque_A, data, sizeof(float));
        // Automatyczne przełączenie na tryb regulacji momentu
        // Domyślnie torque w A - do ustalenia czy w Nm czy w A
        MotorControl_SetTorque_Iq(torque_A);
        break;
    }
    case ODRIVE_GET_ENCODER_ESTIMATES:
    {
        uint8_t tx_data[8];
        float pos_estimate = (float)MC_GetElAngledppMotor1();
        float vel_estimate = MC_GetAverageMecSpeedMotor1_F();
        memcpy(&tx_data[0], &pos_estimate, sizeof(float));
        memcpy(&tx_data[4], &vel_estimate, sizeof(float));
        uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_GET_ENCODER_ESTIMATES);
        send_can_frame(id, tx_data, 8);
        break;
    }
    case ODRIVE_GET_IQ:
    {
        uint8_t tx_data[8];
        qd_t iqd_measured = MC_GetIqdMotor1();
        qd_t iqd_setpoint = MC_GetIqdrefMotor1();
        float iq_setpoint_val = (float)iqd_setpoint.q;
        float iq_measured_val = (float)iqd_measured.q;
        memcpy(&tx_data[0], &iq_setpoint_val, sizeof(float));
        memcpy(&tx_data[4], &iq_measured_val, sizeof(float));
        uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_GET_IQ);
        send_can_frame(id, tx_data, 8);
        break;
    }
    case ODRIVE_CLEAR_ERRORS:
    {
        MC_AcknowledgeFaultMotor1();
        break;
    }
    case ODRIVE_REBOOT:
    {
        HAL_NVIC_SystemReset();
        break;
    }
    case ODRIVE_SET_HEARTBEAT_FREQ:
    {
        uint16_t new_heartbeat_freq_hz;
        uint16_t new_telemetry_freq_hz;

        memcpy(&new_heartbeat_freq_hz, &data[0], sizeof(uint16_t));
        memcpy(&new_telemetry_freq_hz, &data[2], sizeof(uint16_t));

        if (new_heartbeat_freq_hz > 0)
        {
            g_heartbeat_period_ms = 1000 / new_heartbeat_freq_hz;
        }
        else
        {
            g_heartbeat_period_ms = 0xFFFF; // Ustaw bardzo duży okres (praktycznie wyłącz)
        }
        if (new_telemetry_freq_hz > 0)
        {
            g_telemetry_period_ms = 1000 / new_telemetry_freq_hz;
        }
        else
        {
            g_telemetry_period_ms = 0xFFFF;
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief Current position in Revolution (0.0 - 1.0)
 */
static float GetEncoderPositionRev(void)
{

    float pole_pairs_float = (float)MotorConfig_reg[M1]->polePairs;

    // Odczytaj kąt elektryczny
    int16_t el_angle_dpp = MC_GetElAngledppMotor1();

    // Konwertuj na obroty elektryczne
    float electric_revolutions = (float)el_angle_dpp / 65536.0f;

    // Konwertuj na obroty mechaniczne
    float mechanical_revolutions = electric_revolutions / pole_pairs_float;

    // Zakres 0.0 - 1.0
    float wrapped_revolutions = fmodf(mechanical_revolutions, 1.0f);
    if (wrapped_revolutions < 0.0f)
    {
        wrapped_revolutions += 1.0f;
    }

    return wrapped_revolutions;
}

/**
 * @brief Current velocity in Revolution Per Second
 */
static float GetEncoderVelocityRPS(void)
{
    float velocity_rpm = MC_GetAverageMecSpeedMotor1_F();
    return velocity_rpm / 60.0f;
}

/**
 * @brief Current velocity in Revolution Per Minute
 */
static float GetEncoderVelocityRPM(void)
{
    return MC_GetAverageMecSpeedMotor1_F();
}

void CAN_Slave_Heartbeat(void)
{
    uint8_t tx_data[8];

    uint16_t axis_error = MC_GetCurrentFaultsMotor1();       // faults that are currently active.
    uint16_t occurred_faults = MC_GetOccurredFaultsMotor1(); // faults that have occurred on Motor 1 since its state machine moved to the #FAULT_NOW state.
    uint16_t current_state = MC_GetSTMStateMotor1();
    uint16_t axis_state = (current_state == RUN) ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;

    uint16_t motor_power_int = (uint16_t)MC_GetAveragePowerMotor1_F();

    memcpy(&tx_data[0], &axis_state, sizeof(uint16_t));
    memcpy(&tx_data[2], &axis_error, sizeof(uint16_t));
    memcpy(&tx_data[4], &occurred_faults, sizeof(uint16_t));
    memcpy(&tx_data[6], &motor_power_int, sizeof(uint16_t));

    uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_HEARTBEAT_MESSAGE);
    send_can_frame(id, tx_data, 8);
}

void CAN_Slave_Telemetry(void)
{
    uint8_t tx_data[8];

    float current_angle_rev = GetEncoderPositionRev();
    float current_velocity = GetEncoderVelocityRPM();

    memcpy(&tx_data[0], &current_angle_rev, sizeof(float));
    memcpy(&tx_data[4], &current_velocity, sizeof(float));

    uint32_t id = ODRIVE_MAKE_CAN_ID(g_my_axis_id, ODRIVE_GET_ENCODER_ESTIMATES);
    send_can_frame(id, tx_data, 8);
}

#endif // DEVICE_IS_SLAVE