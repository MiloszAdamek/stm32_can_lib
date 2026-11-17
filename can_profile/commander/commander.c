#if defined(DEVICE_IS_MASTER)

#include "commander.h"
#include "slave_driver.h" // Potrzebny do wysyłania komend i nowej funkcji
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

static UART_HandleTypeDef* g_huart; // Przechowuje wskaźnik do używanego UART

#define UART_RX_BUFFER_SIZE 64
static uint8_t g_uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t g_uart_rx_char;
static volatile uint8_t g_uart_rx_index = 0;
static volatile bool g_new_command_flag = false;

static void process_command(char* cmd);

// --- Implementacja publicznego API ---

void Commander_Init(void* huart_void) {
    if (huart_void == NULL) {
        Error_Handler();
    }
    g_huart = (UART_HandleTypeDef*)huart_void;

    HAL_UART_Receive_IT(g_huart, &g_uart_rx_char, 1);

    printf("\r\n--- CAN Motor Commander ---\r\n");
    printf("Slave IDs: 1-FLYWHEEL, 2-LEFT, 3-RIGHT, 4-DRIVE\r\n");
    printf("Commands format: <cmd> <id> [args...]\r\n");
    printf(" X <id>                     (Start Motor / Closed Loop)\r\n");
    printf(" B <id>                     (Stop Motor / Idle)\r\n");
    printf(" S <id> <state>             (Request state, 1-STOP or 8-START)\r\n");
    printf(" V <id> <velocity>          (RPM)\r\n");
    printf(" T <id> <torque>            (Amps)\r\n");
    printf(" C <id>                     (Clear Errors)\r\n");
    printf(" R <id>                     (Reboot Slave)\r\n");
    printf(" A <id>                     (Ack Faults - alias for Clear Errors)\r\n");
    printf(" F <id> <hb_hz> <tel_hz>    (Set Heartbeat and Telemetry frequency)\r\n");
    printf(" L <id> <amps>              (Set Current Limit)\r\n");
    printf(" W <id> <timeout_ms>        (Set Watchdog Timeout in ms, 0 to disable)\r\n");
    printf("> ");
}

void Commander_Process(void) {
    if (g_new_command_flag) {
        g_new_command_flag = false;

        g_uart_rx_buffer[g_uart_rx_index] = '\0';
        process_command((char*)g_uart_rx_buffer);

        g_uart_rx_index = 0;
        memset(g_uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);

        printf("> ");
    }
}

// --- Funkcje prywatne (static) ---
static void process_command(char* cmd) {
    char* token = strtok(cmd, " ");
    if (token == NULL) {
        return; // Pusta komenda
    }
    char command_char = token[0];

    // Pobierz drugi token - ID silnika
    char* id_str = strtok(NULL, " ");
    if (id_str == NULL) {
        // Wyjątek dla komend, które mogłyby nie wymagać ID w przyszłości
        if (strchr("xbsvtcrafXBSVTCRAF", command_char)) {
             printf("Error: Missing slave ID for command '%c'\r\n", command_char);
        } else {
             printf("Error: Unknown command or missing arguments\r\n");
        }
        return;
    }

    uint8_t slave_id = atoi(id_str);
    SlaveRole_t slave_role = GetSlaveRoleFromId(slave_id);

    if (slave_role == NUM_SLAVES) {
        printf("Error: Invalid slave ID: %d\r\n", slave_id);
        return;
    }

    // Pobierz resztę argumentów
    char* arg1_str = strtok(NULL, " ");
    char* arg2_str = strtok(NULL, " ");


    switch(command_char) {
        case 'S':
        case 's': {
            if (arg1_str == NULL) {
                printf("Error: Missing state argument. Usage: S <id> <state>\r\n");
                break;
            }
            int32_t state = atoi(arg1_str);
            printf("CMD: Set State for ID %d -> %ld\r\n", slave_id, state);
            SlaveDriver_SetState(slave_role, (CAN_StateRequest_t)state);
            break;
        }
        case 'V':
        case 'v': {
            if (arg1_str == NULL) {
                printf("Error: Missing velocity argument. Usage: V <id> <velocity>\r\n");
                break;
            }
            float velocity = atof(arg1_str);
            printf("CMD: Set Velocity for ID %d -> %.2f RPM\r\n", slave_id, velocity);
            SlaveDriver_SetVelocity(slave_role, velocity);
            break;
        }
        case 'T':
        case 't': {
            if (arg1_str == NULL) {
                printf("Error: Missing torque argument. Usage: T <id> <torque>\r\n");
                break;
            }
            float torque = atof(arg1_str);
            printf("CMD: Set Torque for ID %d -> %.2f A\r\n", slave_id, torque);
            SlaveDriver_SetTorque(slave_role, torque);
            break;
        }
        case 'C':
        case 'c':
        case 'A': // Traktujemy 'A' jako alias do 'C'
        case 'a': {
            printf("CMD: Clear Errors for ID %d\r\n", slave_id);
            SlaveDriver_ClearErrors(slave_role);
            break;
        }
        case 'R':
        case 'r': {
            printf("CMD: Reboot Slave ID %d\r\n", slave_id);
            SlaveDriver_Reboot(slave_role);
            break;
        }
        case 'X': // Start Motor
        case 'x': {
            printf("CMD: Set Axis State for ID %d -> CLOSED_LOOP\r\n", slave_id);
            SlaveDriver_SetState(slave_role, CAN_REQUEST_START);
            break;
        }
        case 'B': // Stop Motor
        case 'b': {
            printf("CMD: Set Axis State for ID %d -> IDLE\r\n", slave_id);
            SlaveDriver_SetState(slave_role, CAN_REQUEST_STOP);
            break;
        }
        case 'F':
        case 'f': {
           if (arg1_str == NULL || arg2_str == NULL) {
               printf("Error: Command 'F' requires two frequency arguments. Usage: F <id> <hb_hz> <tel_hz>\r\n");
               break;
           }
           uint16_t heartbeat_freq = (uint16_t)atoi(arg1_str);
           uint16_t telemetry_freq = (uint16_t)atoi(arg2_str);

           printf("CMD: Set Frequencies for ID %d -> Heartbeat: %u Hz, Telemetry: %u Hz\r\n",
                  slave_id, heartbeat_freq, telemetry_freq);

           SlaveDriver_SetHeartbeatFreq(slave_role, heartbeat_freq, telemetry_freq);
           break;
        }
        case 'L':
        case 'l': {
            if (arg1_str == NULL) {
                printf("Error: Missing current limit. Usage: L <id> <amps>\r\n");
                break;
            }
            float current_limit_A = atof(arg1_str);
            printf("CMD: Set Current Limit for ID %d -> %.2f A\r\n", slave_id, current_limit_A);
            SlaveDriver_SetCurrentLimit(slave_role, current_limit_A);
            break;
        }
        case 'W':
        case 'w': {
            if (arg1_str == NULL) {
                printf("Error: Missing watchdog timeout argument. Usage: W <id> <timeout_ms>\r\n");
                break;
            }
            uint16_t timeout_ms = (uint16_t)atoi(arg1_str);
            printf("CMD: Set Watchdog Timeout for ID %d -> %u ms\r\n", slave_id, timeout_ms);
            SlaveDriver_SetWatchdogTimeout(slave_role, timeout_ms);
            break;
        }
        default:
            printf("Error: Unknown command '%c'\r\n", command_char);
            break;
    }
}

// Ta funkcja pozostaje bez zmian
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == g_huart->Instance) {
        if (g_uart_rx_char == '\r' || g_uart_rx_char == '\n') {
            printf("\r\n");
            if (g_uart_rx_index > 0) {
                g_new_command_flag = true;
            } else {
                printf("> ");
            }
        }
        else if (g_uart_rx_char == '\b' || g_uart_rx_char == 127) {
            if (g_uart_rx_index > 0) {
                g_uart_rx_index--;
                printf("\b \b");
            }
        }
        else if (g_uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
            g_uart_rx_buffer[g_uart_rx_index++] = g_uart_rx_char;
            printf("%c", g_uart_rx_char);
        }

        // Ponownie włącz nasłuchiwanie
        HAL_UART_Receive_IT(g_huart, &g_uart_rx_char, 1);
    }
}

#endif
