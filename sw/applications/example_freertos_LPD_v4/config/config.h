#ifndef UWB_CORE_H
#define UWB_CORE_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "gpio.h"
#include "uwb_commands.h"

extern bool success;

#define RDY_TIMEOUT_MS 1000 // 1 segundo máximo
#define INT_TIMEOUT_MS 1000 // 1 segundo máximo


bool validate_crc(const uint8_t *buffer, size_t length);

uint16_t crc16_xmodem(const uint8_t *data, size_t length);
void reverse_bytes(uint8_t *data, size_t length);

typedef enum {
    WAIT_NTF_NONE = 0,
    WAIT_NTF_PENDING,  // INT bajó
    WAIT_NTF_RECEIVED, // ya leído dentro de la función
} wait_ntf_result_t;

typedef struct {
    uint8_t mt;           // Message Type
    uint8_t gid;          // Group ID
    uint8_t oid;          // Opcode ID
    uint8_t pbf;          // Packet Boundary Flag
    uint8_t pldext;     // Payload Extension
    uint8_t rfu;        // Reserved for Future Use
    uint16_t payload_len; // Longitud del payload
} UCIPacketHeader;


typedef enum {
    UCI_PACKET_OK,           // Paquete completo (PBF=0)
    UCI_PACKET_FRAGMENT,     // Es fragmento, seguir leyendo
    UCI_PACKET_INVALID       // Error de clasificación o cabecera
} UCIPacketStatus;


// Construye comando CMD_RESET_DEVICE (sin payload, solo cabecera + CRC)
// void build_reset_device_command(uint8_t *buffer_out, size_t *len_out);

void build_start_radar_command(uint8_t *buffer_out, size_t *len_out);

//void build_cmd_get_fw_version(uint8_t *cmd_get_fw_version, size_t *len_out);

void hard_reset();
// //================================================================
// // CMD_SET_BITFIELD. Desactivar watchdog (payload: address, offset, width, value)
// // Construye comando CMD_SET_BITFIELD (payload: address, offset, width, value)
void build_cmd_set_bitfield(uint8_t *buffer_out, size_t *len_out, uint32_t address, uint8_t offset, uint8_t width, uint32_t value);
// // CMD_GENERIC_BASEBAND_CONFIG. Cualquier configuración baseband (RX, TX, CIR, etc)
// void build_cmd_generic_baseband_config();
// // CMD_RADAR_APPLICATION_CONFIG. Configuración de modo radar (calibración, streaming...)
// void build_cmd_radar_application_config();
// // CMD_START_RADAR. Comenzar ejecución del radar
// void build_cmd_start_radar();
// // CMD_GET_BASEBAND_RESULTS. Obtener resultados de ganancia, calibración o datos CIR
// void build_cmd_get_baseband_results();

// CMD_RX_RADIO_CONFIG. Alternativa opcional con configuración en binario RX
//void build_cmd_rx_radio_config();
// CMD_TX_RADIO_CONFIG. Alternativa opcional con configuración en binario RX
//void build_cmd_tx_radio_config();

// ===========================================================================
// Empaquetado general UCI + CRC16
// Construir el paquete binario UCI listo para enviar por SPI/UART
void build_uci_packet();

//================================================================
// Enviar y recibir comando
bool send_uci_cmd_get_rsp(uint8_t *cmd_buffer, size_t cmd_len);
bool transmit_uci_command(uint8_t *cmd_buffer, size_t len);
bool receive_uci_message(uint8_t *recv_buffer, size_t max_len, size_t *recv_len);
bool wait_for_gpio_low(gpio_pin_number_t gpio, uint32_t timeout_ms);
//void parse_uwb_response(const uint8_t *rx_buffer, size_t length);
void gpio_monitor_task(void* arg);

UCIPacketStatus analyze_uci_header(const uint8_t *rx_buffer, size_t len, UCIPacketHeader *out);
void handle_ntf_core_generic(const uint8_t *payload, size_t payload_len);
void handle_resp_status(const uint8_t *payload, size_t payload_len);
void interpret_ntf_radar_result(const uint8_t *payload, size_t payload_len); 
void print_resp_status(uint8_t code);
void print_gpio_states();
#endif // UWB_CORE_H