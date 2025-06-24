#ifndef UWB_CORE_H
#define UWB_CORE_H

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
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


wait_ntf_result_t  wait_for_notification_or_skip_if_ready(uint32_t timeout_ms);

void build_start_radar_command(uint8_t *buffer_out, size_t *len_out);

void build_cmd_set_bitfield(uint8_t *buffer_out, size_t *len_out, uint32_t address, uint8_t offset, uint8_t width, uint32_t value);

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