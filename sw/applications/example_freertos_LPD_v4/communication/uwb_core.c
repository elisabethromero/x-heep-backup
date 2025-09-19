#include "uwb_core.h"
#include "uwb_commands.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "gpio.h"
#include "string.h"
#include "circular_buffer.h"
#include "task_control.h"
#include "uwb_core.h"
#include "uwb_api.h"
#include "spi_interface.h"
#include "gpio_interface.h"
#include "Config.h"
#include "sync.h"

// ================================================================
// ================================================================
// ================================================================
// ================================================================

void hard_reset(){

    if(interrupt_processing_enabled) {
        printf("\n[HARD RESET] Disabling interrupt processing...\n");
    }

    printf("\n[HARD RESET] Triggering UWB reset...\n");
    vTaskDelay(pdMS_TO_TICKS(10));
    // gpio_write(GPIO_RST_IO, 1);
    printf("\033[0;31m[INTERRUPCIÓN] RST_IO BAJA\033[0m\n");
    gpio_write(GPIO_RST_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    printf("\033[0;31m[INTERRUPCIÓN] RST_IO SUBE\033[0m\n");
    gpio_write(GPIO_RST_IO, 1);

    if (!interrupt_processing_enabled) {
        printf("\n[HARD RESET] Interrupt processing is disabled. Enabling it now...\n");
        interrupt_processing_enabled = true;
    }
}

// FUNCIÓN PARA ENVIAR Y RECIBIR UN COMANDO UCI
bool send_uci_cmd_get_rsp(uint8_t *cmd_buffer, size_t cmd_len) {
   
    // printf("\t[UCI] Enviando comando UCI...\n");
   
    //print_gpio_states(); 
 
    // =============================================================
    // BAJADA DE CS Y ESPERA DE RDY
    // =============================================================
    printf("\n\t[WAIT RDY] Bajando CS y esperando a que RDY baje...\n");
   
    // Bajar CS para iniciar la transmisión 
    //printf("\033[0;31m[INTERRUPCIÓN] GPIO_CS BAJA\033[0m\n");
    gpio_write(GPIO_CS, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    bool level;
    gpio_read(GPIO_RDY_IO, &level);
    printf("\tNivel del gpio RDY: %d\n", level);
    //print_gpio_states();
    if (!wait_for_gpio_low(GPIO_RDY_IO, RDY_TIMEOUT_MS)) return false;
    gpio_read(GPIO_RDY_IO, &level);
    printf("\tNivel del gpio RDY: %d\n", level);
    //print_gpio_states();

    // ==============================================================
    // ENVÍO DEL COMANDO UCI
    // ==============================================================
    if (!transmit_uci_command(cmd_buffer, cmd_len)) return false;

    // ==============================================================
    // ESPERA RESPUESTA DEL UWB (BAJADA DE GPIO INT)
    // ==============================================================
    // printf("\tNivel del gpio INT: %d\n", gpio_get_level(GPIO_INT_IO));
    // if( ! wait_for_gpio_low(GPIO_INT_IO, RDY_TIMEOUT_MS) ) return false;
    // printf("\tNivel del gpio INT: %d\n", gpio_get_level(GPIO_INT_IO));


    // size_t len = 0;
    // while (gpio_get_level(GPIO_INT_IO) == 0) {
    //     // ==============================================================
    //     // RECIBIR RESPUESTA DEL UWB
    //     // ==============================================================
    //     if (!receive_uci_message(recv_buffer, sizeof(recv_buffer), &len)) return false;
    //     // ==============================================================
    //     // PROCESAR RESPUESTA DEL UWB
    //     // ==============================================================
    //     parse_uwb_response(recv_buffer, sizeof(recv_buffer));  
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
    
    return true;
}

void gpio_monitor_task(void* arg) {
    uint32_t io_num;
    uint8_t recv_buffer[403] = {0};  // Buffer para recibir la respuesta del UWB
    bool level;
    while(true) {

        if ((xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))){ // && (acquisition_active || conf_uwb_active)){
            if (interrupt_processing_enabled){
                //printf("ACQUISITION ACTIVE: %d, CONF UWB ACTIVE: %d\n", acquisition_active, conf_uwb_active);
                gpio_read(io_num, &level);
                
                const char* pin_name = (io_num == GPIO_CS) ? "CS" :
                                    (io_num == GPIO_RDY_IO) ? "RDY" : 
                                    (io_num == GPIO_RST_IO) ? "RST" :
                                    (io_num == GPIO_INT_IO) ? "INT" : "UNKNOWN";

                //printf("\033[0;31m[INTERRUPCIÓN] GPIO_%s (%ld) %s\033[0m\n", pin_name, io_num,
                //       level == 0 ? "BAJA" : "SUBE");

                // Si es el GPIO_INT_IO, se llama a uwb_receive_message
                if ((io_num == GPIO_INT_IO) && (interrupt_processing_enabled)){//) && (acquisition_active || conf_uwb_active)){
                    size_t len = 0;
                    printf("Interrupción PIN INT\n");
                    gpio_read(GPIO_INT_IO, &level);
                    while ((level == 0) && (interrupt_processing_enabled)){// && (acquisition_active || conf_uwb_active)) {
                        // ==============================================================
                        // RECIBIR RESPUESTA DEL UWB
                        printf("\nRecibiendo respuesta del UWB desde ISR...");
                        // ==============================================================
                        gpio_read(GPIO_INT_IO, &level);
                        gpio_write(GPIO_PRUEBA, 1);
                        if (!receive_uci_message(recv_buffer,sizeof(recv_buffer), &len)) {
                            gpio_write(GPIO_CS, true);  // Asegurar que se libera CS
                            break;
                        } 
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                // print_gpio_states();
                } else if (io_num == GPIO_RDY_IO){
                    printf("Interrupción PIN RDY\n");
                }
            }
        }
    }
}

void print_gpio_states(){
    bool level;
    printf("\033[0;32m\t\tChecando estado de los pines GPIO...\033[0m\n");
    gpio_read(GPIO_INT_IO, &level);
    printf("\033[0;32m\t\tNivel del gpio INT: %d\033[0m\n", level);
    gpio_read(GPIO_RDY_IO, &level);
    printf("\033[0;32m\t\tNivel del gpio RDY: %d\033[0m\n", level);
    gpio_read(GPIO_CS, &level);
    printf("\033[0;32m\t\tNivel del gpio CS: %d\033[0m\n", level);
    gpio_read(GPIO_RST_IO, &level);
    printf("\033[0;32m\t\tNivel del gpio RST: %d\033[0m\n", level);
}

bool wait_for_gpio_low(gpio_pin_number_t gpio, uint32_t timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    bool level;
    gpio_read(gpio, &level);
    if (level == 1) {
        printf("\n\t[WAIT %s] Esperando a que %s para recibir mensaje...\n", gpio == GPIO_INT_IO ? "INT" : "RDY", gpio == GPIO_INT_IO ? "INT" : "RDY");
    }
    
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        gpio_read(gpio, &level);
        if (level == 0) {
            printf("\t[WAIT] %s_GPIO está en LOW.\n", gpio == GPIO_INT_IO ? "INT" : "RDY");
            return true;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    //print_gpio_states();
    printf("\tTimeout esperando %s, no se recibió mensaje.\n", gpio == GPIO_INT_IO ? "INT" : "RDY");

    return false;
}

// ====================================================================
// Función para transmitir un comando UCI al UWB
// ====================================================================
bool transmit_uci_command(uint8_t *cmd_buffer, size_t len) {
    printf("\n\t[SEND] Enviando comando UCI...\n");
     
    if ((spi_transfer(cmd_buffer, NULL, len, SPI_DIR_TX_ONLY) != SPI_CODE_OK))// Inicializa la transferencia SPI
    {
        printf("Error en transferencia SPI.\n");
        gpio_write(GPIO_CS, true);
        success = false;
        return false;
    } else {
        // Subir CS para terminar la transmisión
        vTaskDelay(5 / portTICK_PERIOD_MS);
        gpio_write(GPIO_CS, true);
        printf("\tCommand sent successfully.\n");
        print_gpio_states();
    }
    return true;
}

// ====================================================================
// Función para recibir un mensaje UCI desde el UWB
// ====================================================================
bool receive_uci_message(uint8_t *recv_buffer, size_t max_len, size_t *recv_len){

    printf("\n\t[RECV] Recibiendo respuesta de UWB...\n");
    
    // Bajar CS para iniciar la transmisión
    gpio_write(GPIO_CS, false);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // =============================================================

    // Leer cabecera de 4 bytes
    // printf("\tLeyendo cabecera y payload length...\n");
    uint8_t header[4] = {0x00, 0x00, 0x00, 0x00};
    
    uint8_t tx_empty[4] = {0x00, 0x00, 0x00, 0x00};
    spi_codes_e code_test;
    code_test = spi_transfer(tx_empty, header, 4, SPI_DIR_RX_ONLY);
    /*if (code_test != SPI_CODE_OK) {
        gpio_write(GPIO_CS, true);
        printf("Error leyendo cabecera SPI Code= 0x%02X\n", code_test);
        return false;
    }*/



    //print_gpio_states();
    // Imprimir la cabecera recibida
    printf("\tCabecera recibida: ");
    for (size_t i = 0; i < 4; i++) {
        printf("%02X", header[i]);
    }
    printf("\n");

    // Analizar cabecera
    UCIPacketHeader hdr;
    UCIPacketStatus status = analyze_uci_header(header, 4, &hdr);
    if (status == UCI_PACKET_INVALID) {
        printf("\tCabecera inválida. Cancelando recepción.\n");
        gpio_write(GPIO_CS, 1);
        return false;
    } 
    
    // Copiar cabecera al buffer final
    memcpy(recv_buffer, header, 4);
       
    // =============================================================

    // Calcular longitud total del paquete
    //printf("\tPayload length: %u bytes\n",  hdr.payload_len);
    size_t total_len = 4 + hdr.payload_len + 2; // 4 bytes de cabecera + payload + 2 bytes de CRC
    //printf("\tTamaño total del paquete: %zu bytes\n", total_len);

    if (spi_transfer(NULL, recv_buffer + 4, hdr.payload_len + 2, SPI_DIR_RX_ONLY) != SPI_CODE_OK) {
        gpio_write(GPIO_CS, true);
        return false;
    }

    // // Imprimir el payload y CRC recibidos
    // printf("\tPayload y CRC recibidos: ");
    // for (size_t i = 0; i < hdr.payload_len + 2; i++) {
    //     printf("\033[0;31m%02X \033[0m", recv_buffer[4 + i]);
    // }
    // printf("\n");

       
    // Extraer el payload
    uint8_t *payload = recv_buffer + 4;
    size_t payload_len = hdr.payload_len;
    
    // Procesar el payload según el tipo de mensaje
    // Si es una notificación de tipo 0x03 (NTF_CORE_GENERIC)
    if (hdr.mt == 0x03 && hdr.gid == 0x00 && hdr.oid == 0x07 && hdr.payload_len == 1) {
        handle_ntf_core_generic(payload, payload_len);
    } 
    // Si es un comando de tipo 0x02 (CMD_GENERIC_BASEBAND_CONFIG)
    else if (hdr.mt == 0x02 && hdr.gid == 0x0A) {
        handle_resp_status(payload, payload_len);
    }  

    // // Leer respuesta del UWB
    // if ((spi_transfer(NULL, recv_buffer, len, SPI_DIR_RX_ONLY) != SPI_FLAG_OK))// Inicializa la transferencia SPI
    // {
    //     printf("Error en transferencia SPI.\n");
    //     return false;
    // } else {
    //     printf("\tRespuesta recibida correctamente.\n");
    // }
        
    //MENSAJE IMPORTANTE:
    printf("\tMENSAJE RECIBIDO: ");
    for (int i = 0; i < total_len; i++) {
        printf("%02X ", recv_buffer[i]);
    }
    printf("\n");
    

    if(!validate_crc(recv_buffer, total_len)) {
        return false;
    }

    if((hdr.mt == 0x03 && hdr.gid == 0x0A && hdr.oid == 0x31) && (acquisition_active)) {
        interpret_ntf_radar_result(recv_buffer, total_len);
    }

    // Subir CS para finalizar la recepción
    //printf("\033[0;31m[INTERRUPCIÓN] GPIO_CS SUBE\033[0m\n");
    gpio_write(GPIO_CS, true);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Actualizar el tamaño del buffer de respuesta
    *recv_len = total_len;

    return true;
}

// ====================================================================
// Función para invertir los bytes de un buffer
// ====================================================================
void reverse_bytes(uint8_t *data, size_t length) {
    if (!data || length < 2) return;

    for (size_t i = 0; i < length / 2; i++) {
        uint8_t tmp = data[i];
        data[i] = data[length - 1 - i];
        data[length - 1 - i] = tmp;
    }
}

// =====================================================================
// CRC16 XMODEM
// =====================================================================
bool validate_crc(const uint8_t *buffer, size_t length) {
    if (length < 2) {
        printf("Error: El buffer es demasiado corto para contener un CRC.\n");
        return false;
    }

    // CRC16 (little-endian)
    //uint16_t crc_received = ((uint16_t)rx_buffer[total_len - 2] << 8) | rx_buffer[total_len - 1];
    // Extraer el CRC recibido
    uint16_t crc_received = (buffer[length - 2] << 8) | buffer[length - 1];

    // Calcular el CRC sobre el resto del buffer
    uint16_t crc_calculated = crc16_xmodem(buffer, length - 2);
    // Poner CRC en little-endian
    // uint8_t crc_lsb = crc_calculated & 0xFF;
    // uint8_t crc_msb = (crc_calculated >> 8) & 0xFF;

    reverse_bytes((uint8_t *)&crc_calculated, sizeof(crc_calculated));

    // Comparar los CRCs
    if (crc_received == crc_calculated) {
        printf("\tCRC válido: %04X\n", crc_received);
        return true;
    } else {
        printf("\t[ERROR] CRC inválido. Esperado: %04X, Recibido: %04X\n", crc_calculated, crc_received);
        return false;
    }
}



// =====================================================================
// Análisis de la cabecera UCI
// =====================================================================
UCIPacketHeader *out;

UCIPacketStatus analyze_uci_header(const uint8_t *rx_buffer, size_t len, UCIPacketHeader *out){

    // HEADER_BYTE1  
    out->mt   = (rx_buffer[0] >> 5) & 0x07; // Bits 7-5 del primer byte codifican el tipo de mensaje (MT)
    out->pbf  = (rx_buffer[0] >> 4) & 0x01; // Bit 4
    out->gid  = rx_buffer[0] & 0x0F;        // Bits 3-0 del primer byte codifican el GID

    // HEADER_BYTE2
    out->pldext = (rx_buffer[1] >> 7) & 0x01; // Bit 7
    out->rfu    = (rx_buffer[1] >> 6) & 0x01; // Bit 6
    out-> oid = rx_buffer[1]  & 0x3F; // Bits 5-0

    // PAYLOAD LENGTH (big-endian)
    out->payload_len = ((uint16_t)rx_buffer[2] << 8) | rx_buffer[3];
    
    if(acquisition_active || conf_uwb_active || interrupt_processing_enabled){ 
        printf("\t>> MT=0x%02X, GID=0x%02X, OID=0x%02X\n",out->mt, out->gid, out->oid);
        printf("\tTamaño total del paquete: %zu bytes\n", out->payload_len + 4 + 2);
        printf("\tHeader: MT=0x%02X, GID=0x%02X, OID=0x%02X, PBF=%u, PayloadLen=%u, Pldext=%u, RFU=%u\n",
            out->mt, out->gid, out->oid, out->pbf, out->payload_len, out->pldext, out->rfu);

        // Clasificación por tipo de mensajes
        switch (out->mt) {
            case 0x01: // CMD (no deberías recibir esto como respuesta)
                printf("\tReceived command (CMD). Likely an interpretation error.\n");
                return UCI_PACKET_INVALID;

            case 0x02: // RSP
                switch (out->gid) {
                    case 0x0A:
                    switch (out->oid) {
                        case 0x00: printf("\033[0;34m\tResponse: RSP_RESET_DEVICE\033[0m\n"); break;
                        case 0x02: printf("\033[0;34m\tResponse: RSP_GET_FW_VERSION\033[0m\n"); break;
                        case 0x04: printf("\033[0;34m\tResponse: RSP_RX_CONFIG\033[0m\n"); break;
                        case 0x05: printf("\033[0;34m\tResponse: RSP_TX_CONFIG\033[0m\n"); break;
                        case 0x06: printf("\033[0;34m\tResponse: RSP_START_BASEBAND\033[0m\n"); break;
                        case 0x07: printf("\033[0;34m\tResponse: RSP_STOP_BASEBAND\033[0m\n"); break;
                        case 0x08: printf("\033[0;34m\tResponse: RSP_GET_BASEBAND_RESULTS\033[0m\n"); break;
                        case 0x09: printf("\033[0;34m\tResponse: RSP_POWER_MGMT_CONFIG\033[0m\n"); break;
                        case 0x0A: printf("\033[0;34m\tResponse: RSP_GENERIC_BASEBAND_CONFIG\033[0m\n"); break;
                        case 0x0B: printf("\033[0;34m\tResponse: RSP_CLOCK_MGMT_CONFIG\033[0m\n"); break;
                        case 0x0C: printf("\033[0;34m\tResponse: RSP_PIN_TOGGLE_CONFIG\033[0m\n"); break;
                        case 0x0D: printf("\033[0;34m\tResponse: RSP_GPADC_OPERATION\033[0m\n"); break;
                        case 0x13: printf("\033[0;34m\tResponse: RSP_SET_BITFIELD\033[0m\n"); break;
                        case 0x14: printf("\033[0;34m\tResponse: RSP_GET_BITFIELD\033[0m\n"); break;
                        case 0x15: printf("\033[0;34m\tResponse: RSP_ECHO_REV_PAYLOAD\033[0m\n"); break;
                        case 0x18: printf("\033[0;34m\tResponse: RSP_RX_RADIO_CONFIG\033[0m\n"); break;
                        case 0x19: printf("\033[0;34m\tResponse: RSP_TX_RADIO_CONFIG\033[0m\n"); break;
                        case 0x20: printf("\033[0;34m\tResponse: RSP_RANGING_PAYLOAD_CONFIG\033[0m\n"); break;
                        case 0x21: printf("\033[0;34m\tResponse: RSP_RANGING_APPLICATION_CONFIG\033[0m\n"); break;
                        case 0x22: printf("\033[0;34m\tResponse: RSP_RANGING_SEQUENCE_CONFIG\033[0m\n"); break;
                        case 0x23: printf("\033[0;34m\tResponse: RSP_START_RANGING\033[0m\n"); break;
                        case 0x24: printf("\033[0;34m\tResponse: RSP_GET_RANGING_PAYLOAD\033[0m\n"); break;
                        case 0x30: printf("\033[0;34m\tResponse: RSP_RADAR_APPLICATION_CONFIG\033[0m\n"); break;
                        case 0x31: printf("\033[0;34m\tResponse: RSP_START_RADAR\033[0m\n"); break;
                        case 0x3E: printf("\033[0;34m\tResponse: RSP_CONFIG_CAN\033[0m\n"); break;
                        case 0x3F: printf("\033[0;34m\tResponse: RSP_CONFIG_UCI\033[0m\n"); break;
                        default: printf("\tRespuesta no reconocida. GID=0x%02X, OID=0x%02X\n", out->gid, out->oid); return UCI_PACKET_INVALID;
                    }
                    break;
                }
            break;
            case 0x03: // NTF
                switch (out->oid) {
                    case 0x01: printf("\033[0;34m\tNotificación: NTF_BOOT_STATUS\033[0m\n"); break;
                    case 0x07: printf("\033[0;34m\tNotificación: NTF_CORE_GENERIC_ERROR (Formato incorrecto)\033[0m\n"); break;
                    case 0x0B: printf("\033[0;34m\tNotificación: NTF_CLOCK_MGMT\033[0m\n"); break;
                    case 0x0D: printf("\033[0;34m\tNotificación: NTF_GPADC_RESULT\033[0m\n"); break;
                    case 0x0E: printf("\033[0;34m\tNotificación: NTF_BASEBAND_STATUS\033[0m\n"); break;
                    case 0x23: printf("\033[0;34m\tNotificación: NTF_RANGING_STATUS\033[0m\n"); break;
                    case 0x31: printf("\033[0;34m\tNotificación: NTF_RADAR_RESULT\033[0m\n"); break;
                    default:  printf("\tNotificación no reconocida. GID=0x%02X, OID=0x%02X\n", out->gid, out->oid);  return UCI_PACKET_INVALID;
                }
                break;
            default: printf("\tTipo de mensaje desconocido: MT=0x%02X\n", out->mt); return UCI_PACKET_INVALID;
        }
    }

    return UCI_PACKET_OK;  // Fallback de seguridad

}

typedef enum {
    STATUS_OK                  = 0x00,
    STATUS_REJECTED            = 0x01,
    STATUS_FAILED              = 0x02,
    STATUS_SYNTAX_ERROR        = 0x03,
    STATUS_INVALID_PARAM       = 0x04,
    STATUS_INVALID_RANGE       = 0x05,
    STATUS_INVALID_MESSAGE_SIZE= 0x06,
    STATUS_UNKNOWN_GID         = 0x07,
    STATUS_UNKNOWN_OID         = 0x08,
    STATUS_READ_ONLY           = 0x09,
    STATUS_COMMAND_RETRY       = 0x0A,
    STATUS_CRC_ERROR           = 0xF8,
    STATUS_RESERVED            = 0xFF  // RFU o desconocido
} resp_status_t;


void print_resp_status(uint8_t code) {
    switch ((resp_status_t)code) {
        case STATUS_OK: printf("\t\tSTATUS_OK: Operación exitosa\n"); break;
        case STATUS_REJECTED:  printf("\t\tSTATUS_REJECTED: Operación no permitida en el estado actual\n"); break;
        case STATUS_FAILED: printf("\t\tSTATUS_FAILED: La operación falló\n"); break;
        case STATUS_SYNTAX_ERROR: printf("\t\tSTATUS_SYNTAX_ERROR: Estructura de paquete inválida\n"); break;
        case STATUS_INVALID_PARAM: printf("\t\tSTATUS_INVALID_PARAM: Parámetro válido pero valor incorrecto\n"); break;
        case STATUS_INVALID_RANGE: printf("\t\tSTATUS_INVALID_RANGE: Valor fuera de rango\n"); break;
        case STATUS_INVALID_MESSAGE_SIZE: printf("\t\tSTATUS_INVALID_MESSAGE_SIZE: Tamaño de mensaje incorrecto\n"); break;
        case STATUS_UNKNOWN_GID: printf("\t\tSTATUS_UNKNOWN_GID: GID desconocido\n"); break;
        case STATUS_UNKNOWN_OID: printf("\t\tSTATUS_UNKNOWN_OID: OID desconocido\n"); break;
        case STATUS_READ_ONLY: printf("\t\tSTATUS_READ_ONLY: Campo de solo lectura\n"); break;
        case STATUS_COMMAND_RETRY: printf("\t\tSTATUS_COMMAND_RETRY: Se requiere reintento del comando\n"); break;
        case STATUS_CRC_ERROR: printf("\t\tSTATUS_CRC_ERROR: Error de CRC\n"); break;
        default: printf("\t\tCódigo desconocido o reservado: 0x%02X\n", code); break;
    }
}


void handle_ntf_core_generic(const uint8_t *payload, size_t payload_len) {
    if (!payload || payload_len != 1) {
        printf("[GENERIC_NTF] Payload inválido o tamaño incorrecto (%zu)\n", payload_len);
        return;
    }

    uint8_t status = payload[0];
    printf("\t\t[GENERIC_NTF] RESP_STATUS = 0x%02X\n", status);
    print_resp_status(status);
}

void handle_resp_status(const uint8_t *payload, size_t payload_len) {

    uint8_t status = payload[payload_len - 1];
    if (status == 0x00) {
        printf("\t\t[RESP_STATUS] RESP_STATUS = 0x%02X (OK)\n", status);
        return; // No hay nada más que hacer si es OK
    }
    else{
        printf("\033[0;31m\t\t[RESP_STATUS] RESP_STATUS = 0x%02X (Error)\033[0m\n", status);
        printf("\033[0;31m");
        print_resp_status(status);
        printf("\033[0m");
    }
   
}

typedef enum {
    RX_Success                 = 0x00,
    TX_Success                 = 0x01,
    Calibration_Done           = 0x02,
    Aborted                    = 0x03,
    RX_error                   = 0x10,
    TX_error                   = 0x11,
    General_Baseband_error     = 0x12
} radar_status_t;

void print_radar_status(uint8_t code) {
    switch ((radar_status_t)code) {
        case RX_Success: 
            printf("\t\tRX_SUCCESS_CIR_AVAILABLE: Successful reception and CIR available\n"); 
            break;
        case TX_Success: 
            printf("\t\tTX_Success: Successful transmission\n"); 
            break;
        case Calibration_Done: 
            printf("\t\tCalibration_Done: Calibration completed\n"); 
            break;
        case Aborted: 
            printf("\t\tAborted: Operation aborted\n"); 
            break;
        case RX_error: 
            printf("\t\tRX_error: Reception error\n"); 
            break;
        case TX_error: 
            printf("\t\tTX_error: Transmission error\n"); 
            break;
        case General_Baseband_error: 
            printf("\t\tGeneral_Baseband_error: General baseband error\n"); 
            break;
        default: 
            printf("\t\tUnknown or reserved code: 0x%02X\n", code); 
            break;
    }
}

#define CIR_BYTES (CIR_TAPS * 4) // 32 taps * 4 bytes/tap = 128 bytes


void interpret_ntf_radar_result(const uint8_t *payload, size_t payload_len) {
    if (!payload || payload_len < 1) {
        printf("[RADAR_NTF] Payload inválido o tamaño incorrecto (%zu)\n", payload_len);
        return;
    }
    printf("\t[RADAR_NTF] Interpretando payload y extrayendo datos...\n");
    // Interpretar el primer byte como el estado del radar
    uint8_t radar_status = payload[4];
    printf("\t\t[RADAR_NTF] Radar status: 0x%02X\n", radar_status);
    print_radar_status(radar_status);

    // 
    uint8_t antenna_index = payload[5];
    printf("\t\t[RADAR_NTF] Antenna index: : 0x%02X\n", antenna_index);

    if (payload_len >= 3) {
        uint32_t agc_gain = (payload[6]) | (payload[7] << 8) | (payload[8] << 16) | (payload[9] << 24);
        printf("\t\t[RADAR_NTF] AGC Gain: 0x%08lX (%lu)\n", agc_gain, agc_gain);
    } else {
        printf("\t\t[RADAR_NTF] AGC Gain data not available.\n");
    }

    // Extraer el índice del CIR desde el payload
    if (payload_len >= 10) {
        uint16_t cir_idx = (payload[10] << 8) | payload[11];
        printf("\t\t[RADAR_NTF] CIR Index: 0x%04X\n", cir_idx);
    } else {
        printf("\t\t[RADAR_NTF] CIR Index data not available.\n");
    }

    if (payload_len >= 12) {
        uint32_t cir_offset = (payload[15] << 24) |
                              (payload[14] << 16) |
                              (payload[13] << 8) |
                               payload[12];
        printf("\t\t[RADAR_NTF] CIR Offset: 0x%08lX\n", cir_offset);
    } else {
        printf("\t\t[RADAR_NTF] CIR Offset data not available.\n");
    }

    uint32_t num_taps = 0;
    if (payload_len >= 12) {
        num_taps = (payload[19] << 24) | (payload[18] << 16) |  (payload[17] << 8) |  payload[16];
        printf("\t\t[RADAR_NTF] Number of Taps: 0x%08lX (%lu)\n", num_taps, num_taps);
        // printf("\t\t[RADAR_NTF] Number of Taps: 0x%X\n", num_taps); // versión antigua
    } else {
        printf("\t\t[RADAR_NTF] Number of Taps data not available.\n");
    }

    if (payload_len >= 16) {
        uint32_t timestamp = (payload[23] << 24) |
                             (payload[22] << 16) |
                             (payload[21] << 8) |
                              payload[20];
        printf("\t\t[RADAR_NTF] Timestamp: 0x%08lX (%lu)\n", timestamp, timestamp);
        // printf("\t\t[RADAR_NTF] Timestamp (decimal): \n", timestamp);
    } else {
        printf("\t\t[RADAR_NTF] Timestamp data not available.\n");
    }

    if (payload_len >= 24 + 4 * (unsigned long)num_taps) {
        printf("\t\t[RADAR_NTF] CIR Taps:\n");


        // Copiar los 128 bytes (máximo 32 taps × 4 bytes) directamente desde el payload
        uint8_t cir_bytes[CIR_BYTES] = {0};
        memcpy(cir_bytes, &payload[24], CIR_BYTES);

        // Imprimir por debug:
        /* for (uint32_t i = 0; i < num_taps; i++) {
            uint32_t tap = (payload[24 + 4 * i] << 24) |
                           (payload[25 + 4 * i] << 16) |
                           (payload[26 + 4 * i] << 8) |
                            payload[27 + 4 * i];

            // printf("\t\t\tTap %lu: 0x%08lX", i, tap);
            // if ((i + 1) % 2 == 0) {
            //     printf("\n");
            // } else {
            //     printf("\t");
            // }
        }     */
       
        // Enviar directamente por la cola, sin reorganizar nada
        if (xQueueSend(cir_queue, cir_bytes, portMAX_DELAY) != pdTRUE) {
            printf("\n\t\t[ERROR] Cola de CIRs llena, no se pudo enviar.\n");
        } else {
            // printf("\n\t\t[RADAR_NTF] CIR enviado a la cola.\n");
            // printf("\033[0;31mManda cir por cola\033[0m\n"); //para debug

            // Imprimir el contenido del CIR enviado a la cola (SOLO PARA DEBUG)
            // printf("\t\t[RADAR_NTF] CIR enviado a la cola (contenido):\n");
            // for (uint32_t i = 0; i < CIR_BYTES; i++) {
            //     printf("%02X ", cir_bytes[i]);
            //     if ((i + 1) % 16 == 0) {
            //     printf("\n");
            //     }
            // }
            // printf("\n");
        }

    } else {
        printf("\t\t[RADAR_NTF] CIR Taps data not available.\n");
    }
}