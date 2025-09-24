#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "gpio.h"
#include "spi_interface.h"
#include "uwb_core.h"
#include "uwb_api.h"
#include "uwb_commands.h"
#include "sync.h"
#include <string.h>
#include "task_control.h"
#include "Config.h"

#include "radar_config.h"


bool success = true;

bool interrupt_processing_enabled = false;

bool conf_uwb_active = true;

extern QueueHandle_t commandQueue_UWB;  // Cola para recibir comandos del usuario (ej. iniciar, detener)
bool streaming = false;

void task_uwb(){
    while(true){
        if(!streaming){
            if(!uwb_init_device()){
                printf("\n>> UWB module initialization failed.\n");
                continue;
            }
            if(!uwb_conf_device()) {
                printf("\n>> UWB module configuration failed.\n");
                continue;
            }
            if(!uwb_start_streaming()) {
                printf("\n>> UWB module streaming failed.\n");
                continue;
            }
            streaming = true;
            printf("\n>> UWB module streaming started.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


bool initializeDevice() {

    hard_reset(); // Reiniciar el UWB para asegurarse de que comienza desde un estado limpio

    //print_gpio_states();
    
    if (!wait_for_gpio_low(GPIO_INT_IO, RDY_TIMEOUT_MS)) return false;

    printf("\t[INFO] Dispositivo UWB inicializado correctamente.\n");

    return true;
}

bool uwb_init_device() {

    int timeout_wait = 500;
    
    // Inicialización del módulo UWB

    printf("\n>> Initializing UWB module...\n");

    initializeDevice();
    
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // 0. GET_FW_VERSION
    printf("\n0. Getting firmware version...\n");
    if(!uwb_get_version()) {
        printf("\n>> Failed to get firmware version.\n");
        return false;
    }

    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    return true;
}

bool uwb_conf_device(){

    int timeout_wait = 500;
    // Configuración del módulo UWB
    // CONFIGURATION PROCEDURE 
    //bool conf_uwb_active = true;

    printf("\n>> Starting UWB module configuration...\n");
    
    // 1. CMD_SET_BITFIELD: Disable Watchdog
    printf("\n1. Disabling watchdog...\n");
    if(!uwb_disable_watchdog()) {
        printf("\n>> Failed to disable watchdog.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 2. RX Radio Config
    printf("\n2. Configuring RX radio...\n");
    if(!uwb_generic_baseband_config(CFG_RX_RADIO)) {
        printf("\n>> Failed to configure RX radio.\n");
        return false;
    }
    // uwb_configure_rx_radio();
    // // Optional: CMD_RX_RADIO_CONFIG (if binary present) → omitido ahora
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 3. TX Radio Config
    printf("\n3. Configuring TX radio...\n");
    if(!uwb_generic_baseband_config(CFG_TX_RADIO)) {
        printf("\n>> Failed to configure TX radio.\n");
        return false;
    }
    // uwb_configure_rx_radio();
    // // Optional: CMD_TX_RADIO_CONFIG (if binary present) → omitido ahora
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 4. TX Power Config
    printf("\n4. Configuring TX power...\n");
    if(!uwb_generic_baseband_config(CFG_TX_POWER)) {
        printf("\n>> Failed to configure TX power.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 5. CIR Config
    printf("\n5. Configuring CIR...\n");
    if(!uwb_generic_baseband_config(CFG_RX_RADAR_CIR)) {
        printf("\n>> Failed to configure CIR.\n");
        return false;
    }
    // uwb_configure_radar_cir();
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 6. Noise suppression
    printf("\n6. Configuring noise suppression...\n");
    if(!uwb_generic_baseband_config(CFG_RADAR_NOISE_SUPP)) {
        printf("\n>> Failed to configure noise suppression.\n");
        return false;
    }
    // uwb_configure_noise_suppression();
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 7. RX Control
    printf("\n7. Configuring RX radar control...\n");
    if(!uwb_generic_baseband_config(CFG_RX_RADAR_CTRL)) {
        printf("\n>> Failed to configure RX radar control.\n");
        return false;
    }
    // uwb_configure_rx_radar_control();
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 8. TX Control
    printf("\n8. Configuring TX radar control...\n");
    if(!uwb_generic_baseband_config(CFG_TX_RADAR_CTRL)) {
        printf("\n>> Failed to configure TX radar control.\n");
        return false;
    }
    // uwb_configure_tx_radar_control();
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 9. RADAR_APP_CONFIG → Full calibration
    printf("\n9. Configuring radar application in full calibration mode...\n");
    if(!uwb_configure_radar_application(0x03, 0x28, 0x01)) {
        printf("\n>> Failed to configure radar application for full calibration.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 10. START_RADAR → Full calibration
    printf("\n10. Starting radar full calibration...\n");
    if(!uwb_start_radar()) {
        printf("\n>> Failed to start radar for full calibration.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // Optional: Apply RX_CALIB_DATA → omitido aquí

    // // // 11. RADAR_APP_CONFIG → Fast calibration
    // printf("\n11. Configuring radar application in fast calibration mode...\n");
    // uwb_configure_radar_application();
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    // // // 12 START_RADAR → Fast calibration
    // printf("\n12. Starting radar...\n");
    // uwb_start_radar();
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    // // 11. RADAR_APP_CONFIG → Noise suppression calibration
    printf("\n11. Configuring radar application in noise suppression calibration mode...\n");
    if(!uwb_configure_radar_application(0x05, 0x14, 0x01)) {
        printf("\n>> Failed to configure radar application for noise suppression calibration.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 12. START_RADAR → Noise suppression calibration
    printf("\n12. Starting radar RNS calibration...\n");
    if(!uwb_start_radar()) {
        printf("\n>> Failed to start radar for noise suppression calibration.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 13. GET_BASEBAND_RESULT → AGC Gain RX1
    printf("\n13. Getting AGC Gain RX1...\n");
    if(!uwb_get_baseband_results(ACC_GAIN_RESULTS, RX1)) {
        printf("\n>> Failed to get AGC Gain RX1.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 14. GET_BASEBAND_RESULT → AGC Gain RX2
    printf("\n14. Getting AGC Gain RX2...\n");
    if(!uwb_get_baseband_results(ACC_GAIN_RESULTS, RX2)) {
        printf("\n>> Failed to get AGC Gain RX2.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);
    
    // // 15. GET_BASEBAND_RESULT → CALIB GAIN RX1
    printf("\n15. Getting CALIB Gain RX1...\n");
    if(!uwb_get_baseband_results(CALIB_GAIN_RESULTS, RX1)) {
        printf("\n>> Failed to get CALIB Gain RX1.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 16. GET_BASEBAND_RESULT → CALIB GAIN RX2
    printf("\n16. Getting CALIB Gain RX2...\n");
    if(!uwb_get_baseband_results(CALIB_GAIN_RESULTS, RX1)) {
        printf("\n>> Failed to get CALIB Gain RX2.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // // 17. GET_BASEBAND_RESULT → RX_CALIB_DATA
    printf("\n17. Getting RX calibration data...\n");
    if(!uwb_get_baseband_results(CALIB_DATA, 0x00)) {
        printf("\n>> Failed to get RX calibration data.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);
   
    return true;
}

bool uwb_start_streaming(){
    
    int timeout_wait = 500;
    // // 18. RADAR_APP_CONFIG → Final streaming config
    printf("\n18. Configuring radar application in streaming mode...\n");
    if(!uwb_configure_radar_application(0x00, 0x01, 0x00)) {
        printf("\n>> Failed to configure radar application for streaming mode.\n");
        return false;
    }
    //uwb_configure_radar_application(0x00, 0x20, 0x05);
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    // Small delay to ensure the configuration is applied
    printf("UWB module configuration completed successfully.\n");
    conf_uwb_active = false;
    start_acq_algorithm();
                    
    // print_gpio_states();
    // char start_command = 's';
    // xQueueSend(commandQueue, &start_command, portMAX_DELAY); // Enviar comando de inicio de adquisición
    // printf("\n[CONTROL] Command 's' sent to start acquisition.\n");
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);
    
    // // 19. START_RADAR → Start streaming
    printf("\n19. Starting radar CIR streaming...\n");
    if(!uwb_start_radar()) {
        printf("\n>> Failed to start radar for streaming.\n");
        return false;
    }
    vTaskDelay(timeout_wait / portTICK_PERIOD_MS);

    return true;
}

bool uwb_get_version(){
    printf("\n[GET FW VERSION]\n");
    uint8_t cmd_buffer[6] = {0};  // 2 floats = 8 bytes = suficiente para los 6 bytes
    size_t cmd_len = 0;

    // VERSION NUEVA
    build_command_get_fw_version(cmd_buffer, &cmd_len);

    printf("\tComando CMD_GET_FW_VERSION: ");
    for (int i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_reset_device(){
    uint8_t cmd_buffer[7] = {0};  // 2 floats = 8 bytes = suficiente para los 6 bytes
    size_t cmd_len = 0;
    uint8_t reset_type = 0x00; // Reset type: 0x00 = Soft reset, 0x01 = Hard reset
    
    //VERSION NUEVA
    build_command_reset_device(cmd_buffer, &cmd_len, reset_type);
    
    // Debug de comando
    printf("\tComando CMD_RESET_DEVICE: ");
    for (int i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_disable_watchdog(){
    uint8_t cmd_buffer[16] = {0};  // 2 floats = 8 bytes = suficiente para los 6 bytes
    size_t cmd_len = 0;
    build_command_set_bitfield(cmd_buffer, &cmd_len, 0x4000F040, 0x00, 0x20, 0x00000000);
 
    printf("\tComando CMD_DISABLE_WATCHDOG: ");
    for (int i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    // Enviar comando CMD_DISABLE_WATCHDOG
    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_read_bitfield() {
    printf("\n[GET BITFIELD]\n");
    uint8_t cmd_buffer[16] = {0};
    size_t cmd_len = 0;

    uint32_t address = 0x4000F040;  // Dirección de prueba
    uint8_t offset = 0x00;
    uint8_t width = 0x20;

    bool ok = build_command_get_bitfield(cmd_buffer, &cmd_len, address, offset, width);

    if (ok) {
        printf("\tComando CMD_GET_BITFIELD: ");
        for (size_t i = 0; i < cmd_len; i++) {
            printf("%02X ", cmd_buffer[i]);
        }
        printf("\n");

        return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
    } else {
        printf("\t[FALLO] No se pudo construir el comando GET_BITFIELD\n");
        return false;
    }
}

bool uwb_generic_baseband_config(uint8_t config_select){
    uint8_t cmd_buffer[50] = {0};  // Tamaño máximo por especificación
    size_t cmd_len = 0;

    if(!build_command_generic_baseband_config(cmd_buffer, &cmd_len, config_select)) {
        printf("\tError al construir el comando CMD_GENERIC_BASEBAND_CONFIG\n");
        return false;
    }


    printf("\tComando CMD_GENERIC_BASEBAND_CONFIG: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_get_baseband_results(uint8_t result_sel, uint8_t results_idx) {
    uint8_t cmd_buffer[16] = {0};
    size_t cmd_len = 0;

    if(!build_command_get_baseband_results(cmd_buffer, &cmd_len, result_sel, results_idx)) {
        printf("\tError al construir el comando CMD_GET_BASEBAND_RESULTS\n");
        return false;
    }
    
    printf("\tComando CMD_GET_BASEBAND_RESULTS: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}


bool uwb_configure_radar_application(uint8_t radar_mode, uint8_t frame_cnt_byte, uint8_t burst_cnt_byte) {
    uint8_t cmd_buffer[40] = {0};
    size_t cmd_len = 0;

    // Parámetros de configuración (todos en little-endian)
    uint8_t uwb_freq[4]        = {0x00, 0xE0, 0x79, 0x00};   // 6.4896 GHz
    uint8_t ppm[2]             = {0x00, 0x00};               // +10.5 ppm
    uint8_t keep_syspll        = 0x00;
    uint8_t pll_mode           = 0x01;                       // Fraccional
    uint8_t init_delay[4]      = {0xE8, 0x03, 0x00, 0x00};   // 1000 us
    //uint8_t radar_mode         = radar_mode;                 // 0x03 = Full calibration, 0x05 = Noise suppression calibration, 0x00 = Streaming/TRX mode
    uint8_t rx_cfg_idx         = 0x00;
    uint8_t tx_cfg_idx         = 0x00;
    uint8_t notif_enbl         = 0x01;                       // RX/TX habilitados
    uint8_t frame_cnt[4]       = {frame_cnt_byte, 0x00, 0x00, 0x00};   // 0x28 = 40, 0x14 = 20, 0x01 = 1
    uint8_t frame_interval[4]  = {0x03, 0x0B, 0x13, 0x00};   // 250 µs
    uint8_t burst_cnt[4]       = {burst_cnt_byte, 0x00, 0x00, 0x00};   // 0x01 = 1 burst, 0x00 = 0 bursts (deshabilitado)
    uint8_t burst_delay[4]     = {0x03, 0x0B, 0x13, 0x00};   // 500 µs

    // Construir el comando
    build_command_radar_application_config(
        cmd_buffer, &cmd_len,
        uwb_freq, ppm, keep_syspll, pll_mode, init_delay,
        radar_mode, rx_cfg_idx, tx_cfg_idx, notif_enbl,
        frame_cnt, frame_interval, burst_cnt, burst_delay
    );

    printf("\tComando CMD_RADAR_APP_CONFIG: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

 
    return send_uci_cmd_get_rsp(cmd_buffer, sizeof(cmd_buffer));
}

bool uwb_start_radar(){
    uint8_t cmd_buffer[6] = {0};  // 2 floats = 8 bytes = suficiente para los 6 bytes
    size_t cmd_len = 0;

    build_command_start_radar(cmd_buffer, &cmd_len);
    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}


/// ============================================================================================================
/// ============================================================================================================
/// ============================================================================================================
/// ============================================================================================================














































bool uwb_configure_rx_radio(){
    uint8_t cmd_buffer[128] = {0};
    size_t cmd_len = 0;

    uint8_t sts_config[56] = {0xAA};  // Placeholder
    uint8_t rx_radio_cfg[4] = {0x10, 0x11, 0x12, 0x13};
    uint8_t rx_dfe_cfg[12] = {0xBB};
    uint8_t rx_range_ctrl_cfg[16] = {0xCC};
    uint8_t rx_payload_cfg[4] = {0x20, 0x21, 0x22, 0x23};

    build_command_rx_config(
        cmd_buffer,
        &cmd_len,
        sts_config,
        0x01,              // RX_RADIO_CONFIGURATION_INDEX
        rx_radio_cfg,
        0x02,              // RX_DFE_CONFIGURATION_INDEX
        rx_dfe_cfg,
        0x03,              // RX_RANGING_CONTROL_CONFIGURATION_INDEX
        rx_range_ctrl_cfg,
        rx_payload_cfg
    );

    printf("\tComando CMD_RX_CONFIG: ");
    for (int i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_configure_tx_radio(){
    uint8_t cmd_buffer[128] = {0};
    size_t cmd_len = 0;
    
    // ==== STS_CONFIGURATION (56 bytes) ====
    uint8_t sts_config[56];
    memset(sts_config, 0xAA, sizeof(sts_config));  // Placeholder

    // ==== TX_RADIO_CONFIGURATION ====
    uint8_t tx_radio_cfg_idx = 0x01;
    uint8_t tx_radio_cfg[4] = {0x10, 0x11, 0x12, 0x13};

    // ==== TX_POWER_CONFIGURATION ====
    uint8_t tx_power_cfg_idx = 0x02;
    uint8_t tx_power_cfg[4] = {0x22, 0x23, 0x24, 0x25};

    // ==== TX_RANGING_CONTROL_CONFIGURATION ====
    uint8_t tx_range_ctrl_cfg_idx = 0x03;
    uint8_t tx_range_ctrl_cfg[12];
    memset(tx_range_ctrl_cfg, 0xBB, sizeof(tx_range_ctrl_cfg));  // Placeholder

    // ==== TX_PAYLOAD_CONFIGURATION ====
    uint8_t tx_payload_cfg[4] = {0x31, 0x32, 0x33, 0x34};
    size_t tx_payload_cfg_len = sizeof(tx_payload_cfg);

    build_command_tx_config(cmd_buffer, &cmd_len,
                                      sts_config,
                                      tx_radio_cfg_idx, tx_radio_cfg,
                                      tx_power_cfg_idx, tx_power_cfg,
                                      tx_range_ctrl_cfg_idx, tx_range_ctrl_cfg,
                                      tx_payload_cfg, tx_payload_cfg_len);

    printf("\tComando CMD_TX_CONFIG: ");
    for (int i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_start_baseband() {
    uint8_t cmd_buffer[64] = {0};
    size_t cmd_len = 0;

    // Definir parámetros del payload
    uint8_t uwb_freq[4]      = {0x00, 0x06, 0x63, 0x00};
    uint8_t ppm[2]           = {0x00, 0x69};
    uint8_t init_delay[4]    = {0x00, 0x00, 0x03, 0xE8};
    uint8_t pll_mode         = 0x00;
    uint8_t cfg_idx          = 0xFF;
    uint8_t cmd_cfg[20];     memset(cmd_cfg, 0xAA, sizeof(cmd_cfg));
    uint8_t notif_sel[4]     = {0x01, 0x00, 0x00, 0x00};
    uint8_t start_idx[2]     = {0x00, 0x00};
    uint8_t end_idx[2]       = {0x00, 0x00};
    uint8_t cmd_seq_cnt[4]   = {0x00, 0x00, 0x00, 0x01};

    // Construir comando
    build_command_start_baseband(cmd_buffer, &cmd_len,
         uwb_freq, ppm, init_delay, pll_mode, cfg_idx,
         cmd_cfg, notif_sel, start_idx, end_idx, cmd_seq_cnt);

    printf("\tComando CMD_START_BASEBAND: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

bool uwb_stop_baseband() {
    uint8_t cmd_buffer[16] = {0};
    size_t cmd_len = 0;

    if (!build_command_stop_baseband(cmd_buffer, &cmd_len)) {
        printf("\tError al construir el comando STOP_BASEBAND\n");
        return false;
    }

    printf("\tComando CMD_STOP_BASEBAND: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    return send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}