#include <stdbool.h>
#include "gpio.h"
#include "spi_interface.h"
#include "uwb_api.h"
#include "uwb_core.h"
#include "uwb_commands.h"
//#include "sync.h"
#include <string.h>
#define JSMN_PARENT_LINKS // AGREGADO

#include "radar_config.h"
#include "radar_config_json.h"

const char *json_config_str = "{"
"\"device_index\": 0,"
"\"TX_RADIO_CONFIGURATION\": {\"RADIO_CONFIG_SOURCE\": 7},"
"\"RX_RADIO_CONFIGURATION\": {\"RADIO_CONFIG_SOURCE\": 7},"
"\"CMD_RADAR_APPLICATION_CONFIG\": {"
    "\"UWB_CHANNEL_FREQUENCY\": 7987200,"
    "\"PPM\": 0,"
    "\"KEEP_SYSPLL\": 0,"
    "\"RADAR_SYSPLL_MODE\": 1,"
    "\"INIT_DELAY\": 1000,"
    "\"MODE\": 0,"
    "\"RX_RADAR_CONTROL_CONFIGURATION_IDX\": 0,"
    "\"TX_RADAR_CONTROL_CONFIGURATION_IDX\": 0,"
    "\"NOTIFICATION_ENBL\": 1,"
    "\"FRAME_CNT\": 1,"
    "\"FRAME_INTERVAL\": 10000000,"
    "\"BURST_CNT\": 0,"
    "\"BURST_DELAY\": 10000000"
"},"
"\"TX_POWER_CONFIGURATION\": {"
    "\"TX_POWER_NOMINAL\": -9,"
    "\"TX_POWER_OFFSET\": 0,"
    "\"TX_POWER_BOOST\": 0"
"},"
"\"RX_RADAR_CIR_CONFIGURATION\": {"
    "\"CIR_TAPS\": 64,"
    "\"RX_SYNC_SYMBOL_CNT\": 256,"
    "\"CIR_OFFSET\": 0,"
    "\"SENSITIVITY_BOOST_OFFSET_RX1\": 0,"
    "\"SENSITIVITY_BOOST_OFFSET_RX2\": 2,"
    "\"CIR_THRESHOLD_NTF\": 1,"
    "\"TIMESTAMP_LOGGING_ENBL\": 1,"
    "\"CIR_BUFFER_SIZE\": 2"
    "\"DATA_ACQ_MODE\": {"
        "\"Analog coupling\": 1,"
        "\"Radar data selection\": 0,"
        "\"AGC control for Radar fast calibration\": 1"
    "}"
"},"
"\"TX_RADAR_CONTROL_CONFIGURATION\": {"
    "\"TX_ENBL\": 1,"
    "\"TX_SYNC_SYMBOL_CNT\": 256"
"},"
"\"RX_RADAR_CONTROL_CONFIGURATION\": {"
    "\"RX_RADAR_CALIBRATION_SELECT\": 0,"
    "\"RNS_CONFIGURATION_INDEX\": 0,"
    "\"RX_ENBL\": 2,"
    "\"SENSITIVITY_BOOST\": 2"
"},"
"\"RADAR_NOISE_SUPPRESSION_CONFIGURATION\": {"
    "\"RNS_RX1\": {"
        "\"SELF_INTERFERENCE_TAP\": 12,"
        "\"RNS_CONTROL\": {"
            "\"Radar noise suppression\": 0,"
            "\"Radar drift suppression\": 0,"
            "\"DC removal\": 1"
        "},"
        "\"NOISE_SUPPRESSION_INDEX_MASK\": \"0000 0000 0000 0000 0000 1111 1111 1111\","
        "\"DC_FILTER_CUTOFF_HZ\": 0.2,"
        "\"RNS_CALIBRATION_SLOT_INDEX\": 0"
    "},"
    "\"RNS_RX2\": {"
        "\"SELF_INTERFERENCE_TAP\": 13,"
        "\"RNS_CONTROL\": {"
            "\"Radar noise suppression\": 0,"
            "\"Radar drift suppression\": 0,"
            "\"DC removal\": 1"
        "},"
        "\"NOISE_SUPPRESSION_INDEX_MASK\": \"0000 0000 0000 0000 0000 1111 1111 1111\","
        "\"DC_FILTER_CUTOFF_HZ\": 0.2,"
        "\"RNS_CALIBRATION_SLOT_INDEX\": 1"
    "}"
"},"
"\"duration_sec\": 60"
"}";

bool success = true;

void uwb_get_version(){
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

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_reset_device(){
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

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_disable_watchdog(){
    uint8_t cmd_buffer[16] = {0};  // 2 floats = 8 bytes = suficiente para los 6 bytes
    size_t cmd_len = 0;
    build_command_set_bitfield(cmd_buffer, &cmd_len, 0x4000F040, 0x00, 0x20, 0x00000000);
 
    printf("\tComando CMD_DISABLE_WATCHDOG: ");
    for (int i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    // Enviar comando CMD_DISABLE_WATCHDOG
    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_read_bitfield() {
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

        send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
    } else {
        printf("\t[FALLO] No se pudo construir el comando GET_BITFIELD\n");
    }
}

void uwb_generic_baseband_config(uint8_t config_select){
    uint8_t cmd_buffer[50] = {0};  // Tamaño máximo por especificación
    size_t cmd_len = 0;

    if(!build_command_generic_baseband_config(cmd_buffer, &cmd_len, config_select)) {
        printf("\tError al construir el comando CMD_GENERIC_BASEBAND_CONFIG\n");
        return;
    }


    printf("\tComando CMD_GENERIC_BASEBAND_CONFIG: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_get_baseband_results(uint8_t result_sel, uint8_t results_idx) {
    uint8_t cmd_buffer[16] = {0};
    size_t cmd_len = 0;

    if(!build_command_get_baseband_results(cmd_buffer, &cmd_len, result_sel, results_idx)) {
        printf("\tError al construir el comando CMD_GET_BASEBAND_RESULTS\n");
        return;
    }
    
    printf("\tComando CMD_GET_BASEBAND_RESULTS: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_configure_radar_application(uint8_t radar_mode, uint8_t frame_cnt_byte, uint8_t burst_cnt_byte) {
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

 
    send_uci_cmd_get_rsp(cmd_buffer, sizeof(cmd_buffer));
}

void uwb_start_radar(){
    uint8_t cmd_buffer[6] = {0};  // 2 floats = 8 bytes = suficiente para los 6 bytes
    size_t cmd_len = 0;

    build_command_start_radar(cmd_buffer, &cmd_len);
    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_configure_rx_radio(){
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

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}


void uwb_configure_tx_radio(){
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

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_start_baseband() {
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

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}

void uwb_stop_baseband() {
    uint8_t cmd_buffer[16] = {0};
    size_t cmd_len = 0;

    if (!build_command_stop_baseband(cmd_buffer, &cmd_len)) {
        printf("\tError al construir el comando STOP_BASEBAND\n");
        return;
    }

    printf("\tComando CMD_STOP_BASEBAND: ");
    for (size_t i = 0; i < cmd_len; i++) {
        printf("%02X ", cmd_buffer[i]);
    }
    printf("\n");

    send_uci_cmd_get_rsp(cmd_buffer, cmd_len);
}