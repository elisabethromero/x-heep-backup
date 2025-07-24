#include "uwb_commands.h"
#include <string.h>
#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>

// ========================
// CRC
// ========================
// Polinomio para CRC16/XMODEM
uint16_t crc16_xmodem(const uint8_t *data, size_t length) {
    uint16_t crc = CRC16_INIT;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC16_POLY;            
            else{
                crc = crc << 1;
            }
        }
            
    }
    return crc;
}


// ========================
// Packet Builder
// ========================
// Función genérica de construcción de paquetes
bool build_packet(uint8_t *buffer_out, size_t *len_out, uint8_t mt, uint8_t pbf, 
    uint8_t gid, uint8_t oid, const uint8_t *payload, size_t payload_len) {
    if (!buffer_out || !len_out || (payload_len > 0 && !payload)) return false;

    // HEADER
    buffer_out[0] = ((mt & 0x07) << 5) | ((pbf & 0x01) << 4) | (gid & 0x0F);
    buffer_out[1] = oid & 0x3F;
    buffer_out[2] = (payload_len >> 8) & 0xFF;  // big-endian
    buffer_out[3] = payload_len & 0xFF;

    // PAYLOAD
    for (size_t i = 0; i < payload_len; i++) {
        buffer_out[4 + i] = payload[i];
    }

    // CRC
    size_t crc_offset = 4 + payload_len;
    uint16_t crc = crc16_xmodem(buffer_out, crc_offset);
    buffer_out[crc_offset]     = crc & 0xFF;        // LSB
    buffer_out[crc_offset + 1] = (crc >> 8) & 0xFF; // MSB

    *len_out = crc_offset + 2;
    return true;
}

// =========================================================
// =========================================================

static size_t build_payload_reset_device(uint8_t *payload_out, uint8_t reset_type) {
    if (!payload_out) return 0;

    payload_out[0] = reset_type;
    return 1;
}

bool build_command_reset_device(uint8_t *buffer_out, size_t *len_out, uint8_t reset_type) {
    printf("\n\t[BUILD] Construyendo comando CMD_RESET_DEVICE...\n");
    uint8_t payload[1];
    size_t payload_len = build_payload_reset_device(payload, reset_type);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x00,        // OID = RESET_DEVICE
                        payload, payload_len);
}


bool build_command_get_fw_version(uint8_t *buffer_out, size_t *len_out) {
    printf("\t[BUILD] Construyendo comando CMD_GET_FW_VERSION...\n");
    // Sin payload
    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF = 0
                        0x0A,        // GID = CORE
                        0x02,        // OID = GET_FW_VERSION
                        NULL, 0);    // Payload vacío
}


static size_t build_payload_rx_config(uint8_t *payload_out,
    const uint8_t *sts_config,
    uint8_t rx_radio_cfg_idx,
    const uint8_t *rx_radio_cfg,
    uint8_t rx_dfe_cfg_idx,
    const uint8_t *rx_dfe_cfg,
    uint8_t rx_range_ctrl_cfg_idx,
    const uint8_t *rx_range_ctrl_cfg,
    const uint8_t *rx_payload_cfg) {
    if (!payload_out || !sts_config || !rx_radio_cfg || !rx_dfe_cfg || !rx_range_ctrl_cfg || !rx_payload_cfg)
        return 0;

    size_t offset = 0;

    // STS_CONFIGURATION (56 bytes)
    memcpy(&payload_out[offset], sts_config, 56);
    offset += 56;

    // RX_RADIO_CONFIGURATION_INDEX (1 byte)
    payload_out[offset++] = rx_radio_cfg_idx;

    // RX_RADIO_CONFIGURATION (4 bytes)
    memcpy(&payload_out[offset], rx_radio_cfg, 4);
    offset += 4;

    // RX_DIGITAL_FRONT_END_CONFIGURATION_INDEX (1 byte)
    payload_out[offset++] = rx_dfe_cfg_idx;

    // RX_DIGITAL_FRONT_END_CONFIGURATION (12 bytes)
    memcpy(&payload_out[offset], rx_dfe_cfg, 12);
    offset += 12;

    // RX_RANGING_CONTROL_CONFIGURATION_INDEX (1 byte)
    payload_out[offset++] = rx_range_ctrl_cfg_idx;

    // RX_RANGING_CONTROL_CONFIGURATION (16 bytes)
    memcpy(&payload_out[offset], rx_range_ctrl_cfg, 16);
    offset += 16;

    // RX_PAYLOAD_CONFIGURATION (4 bytes)
    memcpy(&payload_out[offset], rx_payload_cfg, 4);
    offset += 4;   

    return offset; // should be 95 bytes
}


bool build_command_rx_config(uint8_t *buffer_out, size_t *len_out,
    const uint8_t *sts_config,
    uint8_t rx_radio_cfg_idx,
    const uint8_t *rx_radio_cfg,
    uint8_t rx_dfe_cfg_idx,
    const uint8_t *rx_dfe_cfg,
    uint8_t rx_range_ctrl_cfg_idx,
    const uint8_t *rx_range_ctrl_cfg,
    const uint8_t *rx_payload_cfg)  {
    
    uint8_t payload[128];  // espacio suficiente
    size_t payload_len = build_payload_rx_config(payload, sts_config,
        rx_radio_cfg_idx, rx_radio_cfg,
        rx_dfe_cfg_idx, rx_dfe_cfg,
        rx_range_ctrl_cfg_idx, rx_range_ctrl_cfg,
        rx_payload_cfg);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF = 0
                        0x0A,        // GID = CORE
                        0x04,        // OID = RX_CONFIG
                        payload, payload_len);
}


static size_t build_payload_tx_config(uint8_t *payload_out,
    const uint8_t *sts_config,
    uint8_t tx_radio_cfg_idx, const uint8_t *tx_radio_cfg,
    uint8_t tx_power_cfg_idx, const uint8_t *tx_power_cfg,
    uint8_t tx_range_ctrl_cfg_idx, const uint8_t *tx_range_ctrl_cfg,
    const uint8_t *tx_payload_cfg, size_t tx_payload_cfg_len) {
    if (!payload_out || !sts_config || !tx_radio_cfg || !tx_power_cfg || !tx_range_ctrl_cfg || !tx_payload_cfg)
        return 0;

    size_t offset = 0;

    // STS_CONFIGURATION (56 bytes)
    memcpy(&payload_out[offset], sts_config, 56);
    offset += 56;

    // TX_RADIO_CONFIGURATION_INDEX (1 byte)
    payload_out[offset++] = tx_radio_cfg_idx;

    // TX_RADIO_CONFIGURATION (4 bytes)
    memcpy(&payload_out[offset], tx_radio_cfg, 4);
    offset += 4;

    // TX_POWER_CONFIGURATION_INDEX (1 byte)
    payload_out[offset++] = tx_power_cfg_idx;

    // TX_POWER_CONFIGURATION (4 bytes)
    memcpy(&payload_out[offset], tx_power_cfg, 4);
    offset += 4;

    // TX_RANGING_CONTROL_CONFIGURATION_INDEX (1 byte)
    payload_out[offset++] = tx_range_ctrl_cfg_idx;

    // TX_RANGING_CONTROL_CONFIGURATION (12 bytes)
    memcpy(&payload_out[offset], tx_range_ctrl_cfg, 12);
    offset += 12;

    // TX_PAYLOAD_CONFIGURATION (tamaño variable, típicamente 4 bytes)
    memcpy(&payload_out[offset], tx_payload_cfg, tx_payload_cfg_len);
    offset += tx_payload_cfg_len;

    return offset; // total = 83 bytes
}

bool build_command_tx_config(uint8_t *buffer_out, size_t *len_out,
    const uint8_t *sts_config,
    uint8_t tx_radio_cfg_idx, const uint8_t *tx_radio_cfg,
    uint8_t tx_power_cfg_idx, const uint8_t *tx_power_cfg,
    uint8_t tx_range_ctrl_cfg_idx, const uint8_t *tx_range_ctrl_cfg,
    const uint8_t *tx_payload_cfg, size_t tx_payload_cfg_len) {

    uint8_t payload[128];
    size_t payload_len = build_payload_tx_config(payload,
                                                 sts_config,
                                                 tx_radio_cfg_idx, tx_radio_cfg,
                                                 tx_power_cfg_idx, tx_power_cfg,
                                                 tx_range_ctrl_cfg_idx, tx_range_ctrl_cfg,
                                                 tx_payload_cfg, tx_payload_cfg_len);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF = 0
                        0x0A,        // GID = CORE
                        0x05,        // OID = TX_CONFIG
                        payload, payload_len);
}


static size_t build_payload_start_baseband(uint8_t *payload_out,
    const uint8_t *uwb_freq,   // 4 bytes
    const uint8_t *ppm,        // 2 bytes
    const uint8_t *init_delay, // 4 bytes
    uint8_t pll_mode,
    uint8_t cfg_idx,
    const uint8_t *cmd_cfg,    // 20 bytes
    const uint8_t *notif_sel,  // 4 bytes
    const uint8_t *start_idx,  // 2 bytes
    const uint8_t *end_idx,    // 2 bytes
    const uint8_t *cmd_seq_cnt // 4 bytes
    ){
    if (!payload_out || !uwb_freq || !ppm || !init_delay || !cmd_cfg || !notif_sel || !start_idx || !end_idx || !cmd_seq_cnt)
        return 0;

    size_t offset = 0;

    // UWB_CHANNEL_FREQUENCY (ejemplo: 0x00630600 = 6.4896 GHz)
    memcpy(&payload_out[offset], uwb_freq, 4);      
    offset += 4;

    // PPM (ejemplo: 0x0069 = +10.5 ppm)
    memcpy(&payload_out[offset], ppm, 2);           
    offset += 2;

    // INIT_DELAY (ejemplo: 0x000003E8 = 1000 us)
    memcpy(&payload_out[offset], init_delay, 4);    
    offset += 4;

    // RADAR_SYSPLL_MODE (0x00 = modo fraccional)
    payload_out[offset++] = pll_mode;
        
    // CONFIGURATION_INDEX (0xFF = no guardar)
    payload_out[offset++] = cfg_idx;

    // COMMAND_CONFIGURATION (20 bytes - relleno temporal)
    memcpy(&payload_out[offset], cmd_cfg, 20);      
    offset += 20;
   
    // NOTIFICATION_SEL (4 bytes)
    memcpy(&payload_out[offset], notif_sel, 4);     
    offset += 4;
    
    // START_INDEX (2 bytes)
    memcpy(&payload_out[offset], start_idx, 2);     
    offset += 2;

    // END_INDEX (2 bytes)
    memcpy(&payload_out[offset], end_idx, 2);       
    offset += 2;
    // CMD_SEQUENCE_COUNT (4 bytes - 0x00000001 → 1 loop)
    memcpy(&payload_out[offset], cmd_seq_cnt, 4);   
    offset += 4;
    
    return offset; // total = 44 bytes
}


bool build_command_start_baseband(uint8_t *buffer_out, size_t *len_out,
    const uint8_t *uwb_freq,
    const uint8_t *ppm,
    const uint8_t *init_delay,
    uint8_t pll_mode,
    uint8_t cfg_idx,
    const uint8_t *cmd_cfg,
    const uint8_t *notif_sel,
    const uint8_t *start_idx,
    const uint8_t *end_idx,
    const uint8_t *cmd_seq_cnt) {

    uint8_t payload[64];
    size_t payload_len = build_payload_start_baseband(payload, uwb_freq, ppm, init_delay,
        pll_mode, cfg_idx, cmd_cfg, notif_sel,
        start_idx, end_idx, cmd_seq_cnt);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x06,        // OID = START_BASEBAND
                        payload, payload_len);
}

bool build_command_stop_baseband(uint8_t *buffer_out, size_t *len_out) {
    // Payload vacío
    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x07,        // OID = STOP_BASEBAND
                        NULL, 0);
}

#include "radar_config.h"

static void write_u32_le(uint8_t *out, uint32_t val) {
    out[0] = (val >> 0) & 0xFF;
    out[1] = (val >> 8) & 0xFF;
    out[2] = (val >> 16) & 0xFF;
    out[3] = (val >> 24) & 0xFF;
}

static void write_i32_le(uint8_t *out, int32_t val) {
    write_u32_le(out, (uint32_t)val);
}

static void write_u16_le(uint8_t *out, uint16_t val) {
    out[0] = val & 0xFF;
    out[1] = (val >> 8) & 0xFF;
}


static void write_float_le(uint8_t *out, float val) {
    uint32_t bin;
    memcpy(&bin, &val, sizeof(float));  // convierte float en uint32_t binario
    write_u32_le(out, bin);
}

static size_t build_payload_generic_baseband_config(uint8_t *payload_out, uint8_t config_select){
    // , uint8_t config_index,
    // const uint8_t *config_data, size_t data_len) {

    if (!payload_out) return 0;

    size_t offset = 0;

    payload_out[offset++] = config_select;  // BASEBAND_CONFIGURATION_SELECT
    payload_out[offset++] = 0x00;           // CONFIGURATION_INDEX

    switch (config_select) {
        case CFG_RX_RADIO:
            write_u32_le(&payload_out[offset], radar_config.RX_RADIO_CONFIGURATION.RADIO_CONFIG_SOURCE);
            offset += 4;
            // payload_out[offset++] = 0x07;            // payload_out[offset++] = 0x00;
            // payload_out[offset++] = 0x00;            // payload_out[offset++] = 0x00;            
            break;

        case CFG_TX_RADIO:
            write_u32_le(&payload_out[offset], radar_config.TX_RADIO_CONFIGURATION.RADIO_CONFIG_SOURCE);
            offset += 4;
            // payload_out[offset++] = 0x07;            // payload_out[offset++] = 0x00;
            // payload_out[offset++] = 0x00;            // payload_out[offset++] = 0x00;
            break;

        case CFG_TX_POWER:   
            payload_out[offset++] = 0xCA; //(uint8_t) radar_config.TX_POWER_CONFIGURATION.TX_POWER_NOMINAL; //0xCA;
            payload_out[offset++] = (uint8_t) radar_config.TX_POWER_CONFIGURATION.TX_POWER_OFFSET; //0x00;
            write_u16_le(&payload_out[offset], radar_config.TX_POWER_CONFIGURATION.TX_POWER_BOOST); //0x0000;
            offset += 2;
            break;

        case CFG_RX_RADAR_CIR:
            write_u16_le(&payload_out[offset], radar_config.RX_RADAR_CIR_CONFIGURATION.CIR_TAPS); //0x0040;
            offset += 2;
            // payload_out[offset++] = 0x40;    // payload_out[offset++] = 0x00;
            write_u16_le(&payload_out[offset], radar_config.RX_RADAR_CIR_CONFIGURATION.RX_SYNC_SYMBOL_CNT);
            offset += 2;
            // payload_out[offset++] = 0x20;              // payload_out[offset++] = 0x00;
            write_u16_le(&payload_out[offset], radar_config.RX_RADAR_CIR_CONFIGURATION.CIR_OFFSET);
            offset += 2;
            payload_out[offset++] = radar_config.RX_RADAR_CIR_CONFIGURATION.SENSITIVITY_BOOST_OFFSET_RX1; //0x00;
            payload_out[offset++] = radar_config.RX_RADAR_CIR_CONFIGURATION.SENSITIVITY_BOOST_OFFSET_RX2; //0x02;
            write_i32_le(&payload_out[offset], radar_config.RX_RADAR_CIR_CONFIGURATION.CIR_THRESHOLD_NTF);
            offset += 4;
            payload_out[offset++] = radar_config.RX_RADAR_CIR_CONFIGURATION.TIMESTAMP_LOGGING_ENBL; //0x01;
            payload_out[offset++] = radar_config.RX_RADAR_CIR_CONFIGURATION.CIR_BUFFER_SIZE; //0x02;
            payload_out[offset++] = 0x05; // DATA_ACQ_MODE
            payload_out[offset++] = 0x00;
            break;

        case CFG_RX_RADAR_CTRL:
            payload_out[offset++] = 0x00; // RX_RADIO_CONFIGURATION_INDEX
            payload_out[offset++] = 0xFF; // SYNC_CODE_CONFIGURATION_INDEX
            payload_out[offset++] = 0xFF; // RX_DFE_CONFIGURATION_INDEX
            payload_out[offset++] = 0x00; // RX_RADAR_CIR_CONFIGURATION_INDEX
            payload_out[offset++] = radar_config.RX_RADAR_CONTROL_CONFIGURATION.RX_RADAR_CALIBRATION_SELECT; //0x00; // RX_RADAR_CALIBRATION_SELECT
            payload_out[offset++] = radar_config.RX_RADAR_CONTROL_CONFIGURATION.RNS_CONFIGURATION_INDEX; //0x00; // RNS_CONFIGURATION_INDEX
            payload_out[offset++] = radar_config.RX_RADAR_CONTROL_CONFIGURATION.RX_ENBL; //0x02; // RX_ENBL
            payload_out[offset++] = radar_config.RX_RADAR_CONTROL_CONFIGURATION.SENSITIVITY_BOOST; //0x02;
            break;

        case CFG_TX_RADAR_CTRL:
            payload_out[offset++] = 0x00; // TX_RADIO_CONFIGURATION_INDEX
            payload_out[offset++] = 0xFF; // SYNC_CODE_CONFIGURATION_INDEX
            payload_out[offset++] = 0x00;
            payload_out[offset++] = radar_config.TX_RADAR_CONTROL_CONFIGURATION.TX_ENBL; //0x01; // TX_ENBL
            write_u16_le(&payload_out[offset], radar_config.TX_RADAR_CONTROL_CONFIGURATION.TX_SYNC_SYMBOL_CNT); //0x0200;
            offset += 2; // TX_SYNC_SYMBOL_CNT
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            break;

        case CFG_RADAR_NOISE_SUPP:
            // RNS_RX1
            write_u16_le(&payload_out[offset], radar_config.RADAR_NOISE_SUPPRESSION_CONFIGURATION.RNS_RX1.SELF_INTERFERENCE_TAP); //0x0C 0x00;
            offset += 2;
            write_u16_le(&payload_out[offset], 0x04); //0x04 0x00; // RNS_CONTROL
            offset += 2;    

            write_i32_le(&payload_out[offset], radar_config.RADAR_NOISE_SUPPRESSION_CONFIGURATION.RNS_RX1.NOISE_SUPPRESSION_INDEX_MASK); //0x00000FFF;
            // payload_out[offset++] = 0xFF;
            // payload_out[offset++] = 0x0F;
            // payload_out[offset++] = 0x00;
            // payload_out[offset++] = 0x00;
            offset += 4;
            
            payload_out[offset++] = 0x9B;
            payload_out[offset++] = 0x01;
            
            payload_out[offset++] = 0x00;

            // RFU:
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;

            // RNXS_RX2
            write_u16_le(&payload_out[offset], radar_config.RADAR_NOISE_SUPPRESSION_CONFIGURATION.RNS_RX2.SELF_INTERFERENCE_TAP); //0x0D 0x00;
            offset += 2;
            write_u16_le(&payload_out[offset], 0x04); //0x04 0x00; // RNS_CONTROL
            offset += 2; 

            payload_out[offset++] = 0xFF;
            payload_out[offset++] = 0x0F;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            
            payload_out[offset++] = 0x9B;
            payload_out[offset++] = 0x01;
            
            payload_out[offset++] = 0x01;
           
            // RFU:
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;
            payload_out[offset++] = 0x00;

            break;

        case CFG_RADAR_CALIB_DATA:
            payload_out[offset++] = 0x88;
            break;

        default:
            return 0;  // config_select inválido
    }

    return offset;

 
}


bool build_command_generic_baseband_config(uint8_t *buffer_out, size_t *len_out, uint8_t config_select) {
    if (!buffer_out || !len_out) return false;

    uint8_t payload[50];

    size_t payload_len = build_payload_generic_baseband_config(payload, config_select);

    if (payload_len == 0) return false;

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x0A,        // OID = CMD_GENERIC_BASEBAND_CONFIG
                        payload, payload_len);
}



static size_t build_payload_get_baseband_results(uint8_t *payload_out,uint8_t result_sel, uint8_t results_idx) {
    if (!payload_out) return 0;

    
    payload_out[0] = result_sel; // BASEBAND_RESULT_SEL: BASEBAND_STATUS (por ejemplo)
    payload_out[1] = results_idx;// BASEBAND_RESULTS_INDEX: índice 0

    return 2;
}


bool build_command_get_baseband_results(uint8_t *buffer_out, size_t *len_out,
    uint8_t result_sel, uint8_t results_idx) {
    uint8_t payload[4];
    size_t payload_len = build_payload_get_baseband_results(payload, result_sel, results_idx);
    if (payload_len != 2) return false;

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x08,        // OID = GET_BASEBAND_RESULTS
                        payload, payload_len);
}

/*
static size_t build_payload_power_mgmt_config(uint8_t *payload_out) {
    if (!payload_out) return 0;
    size_t offset = 0;

    // 1. TARGET_POWER_MODE
    payload_out[offset++] = 0x00;
s
    // 2. CPU_IDLE_MODE
    payload_out[offset++] = 0x00;

    // 3. BASEBAND_POWER_CONTROL
    payload_out[offset++] = 0x01;

    // 4. SNAPSHOT_CONTROL
    payload_out[offset++] = 0x00;

    // 5. CPU_MEM_CONTROL (4 bytes)
    payload_out[offset++] = 0x01;
    payload_out[offset++] = 0x01;
    payload_out[offset++] = 0x01;
    payload_out[offset++] = 0x01;

    // 6. DSP_MEM_CONTROL_RANGING (4 bytes)
    payload_out[offset++] = 0x02;
    payload_out[offset++] = 0x02;
    payload_out[offset++] = 0x02;
    payload_out[offset++] = 0x02;

    // 7. DSP_MEM_CONTROL_RADAR (4 bytes)
    payload_out[offset++] = 0x01;
    payload_out[offset++] = 0x01;
    payload_out[offset++] = 0x01;
    payload_out[offset++] = 0x01;

    // 8. LOW_POWER_GPIO_CONTROL (8 bytes)
    memset(&payload_out[offset], 0x00, 8);
    offset += 8;

    // 9. LOW_POWER_LDO_CONTROLS (4 bytes)
    payload_out[offset++] = 0x05;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;

    // 10. DPD_WAKEUPTIMER_CONTROL (8 bytes)
    memset(&payload_out[offset], 0x00, 8);
    offset += 8;

    return offset; // should be 36
}

bool build_command_power_mgmt_config(uint8_t *buffer_out, size_t *len_out) {
    uint8_t payload[64];
    size_t payload_len = build_payload_power_mgmt_config(payload);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x09,        // OID = POWER_MGMT_CONFIG
                        payload, payload_len);
}


static size_t build_payload_ranging_application_config(uint8_t *payload_out) {
    if (!payload_out) return 0;
    size_t offset = 0;

    // UWB_CHANNEL_FREQUENCY (0x00630600 → 6.4896 GHz)
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x06;
    payload_out[offset++] = 0x63;
    payload_out[offset++] = 0x00;

    // PPM (ej. +10.5 ppm = 105 = 0x0069)
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x69;

    // KEEP_SYSPLL
    payload_out[offset++] = 0x01;

    // NOTIFICATION_SEL
    payload_out[offset++] = 0x01;

    // RANGING_REPEAT_CNT = 1
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x01;

    // RANGING_REPEAT_INTERVAL = 1000 µs (0x000003E8)
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x03;
    payload_out[offset++] = 0xE8;

    // RX_START_MARGIN = 100 µs (0x00000064)
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x00;
    payload_out[offset++] = 0x64;

    return offset; // Debe ser 20 bytes
}

bool build_command_ranging_application_config(uint8_t *buffer_out, size_t *len_out) {
    uint8_t payload[32];
    size_t payload_len = build_payload_ranging_application_config(payload);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x21,        // OID = RANGING_APPLICATION_CONFIG
                        payload, payload_len);
}

bool build_command_start_ranging(uint8_t *buffer_out, size_t *len_out) {
    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x23,        // OID = START_RANGING
                        NULL, 0);    // Payload vacío
}


static size_t build_payload_get_ranging_payload(uint8_t *payload_out) {
    if (!payload_out) return 0;

    payload_out[0] = 0x00;  // Por defecto: índice 0
    return 1;
}

bool build_command_get_ranging_payload(uint8_t *buffer_out, size_t *len_out) {
    uint8_t payload[1];
    size_t payload_len = build_payload_get_ranging_payload(payload);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x24,        // OID = GET_RANGING_PAYLOAD
                        payload, payload_len);
}
*/
static size_t build_payload_radar_application_config(uint8_t *payload_out,
                                        const uint8_t *uwb_freq,         // 4 bytes
                                        const uint8_t *ppm,              // 2 bytes
                                        uint8_t keep_syspll,             // 1 byte
                                        uint8_t pll_mode,                // 1 byte
                                        const uint8_t *init_delay,       // 4 bytes
                                        uint8_t radar_mode,              // 1 byte
                                        uint8_t rx_cfg_idx,              // 1 byte
                                        uint8_t tx_cfg_idx,              // 1 byte
                                        uint8_t notif_enbl,              // 1 byte
                                        const uint8_t *frame_cnt,        // 4 bytes
                                        const uint8_t *frame_interval,   // 4 bytes
                                        const uint8_t *burst_cnt,        // 4 bytes
                                        const uint8_t *burst_delay) 
{
    if (!payload_out) return 0;
    size_t i = 0;

    // UWB_CHANNEL_FREQUENCY (0x00630600)
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x06;
    // payload_out[i++] = 0x63;
    // payload_out[i++] = 0x00;
    memcpy(&payload_out[i], uwb_freq, 4); i += 4;

    // PPM (10.5 ppm → 0x0069)
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x69;
    memcpy(&payload_out[i], ppm, 2); i += 2;

    // KEEP_SYSPLL
    // payload_out[i++] = 0x01;
    payload_out[i++] = keep_syspll;

    // RADAR_SYSPLL_MODE
    // payload_out[i++] = 0x00;
    payload_out[i++] = pll_mode;

    // INIT_DELAY (0x000003E8 = 1000 µs)
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x03;
    // payload_out[i++] = 0xE8;
    memcpy(&payload_out[i], init_delay, 4); i += 4;

    // MODE (0x00 = TRX mode)
    // payload_out[i++] = 0x00;
    payload_out[i++] = radar_mode;

    // RX_RADAR_CONTROL_CONFIGURATION_IDX
    // payload_out[i++] = 0x00;
    payload_out[i++] = rx_cfg_idx;

    // TX_RADAR_CONTROL_CONFIGURATION_IDX
    // payload_out[i++] = 0x00;
    payload_out[i++] = tx_cfg_idx;

    // NOTIFICATION_ENBL (0x03 = RX/TX notif. habilitadas)
    // payload_out[i++] = 0x03;
    payload_out[i++] = notif_enbl;

    // FRAME_CNT = 5
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x05;
    memcpy(&payload_out[i], frame_cnt, 4); i += 4;
    
    // FRAME_INTERVAL = 250 µs (0x000079E0)
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x79;
    // payload_out[i++] = 0xE0;
    memcpy(&payload_out[i], frame_interval, 4); i += 4;
    
    // BURST_CNT = 3
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x03;
    memcpy(&payload_out[i], burst_cnt, 4); i += 4;
    
    // BURST_DELAY = 500 µs (0x0000C350)
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0x00;
    // payload_out[i++] = 0xC3;
    // payload_out[i++] = 0x50;
    memcpy(&payload_out[i], burst_delay, 4); i += 4;
   

    return i; // 36 bytes
}

bool build_command_radar_application_config(uint8_t *buffer_out, size_t *len_out,
                                            const uint8_t *uwb_freq,
                                            const uint8_t *ppm,
                                            uint8_t keep_syspll,
                                            uint8_t pll_mode,
                                            const uint8_t *init_delay,
                                            uint8_t radar_mode,
                                            uint8_t rx_cfg_idx,
                                            uint8_t tx_cfg_idx,
                                            uint8_t notif_enbl,
                                            const uint8_t *frame_cnt,
                                            const uint8_t *frame_interval,
                                            const uint8_t *burst_cnt,
                                            const uint8_t *burst_delay)
{
    uint8_t payload[64];
    size_t payload_len = build_payload_radar_application_config(payload,
                                            uwb_freq, ppm, keep_syspll, pll_mode,
                                            init_delay, radar_mode, rx_cfg_idx, tx_cfg_idx,
                                            notif_enbl, frame_cnt, frame_interval,
                                            burst_cnt, burst_delay);

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x30,        // OID = RADAR_APPLICATION_CONFIG
                        payload, payload_len);
}


bool build_command_start_radar(uint8_t *buffer_out, size_t *len_out) {
    printf("\n\t[BUILD] Construyendo comando CMD_START_RADAR...\n");
    // No payload
    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF = 0
                        0x0A,        // GID = CORE
                        0x31,        // OID = START_RADAR
                        NULL, 0);    // Payload vacío
}


static size_t build_payload_set_bitfield(uint8_t *payload_out, uint32_t address, uint8_t offset, uint8_t width, uint32_t value) {
    if (!payload_out || offset + width > 32) return 0;

    // ADDRESS (4 bytes, little endian)
    payload_out[0] = (uint8_t)(address & 0xFF);
    payload_out[1] = (uint8_t)((address >> 8) & 0xFF);
    payload_out[2] = (uint8_t)((address >> 16) & 0xFF);
    payload_out[3] = (uint8_t)((address >> 24) & 0xFF);

    // OFFSET
    payload_out[4] = offset;

    // WIDTH
    payload_out[5] = width;

    // VALUE (4 bytes, little endian)
    payload_out[6] = (uint8_t)(value & 0xFF);
    payload_out[7] = (uint8_t)((value >> 8) & 0xFF);
    payload_out[8] = (uint8_t)((value >> 16) & 0xFF);
    payload_out[9] = (uint8_t)((value >> 24) & 0xFF);

    return 10;
}

bool build_command_set_bitfield(uint8_t *buffer_out, size_t *len_out,
                                 uint32_t address, uint8_t offset, uint8_t width, uint32_t value) {
    printf("\n\t[BUILD] Construyendo comando CMD_SET_BITFIELD...\n");

    uint8_t payload[10];
    size_t payload_len = build_payload_set_bitfield(payload, address, offset, width, value);
    if (payload_len != 10) return false;

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x13,        // OID = SET_BITFIELD
                        payload, payload_len);
}


static size_t build_payload_get_bitfield(uint8_t *payload_out, uint32_t address, uint8_t offset, uint8_t width) {
    if (!payload_out || offset + width > 32) return 0;

    // ADDRESS (4 bytes, little endian)
    payload_out[0] = (uint8_t)(address & 0xFF);
    payload_out[1] = (uint8_t)((address >> 8) & 0xFF);
    payload_out[2] = (uint8_t)((address >> 16) & 0xFF);
    payload_out[3] = (uint8_t)((address >> 24) & 0xFF);

    // OFFSET
    payload_out[4] = offset;

    // WIDTH
    payload_out[5] = width;

    return 6;
}

bool build_command_get_bitfield(uint8_t *buffer_out, size_t *len_out,
                                 uint32_t address, uint8_t offset, uint8_t width) {
    uint8_t payload[6];
    size_t payload_len = build_payload_get_bitfield(payload, address, offset, width);
    if (payload_len != 6) return false;

    return build_packet(buffer_out, len_out,
                        0x01,        // MT = CMD
                        0x00,        // PBF
                        0x0A,        // GID = CORE
                        0x14,        // OID = GET_BITFIELD
                        payload, payload_len);
}

