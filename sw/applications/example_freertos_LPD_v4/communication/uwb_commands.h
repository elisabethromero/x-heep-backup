#ifndef UWB_COMMANDS_H
#define UWB_COMMANDS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// ========================
// CRC
// ========================
#define CRC16_POLY 0x1021
#define CRC16_INIT 0x0000

// Función para calcular CRC16/XMODEM
uint16_t crc16_xmodem(const uint8_t *data, size_t length);

// ========================
// Packet Builder
// ========================
// Función genérica de construcción de paquetes
bool build_packet(uint8_t *buffer_out, size_t *len_out,
                  uint8_t mt, uint8_t pbf,
                  uint8_t gid, uint8_t oid,
                  const uint8_t *payload, size_t payload_len);

// ========================
// Comandos
// ========================
// Comandos implementados
bool build_command_reset_device(uint8_t *buffer_out, size_t *len_out, uint8_t reset_type);


bool build_command_get_fw_version(uint8_t *buffer_out, size_t *len_out);


bool build_command_rx_config(uint8_t *buffer_out, size_t *len_out,
    const uint8_t *sts_config,
    uint8_t rx_radio_cfg_idx,
    const uint8_t *rx_radio_cfg,
    uint8_t rx_dfe_cfg_idx,
    const uint8_t *rx_dfe_cfg,
    uint8_t rx_range_ctrl_cfg_idx,
    const uint8_t *rx_range_ctrl_cfg,
    const uint8_t *rx_payload_cfg);

bool build_command_tx_config(uint8_t *buffer_out, size_t *len_out,
    const uint8_t *sts_config,
    uint8_t tx_radio_cfg_idx, const uint8_t *tx_radio_cfg,
    uint8_t tx_power_cfg_idx, const uint8_t *tx_power_cfg,
    uint8_t tx_range_ctrl_cfg_idx, const uint8_t *tx_range_ctrl_cfg,
    const uint8_t *tx_payload_cfg, size_t tx_payload_cfg_len);


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
    const uint8_t *cmd_seq_cnt) ;

bool build_command_stop_baseband(uint8_t *buffer_out, size_t *len_out);


#define CFG_RX_RADIO               0x00
#define CFG_TX_RADIO               0x10
#define CFG_TX_POWER               0x11
#define CFG_RX_RADAR_CIR           0x30
#define CFG_RX_RADAR_CTRL          0x31
#define CFG_TX_RADAR_CTRL          0x32
#define CFG_RADAR_NOISE_SUPP       0x34
#define CFG_RADAR_CALIB_DATA       0x33

bool build_command_generic_baseband_config(uint8_t *buffer_out, size_t *len_out, uint8_t config_select);


#define ACC_GAIN_RESULTS           0x13
#define CALIB_GAIN_RESULTS         0x14
#define CALIB_DATA                 0x15

#define RX1                        0x00
#define RX2                        0x01

bool build_command_get_baseband_results(uint8_t *buffer_out, size_t *len_out,
    uint8_t result_sel, uint8_t results_idx) ;
/*
bool build_command_power_mgmt_config(uint8_t *buffer_out, size_t *len_out,
                                      const uint8_t *payload_36bytes);

bool build_command_ranging_application_config(uint8_t *buffer_out, size_t *len_out) ;

bool build_command_start_ranging(uint8_t *buffer_out, size_t *len_out);

bool build_command_get_ranging_payload(uint8_t *buffer_out, size_t *len_out,
                                       uint8_t payload_index);
*/




bool build_command_get_bitfield(uint8_t *buffer_out, size_t *len_out,
                                uint32_t address, uint8_t offset, uint8_t width);

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
                                                    const uint8_t *burst_delay);

bool build_command_set_bitfield(uint8_t *buffer_out, size_t *len_out,
                                uint32_t address, uint8_t offset, uint8_t width, uint32_t value);

bool build_command_start_radar(uint8_t *buffer_out, size_t *len_out);

#endif // UWB_COMMANDS_H
