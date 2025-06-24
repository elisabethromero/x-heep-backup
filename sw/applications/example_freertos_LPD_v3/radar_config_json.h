#ifndef RADAR_CONFIG_JSON_H
#define RADAR_CONFIG_JSON_H




typedef struct {
    // Configuración general
    int device_index;

    // TX/RX radio config
    int tx_radio_config_source;
    int rx_radio_config_source;

    // CMD_RADAR_APPLICATION_CONFIG
    int uwb_channel_frequency;
    int ppm;
    int keep_syspll;
    int radar_syspll_mode;
    int init_delay;
    int mode;
    int rx_cfg_idx;
    int tx_cfg_idx;
    int notification_enbl;
    int frame_cnt;
    int frame_interval;
    int burst_cnt;
    int burst_delay;

    // TX_POWER_CONFIGURATION
    int tx_power_nominal;
    int tx_power_offset;
    int tx_power_boost;

    // RX_RADAR_CIR_CONFIGURATION
    int cir_taps;
    int rx_sync_symbol_cnt;
    int cir_offset;
    int sensitivity_boost_offset_rx1;
    int sensitivity_boost_offset_rx2;
    int cir_threshold_ntf;
    int timestamp_logging_enbl;
    int cir_buffer_size;

    // DATA_ACQ_MODE
    int analog_coupling;
    int radar_data_selection;
    int agc_control;
    
    // TX_RADAR_CONTROL_CONFIGURATION
    int tx_enbl;
    int tx_sync_symbol_cnt;

    // RX_RADAR_CONTROL_CONFIGURATION
    int rx_radar_calibration_select;
    int rns_configuration_index;
    int rx_enbl;
    int sensitivity_boost;

    // RADAR_NOISE_SUPPRESSION_CONFIGURATION - RNS_RX1
    int rns_rx1_self_interference_tap;
    int rns_rx1_radar_noise_suppression;
    int rns_rx1_radar_drift_suppression;
    int rns_rx1_dc_removal;
    char rns_rx1_index_mask[64];
    float rns_rx1_dc_filter_cutoff_hz;
    int rns_rx1_calibration_slot_index;

    // RADAR_NOISE_SUPPRESSION_CONFIGURATION - RNS_RX2
    int rns_rx2_self_interference_tap;
    int rns_rx2_radar_noise_suppression;
    int rns_rx2_radar_drift_suppression;
    int rns_rx2_dc_removal;
    char rns_rx2_index_mask[64];
    float rns_rx2_dc_filter_cutoff_hz;
    int rns_rx2_calibration_slot_index;

    // Duración
    int duration_sec;

} radar_config_t;


// Función para analizar la configuración del radar desde un JSON
int parse_radar_config(const char *json, radar_config_t *cfg);

// Accede al struct ya cargado
const radar_config_t* radar_config_get(void);

void print_config_radar(const radar_config_t *cfg);

void load_radar_config_from_file();

#endif
