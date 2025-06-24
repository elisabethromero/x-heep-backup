#include <string.h>
#include <stdlib.h>
#define JSMN_PARENT_LINKS // AGREGADO
#include "radar_config_json.h"
#include "jsmn.h"
#include <stdio.h>
#include <unistd.h>
//#include <dirent.h>

#define CONFIG_FILE_PATH "/radar_config.json"

static radar_config_t config_instance;

const radar_config_t* radar_config_get(void) {
    return &config_instance;
}

// Función auxiliar para comparar claves
int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING &&
        (int)strlen(s) == (tok->end - tok->start) &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

// Función para analizar la configuración del radar desde un JSON
// Esta función utiliza la biblioteca jsmn para analizar el JSON y llenar la estructura radar_config_t
int parse_radar_config(const char *json, radar_config_t *cfg){
    static jsmntok_t tokens[256]; //Memoria estática, no ocupa stack
    
    jsmn_parser parser;
    jsmn_init(&parser);
    
    int tok_count = jsmn_parse(&parser, json, strlen(json), tokens, 256);
    if (tok_count < 0) {
        printf("[ERROR] Error parseando JSON: %d\n", tok_count);
        return -1;
    }

    printf("[DEBUG] Tokens parseados: %d\n", tok_count);

    memset(cfg, 0, sizeof(radar_config_t));
    
    for (int i = 1; i < tok_count; i++) {
        if (jsoneq(json, &tokens[i], "device_index") == 0) {
            cfg->device_index = atoi(json + tokens[i + 1].start);
            i++;
        }
        
        // TX_RADIO_CONFIGURATION
        else if (jsoneq(json, &tokens[i], "TX_RADIO_CONFIGURATION") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            for (int k = 0; k < n; k++) {
                if (jsoneq(json, &tokens[j], "RADIO_CONFIG_SOURCE") == 0) {
                    cfg->tx_radio_config_source = atoi(json + tokens[j + 1].start);
                }
                j += 2;
            }
            i = j - 1;
        }

        // RX_RADIO_CONFIGURATION
        else if (jsoneq(json, &tokens[i], "RX_RADIO_CONFIGURATION") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            for (int k = 0; k < n; k++) {
                if (jsoneq(json, &tokens[j], "RADIO_CONFIG_SOURCE") == 0) {
                    cfg->rx_radio_config_source = atoi(json + tokens[j + 1].start);
                }
                j += 2;
            }
            i = j - 1;
        }
        
        // CMD_RADAR_APPLICATION_CONFIG
        else if (jsoneq(json, &tokens[i], "CMD_RADAR_APPLICATION_CONFIG") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "UWB_CHANNEL_FREQUENCY") == 0)
                    cfg->uwb_channel_frequency = atoi(val);
                else if (jsoneq(json, &tokens[j], "PPM") == 0)
                    cfg->ppm = atoi(val);
                else if (jsoneq(json, &tokens[j], "KEEP_SYSPLL") == 0)
                    cfg->keep_syspll = atoi(val);
                else if (jsoneq(json, &tokens[j], "RADAR_SYSPLL_MODE") == 0)
                    cfg->radar_syspll_mode = atoi(val);
                else if (jsoneq(json, &tokens[j], "INIT_DELAY") == 0)
                    cfg->init_delay = atoi(val);
                else if (jsoneq(json, &tokens[j], "MODE") == 0)
                    cfg->mode = atoi(val);
                else if (jsoneq(json, &tokens[j], "RX_RADAR_CONTROL_CONFIGURATION_IDX") == 0)
                    cfg->rx_cfg_idx = atoi(val);
                else if (jsoneq(json, &tokens[j], "TX_RADAR_CONTROL_CONFIGURATION_IDX") == 0)
                    cfg->tx_cfg_idx = atoi(val);
                else if (jsoneq(json, &tokens[j], "NOTIFICATION_ENBL") == 0)
                    cfg->notification_enbl = atoi(val);
                else if (jsoneq(json, &tokens[j], "FRAME_CNT") == 0)
                    cfg->frame_cnt = atoi(val);
                else if (jsoneq(json, &tokens[j], "FRAME_INTERVAL") == 0)
                    cfg->frame_interval = atoi(val);
                else if (jsoneq(json, &tokens[j], "BURST_CNT") == 0)
                    cfg->burst_cnt = atoi(val);
                else if (jsoneq(json, &tokens[j], "BURST_DELAY") == 0)
                    cfg->burst_delay = atoi(val);
                j += 2;
            }
            i = j - 1;
        }


        // TX_POWER_CONFIGURATION
        else if (jsoneq(json, &tokens[i], "TX_POWER_CONFIGURATION") == 0) {
            int j = i + 1;

            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for(int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "TX_POWER_NOMINAL") == 0)
                    cfg->tx_power_nominal = atoi(val);
                else if (jsoneq(json, &tokens[j], "TX_POWER_OFFSET") == 0)
                    cfg->tx_power_offset = atoi(val);
                else if (jsoneq(json, &tokens[j], "TX_POWER_BOOST") == 0)
                    cfg->tx_power_boost = atoi(val);
                j += 2;
            }
            i = j - 1;
        }

        // RX_RADAR_CIR_CONFIGURATION
        else if (jsoneq(json, &tokens[i], "RX_RADAR_CIR_CONFIGURATION") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "CIR_TAPS") == 0)
                    cfg->cir_taps = atoi(val);
                else if (jsoneq(json, &tokens[j], "RX_SYNC_SYMBOL_CNT") == 0)
                    cfg->rx_sync_symbol_cnt = atoi(val);
                else if (jsoneq(json, &tokens[j], "CIR_OFFSET") == 0)
                    cfg->cir_offset = atoi(val);
                else if (jsoneq(json, &tokens[j], "CIR_THRESHOLD_NTF") == 0)
                    cfg->cir_threshold_ntf = atoi(val);
                else if (jsoneq(json, &tokens[j], "TIMESTAMP_LOGGING_ENBL") == 0)
                    cfg->timestamp_logging_enbl = atoi(val);
                else if (jsoneq(json, &tokens[j], "CIR_BUFFER_SIZE") == 0)
                    cfg->cir_buffer_size = atoi(val);
                j += 2;
            }
            i = j - 1;
        }

        // DATA_ACQ_MODE
        else if (jsoneq(json, &tokens[i], "DATA_ACQ_MODE") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "Analog coupling") == 0)
                    cfg->analog_coupling = atoi(val);
                else if (jsoneq(json, &tokens[j], "Radar data selection") == 0)
                    cfg->radar_data_selection = atoi(val);
                else if (jsoneq(json, &tokens[j], "AGC control for Radar fast calibration") == 0)
                    cfg->agc_control = atoi(val);
                j += 2;
            }
            i = j - 1;
        }

        // TX_RADAR_CONTROL_CONFIGURATION
        else if (jsoneq(json, &tokens[i], "TX_RADAR_CONTROL_CONFIGURATION") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "TX_ENBL") == 0)
                    cfg->tx_enbl = atoi(val);
                else if (jsoneq(json, &tokens[j], "TX_SYNC_SYMBOL_CNT") == 0)
                    cfg->tx_sync_symbol_cnt = atoi(val);
                j += 2;
            }
            i = j - 1;
        }

        // RX_RADAR_CONTROL_CONFIGURATION
        else if (jsoneq(json, &tokens[i], "RX_RADAR_CONTROL_CONFIGURATION") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "RX_RADAR_CALIBRATION_SELECT") == 0)
                    cfg->rx_radar_calibration_select = atoi(val);
                else if (jsoneq(json, &tokens[j], "RNS_CONFIGURATION_INDEX") == 0)
                    cfg->rns_configuration_index = atoi(val);
                else if (jsoneq(json, &tokens[j], "RX_ENBL") == 0)
                    cfg->rx_enbl = atoi(val);
                else if (jsoneq(json, &tokens[j], "SENSITIVITY_BOOST") == 0)
                    cfg->sensitivity_boost = atoi(val);
                j += 2;
            }
            i = j - 1;
        }

        // RADAR_NOISE_SUPPRESSION_CONFIGURATION - RNS_RX1
        else if (jsoneq(json, &tokens[i], "RNS_RX1") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "SELF_INTERFERENCE_TAP") == 0)
                    cfg->rns_rx1_self_interference_tap = atoi(val);
                else if (jsoneq(json, &tokens[j], "RNS_CONTROL") == 0) {
                    int l = j + 1;
                    int m = tokens[j + 1].size;  // número de pares clave-valor dentro del objeto
                    l++;  // avanzar al primer par
                    for (int p = 0; p < m; p++) {
                        const char *val2 = json + tokens[l + 1].start;
                        if (jsoneq(json, &tokens[l], "Radar noise suppression") == 0)
                            cfg->rns_rx1_radar_noise_suppression = atoi(val2);
                        else if (jsoneq(json, &tokens[l], "Radar drift suppression") == 0)
                            cfg->rns_rx1_radar_drift_suppression = atoi(val2);
                        else if (jsoneq(json, &tokens[l], "DC removal") == 0)
                            cfg->rns_rx1_dc_removal = atoi(val2);
                        l += 2;
                    }
                    j = l - 1;
                }
                else if (jsoneq(json, &tokens[j], "NOISE_SUPPRESSION_INDEX_MASK") == 0) {
                    strncpy(cfg->rns_rx1_index_mask, json + tokens[j + 1].start, sizeof(cfg->rns_rx1_index_mask) - 1);
                    cfg->rns_rx1_index_mask[sizeof(cfg->rns_rx1_index_mask) - 1] = '\0'; // Asegurar terminación n
                }
                else if (jsoneq(json, &tokens[j], "DC_FILTER_CUTOFF_HZ") == 0)
                    cfg->rns_rx1_dc_filter_cutoff_hz = atof(val);
                else if (jsoneq(json, &tokens[j], "RNS_CALIBRATION_SLOT_INDEX") == 0)
                    cfg->rns_rx1_calibration_slot_index = atoi(val);
                j += 2;
            }
            i = j - 1;
        }
        // RADAR_NOISE_SUPPRESSION_CONFIGURATION - RNS_RX2
        else if (jsoneq(json, &tokens[i], "RNS_RX2") == 0) {
            int j = i + 1;
            int n = tokens[i + 1].size;  // número de pares clave-valor dentro del objeto
            j++;  // avanzar al primer par
            // Recorrer los pares clave-valor dentro del objeto
            int k = 0;
            for (int k = 0; k < n; k++) {
                const char *val = json + tokens[j + 1].start;
                if (jsoneq(json, &tokens[j], "SELF_INTERFERENCE_TAP") == 0)
                    cfg->rns_rx2_self_interference_tap = atoi(val);
                else if (jsoneq(json, &tokens[j], "RNS_CONTROL") == 0) {
                    int l = j + 1;
                    int m = tokens[j + 1].size;  // número de pares clave-valor dentro del objeto
                    l++;  // avanzar al primer par
                    for (int p = 0; p < m; p++) {
                        const char *val2 = json + tokens[l + 1].start;
                        if (jsoneq(json, &tokens[l], "Radar noise suppression") == 0)
                            cfg->rns_rx2_radar_noise_suppression = atoi(val2);
                        else if (jsoneq(json, &tokens[l], "Radar drift suppression") == 0)
                            cfg->rns_rx2_radar_drift_suppression = atoi(val2);
                        else if (jsoneq(json, &tokens[l], "DC removal") == 0)
                            cfg->rns_rx2_dc_removal = atoi(val2);
                        l += 2;
                    }
                    j = l - 1;
                }
                else if (jsoneq(json, &tokens[j], "NOISE_SUPPRESSION_INDEX_MASK") == 0) {
                    strncpy(cfg->rns_rx2_index_mask, json + tokens[j + 1].start, sizeof(cfg->rns_rx2_index_mask) - 1);
                    cfg->rns_rx2_index_mask[sizeof(cfg->rns_rx2_index_mask) - 1] = '\0'; // Asegurar terminación n
                }
                else if (jsoneq(json, &tokens[j], "DC_FILTER_CUTOFF_HZ") == 0)
                    cfg->rns_rx2_dc_filter_cutoff_hz = atof(val);
                else if (jsoneq(json, &tokens[j], "RNS_CALIBRATION_SLOT_INDEX") == 0)
                    cfg->rns_rx2_calibration_slot_index = atoi(val);
                j += 2;
            }
            i = j - 1;
        }

        // duration_sec
        else if (jsoneq(json, &tokens[i], "duration_sec") == 0) {
            cfg->duration_sec = atoi(json + tokens[i + 1].start);
            i++;
        }
    }

    return 0;
}


void print_config_radar(const radar_config_t *cfg) {
    // Mostrar configuración
    printf("[INFO] Configuración del radar:\n");

    printf("\tDevice Index: %d\n", cfg->device_index);

    printf("\tTX Radio Configuration:\n");
    printf("\t  TX Radio Config Source: %d\n", cfg->tx_radio_config_source);

    printf("\tRX Radio Configuration:\n");
    printf("\t  RX Radio Config Source: %d\n", cfg->rx_radio_config_source);

    printf("\tCMD_RADAR_APPLICATION_CONFIG:\n");
    printf("\t  UWB Channel Frequency: %d\n", cfg->uwb_channel_frequency);
    printf("\t  PPM: %d\n", cfg->ppm);
    printf("\t  Keep SysPLL: %d\n", cfg->keep_syspll);
    printf("\t  Radar SysPLL Mode: %d\n", cfg->radar_syspll_mode);
    printf("\t  Init Delay: %d\n", cfg->init_delay);
    printf("\t  Mode: %d\n", cfg->mode);
    printf("\t  RX Config Index: %d\n", cfg->rx_cfg_idx);
    printf("\t  TX Config Index: %d\n", cfg->tx_cfg_idx);
    printf("\t  Notification Enable: %d\n", cfg->notification_enbl);
    printf("\t  Frame Count: %d\n", cfg->frame_cnt);
    printf("\t  Frame Interval: %d\n", cfg->frame_interval);
    printf("\t  Burst Count: %d\n", cfg->burst_cnt);
    printf("\t  Burst Delay: %d\n", cfg->burst_delay);
    
    printf("\tTX Power Configuration:\n");
    printf("\t  TX Power Nominal: %d\n", cfg->tx_power_nominal);
    printf("\t  TX Power Offset: %d\n", cfg->tx_power_offset);
    printf("\t  TX Power Boost: %d\n", cfg->tx_power_boost);
    
    printf("\tRX Radar CIR Configuration:\n");
    printf("\t  CIR Taps: %d\n", cfg->cir_taps);
    printf("\t  RX Sync Symbol Count: %d\n", cfg->rx_sync_symbol_cnt);
    printf("\t  CIR Offset: %d\n", cfg->cir_offset);
    printf("\t  Sensitivity Boost RX1: %d\n", cfg->sensitivity_boost_offset_rx1);
    printf("\t  Sensitivity Boost RX2: %d\n", cfg->sensitivity_boost_offset_rx2);
    printf("\t  CIR Threshold NTF: %d\n", cfg->cir_threshold_ntf);
    printf("\t  Timestamp Logging Enable: %d\n", cfg->timestamp_logging_enbl);
    printf("\t  CIR Buffer Size: %d\n", cfg->cir_buffer_size);

    printf("\tData Acquisition Mode:\n");
    printf("\t    Analog Coupling: %d\n", cfg->analog_coupling);
    printf("\t    Radar Data Selection: %d\n", cfg->radar_data_selection);
    printf("\t    AGC Control: %d\n", cfg->agc_control);

    printf("\tTX Radar Control Configuration:\n");
    printf("\t    TX Enable: %d\n", cfg->tx_enbl);
    printf("\t    TX Sync Symbol Count: %d\n", cfg->tx_sync_symbol_cnt);

    printf("\tRX Radar Control Configuration:\n");
    printf("\t    RX Radar Calibration Select: %d\n", cfg->rx_radar_calibration_select);
    printf("\t    RNS Configuration Index: %d\n", cfg->rns_configuration_index);
    printf("\t    RX Enable: %d\n", cfg->rx_enbl);
    printf("\t    Sensitivity Boost: %d\n", cfg->sensitivity_boost);

    printf("\tRadar Noise Suppression - RNS_RX1:\n");
    printf("\t    Self Interference Tap: %d\n", cfg->rns_rx1_self_interference_tap);
    printf("\t    Radar Noise Suppression: %d\n", cfg->rns_rx1_radar_noise_suppression);
    printf("\t    Radar Drift Suppression: %d\n", cfg->rns_rx1_radar_drift_suppression);
    printf("\t    DC Removal: %d\n", cfg->rns_rx1_dc_removal);
    printf("\t    Index Mask: %s\n", cfg->rns_rx1_index_mask);
    printf("\t    DC Filter Cutoff (Hz): %.2f\n", cfg->rns_rx1_dc_filter_cutoff_hz);
    printf("\t    Calibration Slot Index: %d\n", cfg->rns_rx1_calibration_slot_index);

    printf("\tRadar Noise Suppression - RNS_RX2:\n");
    printf("\t    Self Interference Tap: %d\n", cfg->rns_rx2_self_interference_tap);
    printf("\t    Radar Noise Suppression: %d\n", cfg->rns_rx2_radar_noise_suppression);
    printf("\t    Radar Drift Suppression: %d\n", cfg->rns_rx2_radar_drift_suppression);
    printf("\t    DC Removal: %d\n", cfg->rns_rx2_dc_removal);
    printf("\t    Index Mask: %s\n", cfg->rns_rx2_index_mask);
    printf("\t    DC Filter Cutoff (Hz): %.2f\n", cfg->rns_rx2_dc_filter_cutoff_hz);
    printf("\t    Calibration Slot Index: %d\n", cfg->rns_rx2_calibration_slot_index);

    printf("\tDuration (sec): %d\n", cfg->duration_sec);
    printf("\n");

}