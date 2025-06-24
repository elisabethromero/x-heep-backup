#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#include <stdint.h>

typedef struct {
    uint8_t RADIO_CONFIG_SOURCE;
} TX_RADIO_CONFIGURATION_t;

typedef struct {
    uint8_t RADIO_CONFIG_SOURCE;
} RX_RADIO_CONFIGURATION_t;

typedef struct {
    uint32_t UWB_CHANNEL_FREQUENCY;
    uint8_t PPM;
    uint8_t KEEP_SYSPLL;
    uint8_t RADAR_SYSPLL_MODE;
    uint32_t INIT_DELAY;
    uint8_t MODE;
    uint8_t RX_RADAR_CONTROL_CONFIGURATION_IDX;
    uint8_t TX_RADAR_CONTROL_CONFIGURATION_IDX;
    uint8_t NOTIFICATION_ENBL;
    uint32_t FRAME_CNT;
    uint32_t FRAME_INTERVAL;
    uint32_t BURST_CNT;
    uint32_t BURST_DELAY;
} CMD_RADAR_APPLICATION_CONFIG_t;

typedef struct {
    int32_t TX_POWER_NOMINAL;
    int32_t TX_POWER_OFFSET;
    int32_t TX_POWER_BOOST;
} TX_POWER_CONFIGURATION_t;

typedef struct {
    uint8_t Analog_coupling;                               // "Analog coupling"
    uint8_t Radar_data_selection;                          // "Radar data selection"
    uint8_t AGC_control_for_Radar_fast_calibration;        // "AGC control for Radar fast calibration"
} DATA_ACQ_MODE_t;

typedef struct {
    uint16_t CIR_TAPS;
    uint16_t RX_SYNC_SYMBOL_CNT;
    uint16_t CIR_OFFSET;
    int8_t SENSITIVITY_BOOST_OFFSET_RX1;
    int8_t SENSITIVITY_BOOST_OFFSET_RX2;
    uint8_t CIR_THRESHOLD_NTF;
    uint8_t TIMESTAMP_LOGGING_ENBL;
    uint8_t CIR_BUFFER_SIZE;
    DATA_ACQ_MODE_t DATA_ACQ_MODE; // Nested structure for data acquisition mode
} RX_RADAR_CIR_CONFIGURATION_t;

typedef struct {
    uint8_t TX_ENBL;
    uint16_t TX_SYNC_SYMBOL_CNT;
} TX_RADAR_CONTROL_CONFIGURATION_t;

typedef struct {
    uint8_t RX_RADAR_CALIBRATION_SELECT;
    uint8_t RNS_CONFIGURATION_INDEX;
    uint8_t RX_ENBL;
    uint8_t SENSITIVITY_BOOST;
} RX_RADAR_CONTROL_CONFIGURATION_t;

typedef struct {
    uint8_t Radar_noise_suppression;   // "Radar noise suppression"
    uint8_t Radar_drift_suppression;   // "Radar drift suppression"
    uint8_t DC_removal;                // "DC removal"
} RNS_CONTROL_t;

typedef struct {
    uint8_t SELF_INTERFERENCE_TAP;
    RNS_CONTROL_t RNS_CONTROL;
    uint32_t NOISE_SUPPRESSION_INDEX_MASK;
    float DC_FILTER_CUTOFF_HZ;
    uint8_t RNS_CALIBRATION_SLOT_INDEX;
} RNS_RX_t;

typedef struct {
    RNS_RX_t RNS_RX1;
    RNS_RX_t RNS_RX2;
} RADAR_NOISE_SUPPRESSION_CONFIGURATION_t;


typedef struct {
    uint8_t device_index;
    TX_RADIO_CONFIGURATION_t  TX_RADIO_CONFIGURATION;
    RX_RADIO_CONFIGURATION_t  RX_RADIO_CONFIGURATION;
    CMD_RADAR_APPLICATION_CONFIG_t  CMD_RADAR_APPLICATION_CONFIG;
    TX_POWER_CONFIGURATION_t  TX_POWER_CONFIGURATION;
    RX_RADAR_CIR_CONFIGURATION_t  RX_RADAR_CIR_CONFIGURATION;
    TX_RADAR_CONTROL_CONFIGURATION_t  TX_RADAR_CONTROL_CONFIGURATION;
    RX_RADAR_CONTROL_CONFIGURATION_t  RX_RADAR_CONTROL_CONFIGURATION;
    RADAR_NOISE_SUPPRESSION_CONFIGURATION_t  RADAR_NOISE_SUPPRESSION_CONFIGURATION;
    uint32_t duration_sec;
} RADAR_CONFIG_t ;

extern RADAR_CONFIG_t radar_config;

#endif // RADAR_CONFIG_H
