#ifndef UWB_API_H
#define UWB_API_H

#include <stdint.h>
#include <stdio.h>

extern const char *json_config_str;

void uwb_menu_loop();
void uwb_init_ant();

void uwb_get_version();
void uwb_reset_device();
void uwb_disable_watchdog();
void uwb_read_bitfield();
void uwb_configure_rx_radio();
void uwb_configure_tx_radio();
void uwb_configure_tx_power();

void uwb_generic_baseband_config(uint8_t config_select);
void uwb_start_baseband();
void uwb_stop_baseband();
void uwb_get_baseband_results(uint8_t result_sel, uint8_t results_idx);

void uwb_configure_radar_application(uint8_t radar_mode, uint8_t frame_cnt_byte, uint8_t burst_cnt_byte);
void uwb_start_radar();





#endif // UWB_API_H