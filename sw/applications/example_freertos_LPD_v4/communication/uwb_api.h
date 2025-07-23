#ifndef UWB_API_H
#define UWB_API_H

#include <stdint.h>
#include <stdio.h>

extern bool conf_uwb_active; // Variable global para indicar si la configuración del UWB está activa

extern bool interrupt_processing_enabled;

extern bool streaming;
void task_uwb();

void uwb_menu_loop();
bool uwb_init();
void uwb_init_ant();


bool uwb_init_device();
bool uwb_conf_device();
bool uwb_start_streaming();

bool uwb_get_version();
bool uwb_reset_device();
bool uwb_disable_watchdog();
bool uwb_read_bitfield();
bool uwb_configure_rx_radio();
bool uwb_configure_tx_radio();
bool uwb_configure_tx_power();

bool uwb_generic_baseband_config(uint8_t config_select);
bool uwb_start_baseband();
bool uwb_stop_baseband();
bool uwb_get_baseband_results(uint8_t result_sel, uint8_t results_idx);

bool uwb_configure_radar_application(uint8_t radar_mode, uint8_t frame_cnt_byte, uint8_t burst_cnt_byte);
bool uwb_start_radar();





#endif // UWB_API_H