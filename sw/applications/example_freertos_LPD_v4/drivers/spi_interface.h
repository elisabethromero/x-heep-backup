#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include "spi_host.h" 
#include "spi_sdk.h"
#include "spi_risccom.h"
#include "Config.h"
// =========================================================
// ESP32
// =========================================================
#ifdef TARGET_ESP32
// SPI del ESP32c3
//#include "driver/spi_master.h" //solo en esp32, sustituir en fpga
#include "driver/gpio.h" //solo en esp32, sustituir en fpga
#include "driver/spi_master.h"

// Definiciones de pines GPIO para conexión SPI (ESP32c3) 
//==========
//#define GPIO_HANDSHAKE      8//2
#define GPIO_MOSI           7//12
#define GPIO_MISO           2//13
#define GPIO_SCLK           6//15
//#define GPIO_CS             10//14

// Definición del sender host
#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST HSPI_HOST
#else
#define SENDER_HOST SPI2_HOST
#endif

extern spi_device_interface_config_t devcfg;

extern spi_bus_config_t buscfg;

extern spi_device_handle_t handle;

// ==========================================================
#else
    #include "gpio.h"
    #include "fast_intr_ctrl.h"
    #include "Config.h"

    // Variables globales (declaradas en el .c)
    extern uint8_t synq;
    extern uint32_t csid;
    extern uint16_t i;
    extern spi_t spi_pynq;

#endif
// Inicializa el bus SPI y el dispositivo esclavo
void spi_initialization();

// Realiza una transacción SPI completa (envío y recepción)
//spi_return_flags_e spi_transfer(float *tx_buf, float *rx_buf, size_t len, spi_dir_e direction);
spi_codes_e spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, size_t len, spi_dir_e direction);


#endif // SPI_INTERFACE_H
