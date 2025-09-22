#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "spi_sdk.h"
#include "timer_sdk.h"
#include "csr.h"
#include "csr_registers.h"

/****************************************************************************//*
 * Constants and Macros
 ******************************************************************************/

#define RISCOM_PLATFORM_SPI_DEBUG

// ACTIVAR DEBUG TEMPORAL PARA DIAGNOSTICAR
#define RISCOM_PLATFORM_SPI_DEBUG

#define RISCOM_PLATFORM_SPI         SPI_IDX_HOST
#define RISCOM_PLATFORM_SPI_SPEED   2000
#define RISCOM_PLATFORM_SPI_CS      0

// CS MANUAL usando GPIO[0] - igual que main.c
#define CS_GPIO 6 //Antes 0

#define FIC_SPI_HOST_MEIE  20               // SPI Host 1 fast interrupt bit enable
#define CSR_INTR_EN        0x08             // CPU Global interrupt enable

/* SPI Instance */
static spi_t riscom_platform_spi_inst;

// Funciones para CS manual
static void setup_manual_cs(void) {    
    gpio_cfg_t cs_config = {0};
    cs_config.pin = CS_GPIO;
    cs_config.mode = GpioModeOutPushPull;
    cs_config.en_input_sampling = true;
    cs_config.en_intr = false;
    
    gpio_result_t result = gpio_config(cs_config);
    gpio_write(CS_GPIO, true);  // CS idle high
}

static void cs_low(void) {
    gpio_write(CS_GPIO, false);
}

static void cs_high(void) {
    gpio_write(CS_GPIO, true);
}

bool riscom_platform_init(void)
{
    // Configurar CS manual primero
    setup_manual_cs();

    /** SPI **/
    spi_slave_t spi_slave = {
        .csid = RISCOM_PLATFORM_SPI_CS,
        .csn_idle = 10,
        .csn_lead = 10,
        .csn_trail = 10,
        .data_mode = SPI_DATA_MODE_0,
        .full_cycle = true,
        .freq = RISCOM_PLATFORM_SPI_SPEED
    };

    riscom_platform_spi_inst = spi_init(RISCOM_PLATFORM_SPI, spi_slave);

    if (!riscom_platform_spi_inst.init)
    {
        return false;
    }

    // Enable global interrupt for machine-level interrupts
    CSR_SET_BITS(CSR_REG_MSTATUS, CSR_INTR_EN);
    // Set mie.MEIE bit to one to enable machine-level fast spi_host interrupt
    const uint32_t mask = 1 << FIC_SPI_HOST_MEIE;
    CSR_SET_BITS(CSR_REG_MIE, mask);

    return true;
}

bool riscom_platform_spi_transmit(uint8_t * data, uint32_t length)
{
    spi_codes_e spi_ret;

#ifdef RISCOM_PLATFORM_SPI_DEBUG
    printf("SPI TRANSMIT: ");
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
#endif //RISCOM_PLATFORM_SPI_DEBUG

    // Activar CS manualmente antes de transmitir
    cs_low();
    for (volatile int i = 0; i < 50000; i++);  // Delay para CS estable

    spi_ret = spi_transmit(&riscom_platform_spi_inst, data, length);
    
    // Desactivar CS manualmente después de transmitir
    for (volatile int i = 0; i < 50000; i++);  // Delay antes de CS high
    cs_high();

    if (spi_ret != SPI_CODE_OK)
    {
#ifdef RISCOM_PLATFORM_SPI_DEBUG
        printf("SPI TRANSMIT ERROR (%u)\r\n", spi_ret);
#endif //RISCOM_PLATFORM_SPI_DEBUG
        return false;
    }

    return true;
}

bool riscom_platform_spi_receive(uint8_t * data, uint32_t length)
{
    spi_codes_e spi_ret;

    // Activar CS manualmente antes de recibir
    cs_low();
    for (volatile int i = 0; i < 50000; i++);  // Delay para CS estable

    spi_ret = spi_receive(&riscom_platform_spi_inst, data, length);
    
    // Desactivar CS manualmente después de recibir
    for (volatile int i = 0; i < 50000; i++);  // Delay antes de CS high
    cs_high();

    if (spi_ret != SPI_CODE_OK)
    {
#ifdef RISCOM_PLATFORM_SPI_DEBUG
        printf("SPI RECEIVE ERROR (%u)\r\n", spi_ret);
#endif //RISCOM_PLATFORM_SPI_DEBUG
        return false;
    }

#ifdef RISCOM_PLATFORM_SPI_DEBUG
    printf("SPI RECEIVE: ");
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
#endif //RISCOM_PLATFORM_SPI_DEBUG

    return true;
}

bool riscom_platform_spi_transceive(const uint8_t *tx_data, uint8_t *rx_data, uint32_t length)
{
    spi_codes_e spi_ret;

    cs_low();
    for (volatile int i = 0; i < 50000; i++);  // Delay para CS estable

    spi_ret = spi_transceive(&riscom_platform_spi_inst, tx_data,rx_data, length);

    cs_high();


    printf("SPI TRANSCEIVE MANUAL (%u bytes)\r\n", (unsigned int)length);
    printf("SPI TX: ");
    for (size_t i = 0; i < length; i++) printf("%02X ", tx_data[i]);
    printf("\r\n");

    
    printf("SPI RX: ");
    for (size_t i = 0; i < length; i++) printf("%02X ", rx_data[i]);
    printf("\r\n");

    printf("=== TRANSACCIÓN COMPLETA ===\r\n");
    return true;
}

/******************************** End of file *********************************/