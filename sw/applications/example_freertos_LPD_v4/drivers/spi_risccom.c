#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "spi_sdk.h"
#include "timer_sdk.h"
#include "csr.h"
#include "csr_registers.h"
#include "Config.h"

/****************************************************************************//*
 * Constants and Macros
 ******************************************************************************/

//#define RISCOM_PLATFORM_SPI_DEBUG

// ACTIVAR DEBUG TEMPORAL PARA DIAGNOSTICAR
//#define RISCOM_PLATFORM_SPI_DEBUG

#define RISCOM_PLATFORM_SPI         SPI_IDX_HOST
#define RISCOM_PLATFORM_SPI_SPEED   500000
#define RISCOM_PLATFORM_SPI_CS      0

#define FIC_SPI_HOST_MEIE  20               // SPI Host 1 fast interrupt bit enable
#define CSR_INTR_EN        0x08             // CPU Global interrupt enable

/* SPI Instance */
static spi_t riscom_platform_spi_inst;


static void cs_low(void) {
    gpio_write(GPIO_CS, false);
}

static void cs_high(void) {
    gpio_write(GPIO_CS, true);
}

bool riscom_platform_init(void)
{

    /** SPI **/
    spi_slave_t idneo_slave = {
        .csid = RISCOM_PLATFORM_SPI_CS,
        .csn_idle = 10,
        .csn_lead = 10,
        .csn_trail = 10,
        .data_mode = SPI_DATA_MODE_0,
        .full_cycle = true,
        .freq = RISCOM_PLATFORM_SPI_SPEED
    };

    riscom_platform_spi_inst = spi_init(RISCOM_PLATFORM_SPI, idneo_slave);

    if (!riscom_platform_spi_inst.init)
    {
        return false;
    }

    // Enable global interrupt for machine-level interrupts
    //CSR_SET_BITS(CSR_REG_MSTATUS, CSR_INTR_EN);
    // Set mie.MEIE bit to one to enable machine-level fast spi_host interrupt
    //const uint32_t mask = 1 << FIC_SPI_HOST_MEIE;
    //CSR_SET_BITS(CSR_REG_MIE, mask);

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


    for (volatile int i = 0; i < 5000; i++);  // Delay para CS estable

    spi_ret = spi_transmit(&riscom_platform_spi_inst, data, length);


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
    //cs_low();
    //for (volatile int i = 0; i < 5000; i++);  // Delay para CS estable

    spi_ret = spi_receive(&riscom_platform_spi_inst, &data, length);
    
    // Desactivar CS manualmente después de recibir
    //cs_high();

    if (spi_ret != SPI_CODE_OK)
    {
#ifdef RISCOM_PLATFORM_SPI_DEBUG
        printf("SPI RECEIVE ERROR (%u)\r\n", spi_ret);
#endif //RISCOM_PLATFORM_SPI_DEBUG
        return false;
    }

    return true;
}

bool riscom_platform_spi_transceive(const uint8_t *tx_data, uint8_t *rx_data, uint32_t length)
{
    spi_codes_e spi_ret;

    
    for (volatile int i = 0; i < 10000; i++);  // Delay para CS estable

    spi_ret = spi_transceive(&riscom_platform_spi_inst, tx_data,rx_data, length);

    //vTaskDelay(pdMS_TO_TICKS(100)); //EN FUCNION DEL LENGTH

   


    printf("SPI TRANSCEIVE MANUAL (%u bytes)\r\n", (unsigned int)length);
    /*printf("SPI TX: ");
    for (size_t i = 0; i < length; i++) printf("%02X ", tx_data[i]);
    printf("\r\n");

    
    printf("SPI RX: ");
    for (size_t i = 0; i < length; i++) printf("%02X ", rx_data[i]);
    printf("\r\n");*/

    //printf("=== TRANSACCIÓN COMPLETA ===\r\n");
    return true;

}

/******************************** End of file *********************************/