#include "spi_interface.h"
#include <string.h>

#include "x-heep.h"
#include "spi_host.h"
#include "spi_sdk.h"
#include "gpio.h"
#include "hart.h"
#include "timer_sdk.h"
// =========================================================
// ESP32
// =========================================================
#ifdef TARGET_ESP32

// Configuration for the SPI bus
spi_bus_config_t buscfg = {
    .mosi_io_num = GPIO_MOSI,
    .miso_io_num = GPIO_MISO,
    .sclk_io_num = GPIO_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
};

// Configuration for the SPI device on the other side of the bus
spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1000000,
    .duty_cycle_pos = 128,      //50% duty cycle
    .mode = 0,
    .spics_io_num = GPIO_CS,
    .cs_ena_posttrans = 3,      //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
    .queue_size = 3
};

//Manejador del dispositivo SPI
spi_device_handle_t handle;

esp_err_t ret;

// ==========================================================
// PYNQ-Z2
// ==========================================================
#else

spi_t spi_pynq;

#endif


// Inicialización del SPI
void spi_initialization() { //[EMRL] La modificacion del nombre se debe a que otro driver interno tiene una función con el mismo nombre
    // ======================================================
    // ESP32
    // ======================================================
    #ifdef TARGET_ESP32
    printf("\n>> Initializing SPI bus...\n");

    //4. Inicializar el bus SPI
    ret = spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    //assert(ret == ESP_OK);
    if (ret != ESP_OK) {
        printf("   SPI bus initialization failed. (Error code: %d)\n", ret);
        return;
    }
    //5. Añadir el dispositivo esclavo
    ret = spi_bus_add_device(SENDER_HOST, &devcfg, &handle);    
    //assert(ret == ESP_OK);
    if (ret != ESP_OK) {
        printf("   SPI slave device registration failed. (Error code: %d)\n", ret);
        return;
    }

    printf("SPI bus configuration completed successfully.\n");

    // =======================================================
    // PYNQ-Z2
    // =======================================================
    #else
    printf("\n>> Initializing SPI bus...\n");

    // Definimos al B-Sample como esclavo con GPIO_CS 0 y frecuencia 100MHz
    spi_slave_t idneo_slave = SPI_SLAVE(GPIO_CS, IDNEO_SPI_SPEED);
    idneo_slave.data_mode = SPI_DATA_MODE_0;  // CPOL = 0, CPHA = 0

    // Inicializamos SPI1 (HOST1)
    spi_pynq = spi_init(SPI_IDX_HOST, idneo_slave); 

    if (!spi_pynq.init) {
        printf("Error al inicializar SPI\n");
        return false;
    }

    // Enable global interrupt for machine-level interrupts
    CSR_SET_BITS(CSR_REG_MSTATUS, CSR_INTR_EN);
    // Set mie.MEIE bit to one to enable machine-level fast spi_flash interrupt
    const uint32_t mask = 1 << FIC_FLASH_MEIE;
    CSR_SET_BITS(CSR_REG_MIE, mask);

    printf("[SPI] SPI initialized successfully.\n");

    #endif
    // ======================================================

    //vTaskDelay(500 / portTICK_PERIOD_MS); //[EMRL] La he comentado porque esto es de freertos y no se puede usar aquí
}

// Función que ejecuta la tx/rx a través del buffer 
spi_codes_e spi_transfer(float* sendbuf, float* recvbuf, size_t len, spi_dir_e direction) {
    // ======================================================
    // ESP32
    // ======================================================
    #ifdef TARGET_ESP32

    // Envío y recepción de datos por SPI
    spi_transaction_t tr;
    memset(&tr, 0, sizeof(tr));
    tr.length = len * sizeof(float) * 8;  // en bits
    tr.tx_buffer = (direction == SPI_DIR_TX_ONLY || direction == SPI_DIR_BIDIR) ? sendbuf : NULL;
    tr.rx_buffer = (direction == SPI_DIR_RX_ONLY || direction == SPI_DIR_BIDIR) ? recvbuf : NULL;

    ret = spi_device_transmit(handle, &tr);
    if (ret != ESP_OK) {
        printf("SPI transaction failed. (Error code: %d)\n", ret);
    }

    return SPI_FLAG_OK;

    // =======================================================
    // PYNQ-Z2
    // =======================================================
    #else
        // Convertimos los punteros float* a uint32_t* como espera el SDK
        const uint32_t* txbuf = (const uint32_t*)sendbuf;
        uint32_t* rxbuf = (uint32_t*)recvbuf;
        size_t byte_len = len * sizeof(float); // El SDK espera longitud en bytes

        spi_codes_e code;

        switch (direction) {
            case SPI_DIR_TX_ONLY:
                code = spi_transmit(&spi_pynq, txbuf, byte_len);
                break;

            case SPI_DIR_RX_ONLY:
                code = spi_receive(&spi_pynq, rxbuf, byte_len);
                break;

            case SPI_DIR_BIDIR:
                code = spi_transceive(&spi_pynq, txbuf, rxbuf, byte_len);
                break;

            default:
                return SPI_CODE_IDX_INVAL; // Dirección no válida
        }

        // Traduce el código de retorno del SDK a tus flags
        //return (code == SPI_CODE_OK) ? SPI_CODE_OK : SPI_FLAG_ERROR;

        return code;
    
    #endif

}