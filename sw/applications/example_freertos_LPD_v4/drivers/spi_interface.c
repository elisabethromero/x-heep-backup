#include "spi_interface.h"
#include <string.h>
#include "spi_host.h"
#include "x-heep.h"
#include "spi_sdk.h"
#include "gpio.h"
#include "hart.h"
#include "timer_sdk.h"
#include "Config.h"
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
    .spics_io_num = -1, // Desactivar control automático del CS
    // .spics_io_num = GPIO_CS,
    .cs_ena_posttrans = 3,      //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
    .queue_size = 3
};

//Manejador del dispositivo SPI
spi_device_handle_t handle;



// ==========================================================
// PYNQ-Z2
// ==========================================================
#else

spi_t spi_pynq;
//[EMRL] Chip select
uint32_t csid = 1;

bool ret;


#endif


// Inicialización del SPI
void spi_initialization() {
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

    // Configurar el pin CS como salida y ponerlo en alto
    gpio_set_direction(GPIO_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_CS, 1); // CS alto por defecto
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //printf("Nivel del gpio CS1: %d\n", gpio_get_level(GPIO_CS));
    printf("SPI bus configuration completed successfully.\n");

    // =======================================================
    // PYNQ-Z2
    // =======================================================
    #else
        printf("SPI communication started.\r\n");
        ret = riscom_platform_init();
        if (ret)
        {
            printf("SPI initialization correct.\r\n");
        }
        else
        {
            printf("SPI initialization error.\r\n");
            return;
        }

    #endif
    // ======================================================
    
    //vTaskDelay(500 / portTICK_PERIOD_MS);
}

// Función que ejecuta la tx/rx a través del buffer 
spi_codes_e spi_transfer(uint8_t* sendbuf, uint8_t* recvbuf, size_t len, spi_dir_e direction) {
    // ======================================================
    // ESP32
    // ======================================================
    #ifdef TARGET_ESP32

    // Envío y recepción de datos por SPI
    spi_transaction_t tr;
    memset(&tr, 0, sizeof(tr));
    tr.length = len * 8;  // en bits
    tr.tx_buffer = (direction == SPI_DIR_TX_ONLY || direction == SPI_DIR_BIDIR) ? sendbuf : NULL;
    tr.rx_buffer = (direction == SPI_DIR_RX_ONLY || direction == SPI_DIR_BIDIR) ? recvbuf : NULL;
    // Debug
    if (direction == SPI_DIR_TX_ONLY) {
        // printf("\tAntes de enviar tx_buffer: ");
        // const uint8_t *tx = (const uint8_t *)tr.tx_buffer;
        // for (int i = 0; i < len; i++) {
        //     printf("%02X ", tx[i]);
        // }
        // printf("\n");
    
    } else if (direction == SPI_DIR_RX_ONLY) {
        // printf("\tAntes de recibir rx_buffer: ");
        // const uint8_t *rx = (const uint8_t *)tr.rx_buffer;
        // for (int i = 0; i < len; i++) {
        //     printf("%02X ", rx[i]);
        // }
        // printf("\n");
    
    } else if (direction == SPI_DIR_BIDIR) {
        // printf("\tAntes de enviar tx_buffer: ");
        // const uint8_t *tx = (const uint8_t *)tr.tx_buffer;
        // for (int i = 0; i < len; i++) {
        //     printf("%02X ", tx[i]);
        // }
        // printf("\n");
    
        // printf("\tAntes de recibir rx_buffer: ");
        // const uint8_t *rx = (const uint8_t *)tr.rx_buffer;
        // for (int i = 0; i < len; i++) {
        //     printf("%02X ", rx[i]);
        // }
        // printf("\n");
    }
    

    ret = spi_device_transmit(handle, &tr);
    if (ret != ESP_OK) {
        printf("SPI transaction failed. (Error code: %d)\n", ret);
    }

    // Imprimir el buffer de transmisión
    if (direction == SPI_DIR_TX_ONLY) {
        // printf("\tDespués de enviar tx_buffer: ");
        // const uint8_t *tx = (const uint8_t *)tr.tx_buffer;
        // for (int i = 0; i < len; i++) {
        //     printf("%02X ", tx[i]);
        // }
        // printf("\n");
    }
    //Imprimir el buffer de recepción
    //printf("\tNivel de CS: %d\n", gpio_get_level(GPIO_CS));
    if ((direction == SPI_DIR_RX_ONLY) || (direction == SPI_DIR_BIDIR)){
        // printf("\tDespués de recibir rx_buffer: ");
        //     const uint8_t *rx = (const uint8_t *)tr.rx_buffer;
        //     for (int i = 0; i < len; i++) {
        //         printf("%02X ", rx[i]);
        //     }
        //     printf("\n");
        }
    return SPI_FLAG_OK;

    // =======================================================
    // PYNQ-Z2
    // =======================================================
    #else

        spi_codes_e code;

        uint8_t dummy_tx[len]; // Buffer de transmisión vacío
        for (int i = 0; i < len; i++) {
            dummy_tx[i] = 0x00; // Rellenar con ceros
        }

        switch (direction) {
            case SPI_DIR_TX_ONLY:
                if (riscom_platform_spi_transmit(sendbuf, (uint32_t)len)) {
                    /*printf("Código enviado por maestro SPI: ");
                    for (int i = 0; i < len; i++) {
                        printf("%02X ", sendbuf[i]);
                    }
                    printf("\n");*/
                    code = SPI_CODE_OK;
                } else {
                    printf("Error de transmisión.\n");
                }
                
                break;

            case SPI_DIR_RX_ONLY:
                if (riscom_platform_spi_transceive(dummy_tx, recvbuf, (uint32_t)len)) {
                    /*printf("Datos recibidos del esclavo SPI: ");
                    for (int i = 0; i < len; i++) {
                        printf("%02X ", recvbuf[i]);
                    }
                    printf("\n");*/
                    code = SPI_CODE_OK;
                } else {
                    printf("Error de recepción.\n");
                }
                break;

            case SPI_DIR_BIDIR:
                if (riscom_platform_spi_transceive(sendbuf, recvbuf, 16)) {
                    printf("Datos recibidos del esclavo SPI: ");
                    for (int i = 0; i < len; ++i) {
                        printf("%02X ", recvbuf[i]);
                    }
                    printf("\n");
                    printf("Código enviado por maestro SPI: ");
                    for (int i = 0; i < len; i++) {
                        printf("%02X ", sendbuf[i]);
                    }
                    printf("\n");
                    code = SPI_CODE_OK;
                } else {
                    printf("Fallo en la transacción.\n");
                }
                break;

            default:
                code = SPI_CODE_TIMEOUT_INVAL; // Dirección no válida
        }

        return code;
    
    #endif

}