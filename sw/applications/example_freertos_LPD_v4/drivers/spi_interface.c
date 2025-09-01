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

esp_err_t ret;

// ==========================================================
// PYNQ-Z2
// ==========================================================
#else

spi_t spi_pynq;
//[EMRL] Chip select
uint32_t csid = 1;


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
        printf("\n>> Initializing SPI bus...\n");

        // Definimos al B-Sample como esclavo con GPIO_CS 0 y frecuencia 100 kbits/s
        /** SPI **/
        spi_slave_t idneo_slave = {
            .csid = csid,
            .csn_idle = 15, //15 por defecto
            .csn_lead = 15, //15 por defecto
            .csn_trail = 15, //15 por defecto y .cs_ena_posttrans = 3 en la ESP32
            .data_mode = SPI_DATA_MODE_0, // Yo creo que es SPI_DATA_MODE_2 en vez de 0
            .full_cycle = true, 
            .freq = IDNEO_SPI_SPEED
        };

        // Validamos el csid del slave
        if (spi_validate_slave(idneo_slave) != SPI_CODE_OK) {
            printf("Error: csid no válido\n");
            return;
        }

        // Inicializamos SPI1 (HOST1)
        spi_pynq = spi_init(IDNEO_SPI, idneo_slave); 

        if (!spi_pynq.init) {
            printf("Error al inicializar SPI\n");
            return false;
        }
        gpio_write(GPIO_CS, true); 

        // Enable global interrupt for machine-level interrupts
        //CSR_SET_BITS(CSR_REG_MSTATUS, CSR_INTR_EN);
        // Set mie.MEIE bit to one to enable machine-level fast spi_flash interrupt
        //const uint32_t mask_spi_flash = 1 << FIC_FLASH_MEIE; //FIC_SPI_HOST_MEIE
        //const uint32_t mask_spi_host = 1 << FIC_SPI_HOST_MEIE; 
        //CSR_SET_BITS(CSR_REG_MIE, mask_spi_flash);
        //CSR_SET_BITS(CSR_REG_MIE, mask_spi_host);

        spi_state_e state = spi_get_state(&spi_pynq);
        if (state != SPI_STATE_INIT) {
            printf("Error: SPI not initialized properly\n");
            printf("Problema en la SPI: %d\n", state);
            return;
        }

        printf("[SPI] SPI initialized successfully.\n");
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
    
        // Validamos el bus spi antes de cualquier operación
        if (spi_prepare_transfer(&spi_pynq) != SPI_CODE_OK) {
            printf("Error setting transfer\n");
            return;
        }

        uint32_t length = len * 8;  // en bits

        spi_codes_e code;

        switch (direction) {
            case SPI_DIR_TX_ONLY:
                // code = spi_transmit(&spi_pynq, txbuf, sizeof(sendbuf));
                code = spi_transmit(&spi_pynq, sendbuf, length);
                break;

            case SPI_DIR_RX_ONLY:
                //code = spi_receive(&spi_pynq, rxbuf, sizeof(recvbuf));
                code = spi_receive(&spi_pynq, recvbuf, length);
                break;

            case SPI_DIR_BIDIR:{

                if (length == 0 || length % 4 != 0) {
                    // Longitud inválida para palabras de 32 bits
                    return SPI_CODE_IDX_INVAL;
                }

                // Buffers temporales alineados a 4 bytes
                uint32_t tx_buf_aligned[length / 4];
                uint32_t rx_buf_aligned[length / 4];

                // Copiar datos TX a buffer alineado
                memcpy(tx_buf_aligned, sendbuf, length);

                code = spi_transceive(&spi_pynq, tx_buf_aligned, rx_buf_aligned, length);

                // Copiar resultado RX al buffer de usuario
                memcpy(recvbuf, rx_buf_aligned, length);
                }
                break;

            default:
                return SPI_CODE_IDX_INVAL; // Dirección no válida
        }

        // Traduce el código de retorno del SDK a tus flags
        //return (code == SPI_CODE_OK) ? SPI_CODE_OK : SPI_FLAG_ERROR;

        spi_state_e state = spi_get_state(&spi_pynq);
        if (state != SPI_STATE_DONE) {
            printf("Problema en la SPI: %d\n", state);
            //return;
        } else {
            printf("SPI successfully executed a transaction\n");
        }

        return code;
    
    #endif

}