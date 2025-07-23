#include "spi_interface.h"
#include <string.h>
// #include "spi_host.h"
#include "gpio.h"
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

spi_host_t* spi = spi_host1;  // El dispositivo a usar (dummy en ESP32)
uint32_t csid = 0;            // Chip select

// 1. CONFIGURACIÓN SPI
spi_configopts_t config = {
    .clkdiv = 2,
    .csnidle = 1,
    .csntrail = 1,
    .csnlead = 1,
    .fullcyc = 0,
    .cpha = 0,
    .cpol = 0
};

//Convierte la configuración configopts (entrada) en una palabra de 32 bits config_reg (salida)
uint32_t conf_word = spi_create_configopts(config);


#endif


// Inicialización del SPI
void spi_init() {
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

    //Aplicación de la configuración del esclavo al SPI_Host
    if (spi_set_configopts(spi, csid, conf_word) != SPI_FLAG_OK) {
        printf("Error al configurar el dispositivo esclavo\n");
        return -1;
    }
    
    // HABILITAR SPI
    spi_set_enable(spi, true);
    spi_output_enable(spi, true);
    
    // SELECCIONAR CHIP SELECT
    if (spi_set_csid(spi, csid) != SPI_FLAG_OK) {
        printf("Error al establecer CSID\n");
        return -1;
    }
    #endif
    // ======================================================
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

// Función que ejecuta la tx/rx a través del buffer 
spi_return_flags_e spi_transfer(uint8_t* sendbuf, uint8_t* recvbuf, size_t len, spi_dir_e direction) {
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
    
    spi_return_flags_e result;

    // Crear y configurar el comando SPI
    spi_command_tr cmd = {
        .len = len * sizeof(float),  // en bytes
        .csaat = false,
        .speed = SPI_SPEED_STANDARD,
        .direction = SPI_DIR_BIDIR
    };
    uint32_t cmd_reg = spi_create_command(cmd);

    // Esperar que esté listo
    spi_wait_for_ready(spi_host1);

    // Enviar el comando al dispositivo
    spi_set_command(spi_host1, cmd_reg);

   
    // 4. TX (Escribir los datos a enviar)
    if (direction == SPI_DIR_TX_ONLY || direction == SPI_DIR_BIDIR) {
        for (int i = 0; i < len; i++) {
            spi_write_word(spi_host1, ((uint32_t*)sendbuf)[i]);
        }
    }

    // 5. Esperar que se vacíe el FIFO de TX
    if (direction == SPI_DIR_TX_ONLY || direction == SPI_DIR_BIDIR) {
        spi_wait_for_tx_empty(spi_host1);
    }

    // 6. RX (Leer los datos recibidos)
    if (direction == SPI_DIR_RX_ONLY || direction == SPI_DIR_BIDIR) {
        for (int i = 0; i < len; i++) {
            spi_read_word(spi_host1, (uint32_t*)&recvbuf[i]);
        }
    }

    return SPI_FLAG_OK;
    
    #endif

}