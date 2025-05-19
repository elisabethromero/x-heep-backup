#include "spi_interface.h"
#include <string.h>

#include "x-heep.h"
#include "spi_host.h"
#include "spi_slave_sdk.h"
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
        
    /*
    // Opcional, si se conoce la frecuencia exacta del sistema no es necesario
    uint32_t core_clk = soc_ctrl_get_frequency(&soc_ctrl);

    uint16_t clk_div = 1; // Valor mínimo válido
    if (PKE_CLK_MAX_HZ < core_clk / 2) {
        clk_div = (core_clk / PKE_CLK_MAX_HZ - 2) / 2;
        if (clk_div < 1) clk_div = 1; // Asegurar un divisor mínimo
        if (core_clk / (2 * clk_div + 2) > PKE_CLK_MAX_HZ) clk_div++;
    }
    */

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

    // Verifica si es maestro (SYNC_LOGIC = true)
    gpio_read(GPIO_SYNQ, &synq);
    if (SYNC_LOGIC(synq)) {
        // Modo maestro
        gpio_set_mode(GPIO_SYNQ, GpioModeOutPushPull);
        gpio_write(GPIO_SYNQ, SYNC_LOGIC(false));
        printf("[SPI] I am the master.\n");

        // Turn on the white LED to identify the master. 
        gpio_write(GPIO_LD5_R, true);
        gpio_write(GPIO_LD5_B, true);
        gpio_write(GPIO_LD5_G, true);

        // Inicializa SPI
        if (spi_host_init(spi_host1, csid) != SPI_FLAG_SUCCESS) {
            printf("[SPI] SPI host initialization failed.\n");
            return SPI_HOST_FLAG_NOT_INIT;
        }
    } else {
        // Modo esclavo (opcional)
        printf("[SPI] I am the slave.\n");
        // Turn the red LED in disapproval 
        gpio_write(GPIO_LD5_R, true);
    }
    printf("[SPI] SPI initialized successfully.\n");

    #endif
    // ======================================================

    //vTaskDelay(500 / portTICK_PERIOD_MS); //[EMRL] La he comentado porque esto es de freertos y no se puede usar aquí
}

// Función que ejecuta la tx/rx a través del buffer 
spi_return_flags_e spi_transfer(float* sendbuf, float* recvbuf, size_t len, spi_dir_e direction) {
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
    // Verifica si es maestro (SYNC_LOGIC = true)
    gpio_read(GPIO_SYNQ, &synq);
    if (SYNC_LOGIC(synq)) {
        /*
        // [EMRL] Muestro lo que tengo en el buffer de envío
        for (int i = 0; i < len; i++) {
            printf("SENDBUF:"); 
            int entero = (int)sendbuf[i];
            int decimales = (int)((sendbuf[i] - entero) * 1000); // 3 cifras decimales
            if (decimales < 0) decimales *= -1;
            printf("%d.%03d, ", entero, decimales);
        }
        printf("\n\r");
        */
        spi_slave_write(spi_host1, recvbuf, sendbuf, len*4);
        for(int i=0; i < 32; i++){
            /*
            printf("SENDBUF:"); 
            int entero = (int)sendbuf[i];
            int decimales = (int)((sendbuf[i] - entero) * 1000); // 3 cifras decimales
            if (decimales < 0) decimales *= -1;
            printf("%d.%03d ", entero, decimales);

            printf("RECVBUF:"); 
            int entero1 = (int)recvbuf[i];
            int decimales1 = (int)((recvbuf[i] - entero1) * 1000); // 3 cifras decimales
            if (decimales1 < 0) decimales1 *= -1;
            printf("%d.%03d ", entero1, decimales1);

            printf("\n\r");
            */
            if(sendbuf[i] != recvbuf[i]){
                return SPI_FLAG_RX_QUEUE_EMPTY;
            } else{
                //printf("Write Ok\n\r");
            }
        }
        printf("[SPI] SPI transaction success!\n\r");
        gpio_write(GPIO_LD5_G, true);
        gpio_write(GPIO_LD5_R, false);
        gpio_write(GPIO_LD5_B, false);
        return SPI_FLAG_OK;
    }
    
    return SPI_FLAG_OK;
    
    #endif

}