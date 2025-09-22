#include <string.h>
#include <stdio.h>

#include "spi_risccom.h"

#define RX_LEN 16

/****************************************************************************//*
 * Public code
 ******************************************************************************/

int main()
{
    uint8_t tx_buf[16] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00 };
    uint8_t rx_buf[RX_LEN] = {0};
    bool ret;
    
    //Init board
    printf("RISCOM Program started.\r\n");
    ret = riscom_platform_init();
    if (ret)
    {
        printf("Platform initialization correct.\r\n");
    }
    else
    {
        printf("Platform initialization error.\r\n");
        return -1;
    }

    /****************************************************************************//*
     * TRANSMIT SPI DATA
    ******************************************************************************/
    /*if (riscom_platform_spi_transmit(tx_buf, sizeof(tx_buf))) {
        printf("Código enviado por maestro SPI: ");
        for (int i = 0; i < sizeof(tx_buf); i++) {
            printf("%02X ", tx_buf[i]);
        }
        printf("\n");
    } else {
        printf("Error de transmisión.\n");
    }*/

    /****************************************************************************//*
     * RECEIVE SPI DATA
    ******************************************************************************/
/*
    printf("Esperando datos desde el esclavo SPI...\n");

    if (riscom_platform_spi_receive(rx_buf, RX_LEN)) {
        printf("Datos recibidos del esclavo SPI: ");
        for (int i = 0; i < RX_LEN; i++) {
            printf("%02X ", rx_buf[i]);
        }
        printf("\n");
    } else {
        printf("Error de recepción.\n");
    }
*/
    /****************************************************************************//*
     * TRANSCEIVE SPI DATA
    ******************************************************************************/

    if (riscom_platform_spi_transceive(tx_buf, rx_buf, 16)) {
        printf("Datos recibidos del esclavo SPI: ");
        for (int i = 0; i < RX_LEN; ++i) {
            printf("%02X ", rx_buf[i]);
        }
        printf("\n");
        printf("Código enviado por maestro SPI: ");
        for (int i = 0; i < sizeof(tx_buf); i++) {
            printf("%02X ", tx_buf[i]);
        }
        printf("\n");
    } else {
        printf("Fallo en la transacción.\n");
    }
}

/******************************** End of file *********************************/