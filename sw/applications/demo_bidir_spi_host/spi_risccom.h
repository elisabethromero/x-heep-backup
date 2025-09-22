#ifndef SPI_RISCCOM_H
#define SPI_RISCCOM_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Run platform initialization.
 * SPI: Master mode, 100Kbit/s, CPOL=0, CPHA=0, data size 8bits.
 * GPIO: Leds (output low), Button (input pullup), Reset (output high), CS (output high)
 * @return bool : True if success, False otherwise.
 */
bool riscom_platform_init(void);

/**
 * @brief Transmit SPI data.
 * @param data : Data buffer to transmit.
 * @param length : Length of the data buffer.
 * @return bool : True if success, False otherwise.
 */
bool riscom_platform_spi_transmit(uint8_t * data, uint32_t length);

/**
 * @brief Receive SPI data.
 * @param data : Data buffer to transmit.
 * @param length : Length of the data buffer.
 * @return bool : True if success, False otherwise.
 */
bool riscom_platform_spi_receive(uint8_t * data, uint32_t length);

/**
 * @brief Transmit and receive SPI data simultaneously.
 *
 * Performs a full-duplex SPI transaction, transmitting the contents of 
 * the TX buffer while simultaneously receiving data into the RX buffer.
 * Internally ensures proper memory alignment for 32-bit word transfers.
 *
 * @param tx_data : Pointer to the buffer containing data to transmit.
 * @param rx_data : Pointer to the buffer where received data will be stored.
 * @param length  : Length of the data buffers in bytes. Must be a multiple of 4.
 *
 * @return bool : True if the transaction succeeded, False otherwise.
 */
bool riscom_platform_spi_transceive(const uint8_t *tx_data, uint8_t *rx_data, uint32_t length);

#endif //SPI_RISCCOM_H

/******************************** End of file *********************************/