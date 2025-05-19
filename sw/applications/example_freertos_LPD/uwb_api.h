#ifndef UWB_API_H
#define UWB_API_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Estructura de respuesta de la API
typedef struct {
    bool success;
    char message[256];
} UWB_Response;

// Inicialización del dispositivo
UWB_Response initializeDevice(uint8_t device_index, uint32_t spi_timeout_ms);

// Deshabilitar el watchdog
UWB_Response disableWatchdog(uint8_t device_index);

// Configuración del radar
UWB_Response configureRxRadio(uint8_t device_index, void *config);
UWB_Response configureTxRadio(uint8_t device_index, void *config);
UWB_Response configureTxPower(uint8_t device_index, void *config);
UWB_Response configureRadarCIR(uint8_t device_index, void *config);
UWB_Response configureNoiseSuppression(uint8_t device_index, void *config);
UWB_Response configureRxRadarControl(uint8_t device_index, void *config);
UWB_Response configureTxRadarControl(uint8_t device_index, void *config);

// Calibración y configuración
UWB_Response fullCalibration(void);
UWB_Response configureRadarApplication(uint8_t device_index, void *config);

// Recepción de datos
UWB_Response receiveData(uint8_t device_index, uint8_t *buffer, uint32_t length);

// Envío de comandos UCI
UWB_Response sendUciCommand(uint8_t device_index, const char *command, void *params, float timeout);

#endif // UWB_API_H
