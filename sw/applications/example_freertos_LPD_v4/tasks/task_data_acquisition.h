#ifndef TASK_DATA_ACQUISITION_H
#define TASK_DATA_ACQUISITION_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CIR_BYTES (CIR_TAPS * 4) // 32 taps * 4 bytes/tap = 128 bytes


// Declaración de funciones
void receive_data(complex_float_t *cir_taps);
void handle_buffering(complex_float_t *cir_taps);
void task_data_acquisition(void *pvParameter);

void adjustLinearBuffer(LinearBuffer *lin_buffer);

// Mostrar valores recibidos a través de SPI en tr.tx_buffer
void printReceivedData(float *recvbuf);

void receive_cir_from_uwb(uint8_t *cir_bytes);
void simulate_cir_input(uint8_t *cir_bytes_out);
void testing_cir_input(uint8_t *cir_bytes_out);

void print_raw_bytes(const uint8_t *buf);
void extract_and_convert_to_complex(const uint8_t *cir_bytes, complex_float_t *cir_taps);
void print_complex_cir(complex_float_t *cir_taps);

// Variables globales compartidas entre tareas
extern SemaphoreHandle_t data_sampling_ready_sem;  // Semáforo para sincronización con otras tareas
extern SemaphoreHandle_t data_fft_ready_sem;  // Semáforo para sincronización con otras tareas

#endif // TASK_DATA_ACQUISITION_H
