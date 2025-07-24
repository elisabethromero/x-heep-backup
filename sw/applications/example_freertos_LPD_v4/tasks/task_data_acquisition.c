#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "spi_host.h"
#include "spi_sdk.h" 
#include "gpio.h" 
#include <math.h>
#include "model.h"
#include "circular_buffer.h"
#include "linear_buffer.h"
#include "uwb_api.h"
#include "timer_sdk.h"
#include "task_control.h"
#include "task_data_acquisition.h"
#include "task_algorithm_execution.h"
#include "spi_interface.h"
#include "gpio.h"
#include "sync.h"
#include "config.h"
#include "uwb_core.h"
#include "cir_input_sequence_EXE_EEE_3.h"
//SemaphoreHandle_t data_sampling_ready_sem = NULL;
//SemaphoreHandle_t data_fft_ready_sem = NULL;



complex_float_t cir_taps[CIR_TAPS];

extern TickType_t start_acquisition_time;

// ACQUISITION
// 1) Llegan 128 bytes

// 2) Se cogen de 4 en 4 (32 bits) -> 1 tap esta formado por 4 bytes
    // B1 y b0 -> parte real 
    // B3 y B2 -> parte imag
// Se forma el tap complejo: Al coger los bytes tambien se convierten a float

// 3) Se guardan todos los taps complejos en el buffer circular

// 4) Cuando haya 10 seg guardados (nº taps x nºtaps/s) -> se copian en buffer lineal


// Tarea de adquisición de datos del módulo UWB
void task_data_acquisition(void *pvParameter) {
    while (1) {
        if(acquisition_active){
            
            if (total_cirs == 0) {
               if (SOURCE_DATA == 1){
                    printf("\n==================================================================================================================\n");
                    printf("\n[ACQUISITION] Capturing simulated data...\n");
               }
                else if (SOURCE_DATA == 2) {
                    printf("\n==================================================================================================================\n");
                    printf("\n[ACQUISITION] Capturing training data...\n");
                }
                else if (SOURCE_DATA == 3 && interrupt_processing_enabled) {
                    printf("\n==================================================================================================================\n");
                    printf("\n[ACQUISITION] Capturing data from the UWB module...\n");
                }
            }
            
            // Lectura de datos de entrada
            receive_data(cir_taps);

            // Procesar los datos recibidos y almacenarlos en el buffer circular 
            handle_buffering(cir_taps);
        }
               
        // Intervalo de muestreo depende de CIRs por segundo
        //vTaskDelay(pdMS_TO_TICKS(1000/CIRs_PER_SECOND)); 
        TickType_t xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / CIRs_PER_SECOND));
    }
}


void receive_data(complex_float_t *cir_taps) {
    //===========================================================================================================
    
    uint8_t cir_bytes[CIR_BYTES] = {0}; // Buffer para recibir 128 bytes (32 taps * 4 bytes/tap)

    if (SOURCE_DATA == 1)
        simulate_cir_input(cir_bytes); // Simular la recepción de 128 bytes
    else if (SOURCE_DATA == 2)
        testing_cir_input(cir_bytes); // Recepción de datos de entrenamiento de 128 bytes
    else if (SOURCE_DATA == 3 && interrupt_processing_enabled)
        receive_cir_from_uwb(cir_bytes); // Recepción de datos del UWB
        
    if (CIRs_PER_SECOND < 4)
        print_raw_bytes(cir_bytes); // Imprimir los bytes recibidos

    extract_and_convert_to_complex(cir_bytes, cir_taps); // Extraer y convertir a formato complejo
    
    if (CIRs_PER_SECOND < 4)
        print_complex_cir(cir_taps); // Imprimir los taps complejos
}


static int cirs_since_last_copy = 0;

void handle_buffering(complex_float_t *cir_taps) {
    if (!acquisition_active || !algorithm_active) {
        //printf("\t[BUFFERING] Acquisition or algorithm not active, skipping buffering.\n");
        return; // Si no está activo, no se procesa
    }
    // Encolar los datos en el buffer circular
    enqueueCIR(&circ_buffer, cir_taps);
    cirs_since_last_copy++;
    total_cirs++;
    
    if (CIRs_PER_SECOND < 4)
        printCircularBuffer(&circ_buffer);

    TickType_t now_ac = xTaskGetTickCount();
    TickType_t elapsed_ac = now_ac - start_acquisition_time;
    //printf("Start time: %lu s\n", start_acquisition_time/ configTICK_RATE_HZ);
    //printf("Elapsed time: %lu s\n", elapsed_ac/ configTICK_RATE_HZ);
    //printf("Now time: %lu s\n", now_ac/ configTICK_RATE_HZ);
    printf("\tTime: %lu s\tTotal CIRS: %u \tIndex Sampling Buffer: %u\t Stored CIRs: %u \t Stored Taps: %u\n", elapsed_ac/ configTICK_RATE_HZ, total_cirs, buffer_index, circ_buffer.num_cirs, circ_buffer.num_taps);
    
    // Si el buffer circular tiene al menos 10 segundos de datos (10 CIRs), se copian al buffer lineal
    if(circ_buffer.num_cirs >= BUFFER_LIN_SIZE && cirs_since_last_copy >= CIRs_PER_SECOND){

        copyFromCircBufferToLinearBuffer(&circ_buffer, &lin_buffer);
        if (CIRs_PER_SECOND < 4)
            printLinearBuffer(&lin_buffer);
        
        // Restar el primer CIR a todos
        printf("\t[BUFFERING] Ajustando el buffer lineal restando el primer CIR a todos los CIRs.\n");
        adjustLinearBuffer(&lin_buffer);

        if (CIRs_PER_SECOND < 4)
            printLinearBuffer(&lin_buffer);

        xSemaphoreGive(data_sampling_ready_sem);
        cirs_since_last_copy = 0; // Reiniciar el constador de CIRs
    }
    
    buffer_index = (buffer_index + 1) % BUFFER_CIRC_SIZE;
    
}

#define TOTAL_CIRS_UWB 1500 // Total de CIRs esperados del UWB

void receive_cir_from_uwb(uint8_t *cir_bytes) {
    if(cir_uwb_index < TOTAL_CIRS_UWB) {
        printf("\033[1;35m\n[ACQUISITION] Recibiendo CIR %d...\033[0m\n", cir_uwb_index);
        // Esperar bloqueantemente a que haya un nuevo CIR en la cola
        if (xQueueReceive(cir_queue, cir_bytes, portMAX_DELAY) == pdTRUE) {
            // printf("COMPROBACION\n");
            //print_raw_bytes(cir_bytes); // Imprimir los bytes recibidos
            cir_uwb_index++;
        }
        
    } else {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de reiniciar la secuencia
        printf("End of the CIR sequence for UWB data\n");
        stop_acq_algorithm(); // Detener adquisición si se ha llegado al final de la secuencia
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de reiniciar la simulación
        hard_reset();
    }
}

# define TOTAL_CIRS_SIM (CIRs_PER_SECOND * 60) // Total de CIRs simulados en 1 minuto

/// Simula la recepción de 128 bytes de un CIR como si vinieran por SPI/UART/etc.
void simulate_cir_input(uint8_t *cir_bytes_out) {
    if (cir_simulation_index < TOTAL_CIRS_SIM){ // Simulación de 1 minuto
        cir_simulation_index++;

        // Simulación de 32 taps (128 bytes) con valores aleatorios
        for (int i = 0; i < CIR_TAPS; i++) {
            // Zonas centrales con señal moderada
            int16_t real, imag;
            if (i >= 5 && i <= 15) {
                real = (int16_t)((rand() % 41) - 20);  // [-20, +20]
                imag = (int16_t)((rand() % 41) - 20);
            }
            // Taps fuera del rango útil → valores bajos (ruido)
            else {
                real = (int16_t)((rand() % 11) - 5);   // [-5, +5]
                imag = (int16_t)((rand() % 11) - 5);
            }

            // Parte real → little endian
            cir_bytes_out[i * 4 + 0] = (uint8_t)(real & 0xFF);
            cir_bytes_out[i * 4 + 1] = (uint8_t)((real >> 8) & 0xFF);

            // Parte imag → little endian
            cir_bytes_out[i * 4 + 2] = (uint8_t)(imag & 0xFF);
            cir_bytes_out[i * 4 + 3] = (uint8_t)((imag >> 8) & 0xFF);
        }
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de reiniciar la simulación
        printf("End of CIR simulation\n");
        stop_acq_algorithm(); // Detener adquisición si se ha llegado al final de la simulación
        printControlMenu(); // Imprimir el menú de control
    }
}

#define CIR_BYTES_ENT 128
#define TOTAL_CIRS_ENT 1500

void testing_cir_input(uint8_t *cir_bytes_out) {
    if (cir_sequence_index +1 <= TOTAL_CIRS_ENT) {
        //printf("\n[ACQUISITION] Recibiendo CIR %d...\n", cir_sequence_index);
        memcpy(cir_bytes_out, &cir_input_sequence_EXE_EEE_3[cir_sequence_index], CIR_BYTES_ENT);
        cir_sequence_index ++;
    } else {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de reiniciar la secuencia
        printf("End of the CIR sequence for training data\n");
        stop_acq_algorithm(); // Detener adquisición si se ha llegado al final de la secuencia
        printControlMenu(); // Imprimir el menú de control
    }
}

// Imprime los bytes en hexadecimal
void print_raw_bytes(const uint8_t *buf) {
    printf("Datos recibidos (hex):\n");
    for (int i = 0; i < CIR_BYTES; i++) {
        printf("%02X ", buf[i]);
        if ((i + 1) % 64 == 0) printf("\n");
    }
    printf("\n");
}

// Extrae los componentes I y Q en formato int16_t
void extract_and_convert_to_complex(const uint8_t *cir_bytes, complex_float_t *cir_taps) {
    for (int i = 0; i < CIR_TAPS; i++) {
        int16_t real = (int16_t)(cir_bytes[i * 4 + 1] << 8 | cir_bytes[i * 4 + 0]);
        int16_t imag = (int16_t)(cir_bytes[i * 4 + 3] << 8 | cir_bytes[i * 4 + 2]);
    
        // Almacena los valores en el buffer de taps
        cir_taps[i].real = (float)real;
        cir_taps[i].imag = (float)imag;
    }
}

// Imprime los taps como números complejos
void print_complex_cir(complex_float_t *cir_taps) {
    /*
    printf("Taps complejos (I + jQ):\n");
    for (int i = 0; i < CIR_TAPS; i++) {
        printf("Tap %02d: %4d, %4dj\n", i, cir_taps[i].real, cir_taps[i].imag);
        //printf("Tap %02d: %+4.0f %+4.0fj\n", i, cir_taps[i].real, cir_taps[i].imag);
    }
    */
}


void adjustLinearBuffer(LinearBuffer *lin_buffer) {
    // Copiar el primer CIR a un buffer temporal para usarlo como referencia
    complex_float_t reference_cir[CIR_TAPS];
    for (int j = 0; j < CIR_TAPS; j++) {
        reference_cir[j].real = lin_buffer->data[0][j].real;
        reference_cir[j].imag = lin_buffer->data[0][j].imag;
    }

    // Restar el primer CIR a todos los CIRs del buffer lineal

    for (int i = 0; i < lin_buffer->num_cirs; i++) {
        for (int j = 0; j < CIR_TAPS; j++) {
            lin_buffer->data[i][j].real -= reference_cir[j].real;
            lin_buffer->data[i][j].imag -= reference_cir[j].imag;
        }
    }
}

void printReceivedData(float *recvbuf){
    /*
    printf("\tRecibido: ");
    for(int i = 0; i < CIR_TAPS; i++){
        printf("%.3f ", recvbuf[i]);
    }
    printf("\n");
    */
}
