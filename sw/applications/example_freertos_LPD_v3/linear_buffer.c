#include "linear_buffer.h"
#include "circular_buffer.h"
#include <string.h>
#include <stdio.h>

extern volatile bool min_data_ready;

// Inicializa el buffer lineal
void initializeLinearBuffer(LinearBuffer *buffer) {
    memset(buffer->data, 0, sizeof(buffer->data));
    buffer->num_cirs = 0;
    buffer->num_taps = buffer->num_cirs*CIR_TAPS;
    buffer->ready = false;
}

// Verifica si el buffer está lleno
bool isLinearBufferFull(LinearBuffer *buffer) {
    return buffer->num_cirs == BUFFER_LIN_SIZE;
}

// Verifica si el buffer está vacío
bool isLinearBufferEmpty(LinearBuffer *buffer) {
    return buffer->num_cirs == 0;
}

bool isLinearBufferReady(LinearBuffer *buffer) {
    return buffer->ready;
}

// Imprime el contenido del buffer lineal
void printLinearBuffer(LinearBuffer *buffer) {
    if (isLinearBufferEmpty(buffer)) {
        printf("[LINEAR BUFFER] El buffer está vacío.\n");
        return;
    }

    
    printf("\n[LINEAR BUFFER] Contenido actual (%d CIRs - %d TAPs):\n", buffer->num_cirs, buffer->num_taps);
    /*
    for (int i = 0; i < buffer->num_cirs; i++) {
        printf("CIR #%d: [", i);
        for (int j = 0; j < CIR_TAPS; j++) {
            printf("%3d%+3dj ", buffer->data[i][j].real, buffer->data[i][j].imag);
            // printf("%3.0f%+3.0fj ", buffer->data[i][j].real, buffer->data[i][j].imag);
            if ((j + 1) % 16 == 0) {
                printf("\n\t");
            }
        }
        printf("]\n");
    }
    */
    printf("\n");
}

// Copia los CIRs del buffer circular al buffer lineal
void copyFromCircBufferToLinearBuffer(CircularBuffer *circ_buffer, LinearBuffer *lin_buffer) {
    int index = (circ_buffer->tail - BUFFER_LIN_SIZE + BUFFER_CIRC_SIZE) % BUFFER_CIRC_SIZE; // Comenzar desde el índice más antiguo del buffer circular

    for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
        // Copiar cada CIR del buffer circular al buffer lineal
        memcpy(lin_buffer->data[i], circ_buffer->data[index], CIR_TAPS * sizeof(complex_float_t));
        
        // Avanzar el índice circularmente
        index = (index + 1) % BUFFER_CIRC_SIZE;
    }

    lin_buffer->num_cirs = BUFFER_LIN_SIZE;
    lin_buffer->num_taps = lin_buffer->num_cirs * CIR_TAPS;
    lin_buffer->ready = true;

    min_data_ready = true;

    printf("\n\t\033[1;34m[BUFFER] Se han copiado los últimos %d CIRs al buffer lineal.\033[0m\n", BUFFER_LIN_SIZE);
    printf("\tDatos suficientes para ejecución del algoritmo de detección de presencia (10s)...\n");
}

int getLinearBufferNumCirs(LinearBuffer *buffer){
    return buffer->num_cirs;
}