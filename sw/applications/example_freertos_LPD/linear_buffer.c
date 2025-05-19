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
            //printf("%.3f ", buffer->data[i][j]);  // Imprime con 3 decimales
            int entero = (int)buffer->data[i][j];
            int decimales = (int)((buffer->data[i][j] - entero) * 1000); // 3 cifras decimales
            if (decimales < 0) decimales *= -1;
            printf("%d.%03d, ", entero, decimales);
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
        memcpy(lin_buffer->data[i], circ_buffer->data[index], CIR_TAPS * sizeof(float));
        
        // Avanzar el índice circularmente
        index = (index + 1) % BUFFER_CIRC_SIZE;
    }

    lin_buffer->num_cirs = BUFFER_LIN_SIZE;
    lin_buffer->num_taps = lin_buffer->num_cirs * CIR_TAPS;
    lin_buffer->ready = true;

    min_data_ready = true;

    printf("\n\t[BUFFER] Se han copiado los últimos %d CIRs al buffer lineal.\n", BUFFER_LIN_SIZE);
    printf("\tDatos suficientes para ejecución del algoritmo de detección de presencia (10s)...\n");
}

int getLinearBufferNumCirs(LinearBuffer *buffer){
    return buffer->num_cirs;
}