#include "linear_buffer.h"
#include "circular_buffer.h"
#include <string.h>
#include <stdio.h>

// Inicializa el buffer circular
void initializeCircularBuffer(CircularBuffer *buffer) {
    memset(buffer->data, 0, sizeof(buffer->data));
    buffer->head = 0;
    buffer->tail = 0;
    buffer->num_cirs = 0;
    buffer->num_taps = buffer->num_cirs*CIR_TAPS;
    buffer->acquisition_cycles = 0;
}

// Verifica si el buffer está lleno
bool isCircularBufferFull(CircularBuffer *buffer) {
    return buffer->num_cirs == BUFFER_CIRC_SIZE;
}

// Verifica si el buffer está vacío
bool isCircularBufferEmpty(CircularBuffer *buffer) {
    return buffer->num_cirs == 0;
}

// Consultar ciclos de adquisición
int getCircBufferAcquisitionCycles(CircularBuffer *buffer) {
    return buffer->acquisition_cycles;
}

// Añade un CIR al buffer circular (FIFO)
bool enqueueCIR(CircularBuffer *buffer, complex_float_t cir[CIR_TAPS]) {
    // Copiar el CIR al buffer en la posición de la cola (`tail`)
    memcpy(buffer->data[buffer->tail], cir, CIR_TAPS * sizeof(complex_float_t));

    //Avanzar el índice de la cola circularmente
    buffer->tail = (buffer->tail + 1) % BUFFER_CIRC_SIZE;

    // Si el buffer está lleno, mover también la cabeza (`head`)
    if (buffer->num_cirs == BUFFER_CIRC_SIZE) {
        buffer->head = (buffer->head + 1) % BUFFER_CIRC_SIZE;
    } else {
        buffer->num_cirs ++;
        buffer->num_taps += CIR_TAPS; 
    }

    return true; // Siempre se encola el dato
}

// Extrae un CIR del buffer circular (FIFO)
bool dequeueCIR(CircularBuffer *buffer, complex_float_t cir[CIR_TAPS]) {
    if (isCircularBufferEmpty(buffer)) {
        return false;
    }

    memcpy(cir, buffer->data[buffer->head], CIR_TAPS * sizeof(complex_float_t));
    
    buffer->head = (buffer->head + 1) % BUFFER_CIRC_SIZE;
    
    buffer->num_cirs--;
    
    return true;  
}

// Imprime el contenido del buffer circular
void printCircularBuffer(CircularBuffer *buffer) {
    if (isCircularBufferEmpty(buffer)) {
        printf("[CIRCULAR BUFFER] El buffer está vacío.\n");
        return;
    }

    //[EMRL] Se han comentado los prints de los datos para evitar saturar la salida
    
    printf("\n[CIRCULAR BUFFER] Contenido actual (%d CIRs - %d TAPs):\n", buffer->num_cirs, buffer->num_taps);
    int index = buffer->head;
    
    for (int i = 0; i < buffer->num_cirs; i++) {
        printf("CIR #%d: [", (buffer->head + i) % BUFFER_CIRC_SIZE);
        for (int j = 0; j < CIR_TAPS; j++) {
            //[EMRL] Se ha adaptado a enteros ya que no veo imaginarios
            // Imprime real e imaginario solo como enteros
            printf("(%d, %d) ", buffer->data[index][j].real, buffer->data[index][j].imag);
            if (((j + 1) % 8 == 0) && (j != CIR_TAPS - 1)) {
                 printf("\n\t");
            }
        }
        printf("]\n");
        index = (index + 1) % BUFFER_CIRC_SIZE;
    }
    printf("\n");
}
