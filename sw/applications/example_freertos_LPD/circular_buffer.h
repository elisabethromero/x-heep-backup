#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdbool.h>

#define CIRs_PER_SECOND 1   // 1 CIR por segundo
#define CIR_TAPS 32 // 32 TAPs por CIR
#define SECONDS_SAMPLING 11 // 11 segundos de datos

#define BUFFER_CIRC_SIZE (CIRs_PER_SECOND * SECONDS_SAMPLING)  // 275 CIRs (11s de CIRS)

typedef struct {
    float data[BUFFER_CIRC_SIZE][CIR_TAPS];  // 275 CIRs, cada uno con 32 Taps (float[275][32])
    int head;  // Apunta al primer elemento. Índice de escritura
    int tail; // Apunta al último elemento. Índice de lectura

    int num_cirs;       // Nº de CIRs almacenados
    int num_taps;       // Nº de TAPs almacenados
    int acquisition_cycles; // Nº de ciclos de adquisición realizados
} CircularBuffer;

extern CircularBuffer circ_buffer;

// Funciones de gestión del buffer circular
void initializeCircularBuffer(CircularBuffer *buffer);
bool isCircularBufferFull(CircularBuffer *buffer);
bool isCircularBufferEmpty(CircularBuffer *buffer);
int getCircBufferAcquisitionCycles(CircularBuffer *buffer);
bool enqueueCIR(CircularBuffer *buffer, float cir[CIR_TAPS]);
bool dequeueCIR(CircularBuffer *buffer, float cir[CIR_TAPS]);
void printCircularBuffer(CircularBuffer *buffer);

#endif // CIRCULAR_BUFFER_H