#ifndef LINEAR_BUFFER_H
#define LINEAR_BUFFER_H

#include <stdbool.h>
#include <stdint.h>
#include "circular_buffer.h"

#define CIRs_PER_SECOND 1 // 1 CIR por segundo
#define CIR_TAPS 32     // 32 TAPs por CIR
#define SECONDS_DETECTION 10 // 10 segundos de datos

#define BUFFER_LIN_SIZE (CIRs_PER_SECOND * SECONDS_DETECTION)  // 250 CIRs (Últimos 10s de CIRs)

typedef struct {
    float data[BUFFER_LIN_SIZE][CIR_TAPS];  // 250 CIRs, cada uno con 32 Taps
    int num_cirs;       // Nº de CIRs almacenados
    int num_taps;       // Nº de TAPs almacenados

    bool ready; // Indica si el buffer está listo para procesar
} LinearBuffer;

extern LinearBuffer lin_buffer;

// Funciones de gestión del buffer lineal
void initializeLinearBuffer(LinearBuffer *buffer);
bool isLinearBufferFull(LinearBuffer *buffer);
bool isLinearBufferEmpty(LinearBuffer *buffer);
bool getLinearBufferData(LinearBuffer *buffer, float *output, int size);
bool isLinearBufferReady(LinearBuffer *buffer);
void printLinearBuffer(LinearBuffer *buffer);
void copyFromCircBufferToLinearBuffer(CircularBuffer *circ_buffer, LinearBuffer *lin_buffer);
int getLinearBufferNumCirs(LinearBuffer *buffer);

#endif // LINEAR_BUFFER_H