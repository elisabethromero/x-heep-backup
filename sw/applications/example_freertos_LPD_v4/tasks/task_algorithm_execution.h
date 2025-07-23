#ifndef TASK_ALGORITHM_EXECUTION_H
#define TASK_ALGORITHM_EXECUTION_H

#include "Config.h"

#define DOMINANT_NUM_TAPS 2
#define LOCAL_NUM_TAPS 3
#define MAX_SELECTED_TAPS (DOMINANT_NUM_TAPS * (2 * LOCAL_NUM_TAPS + 1))

#define TARGET_FREQS 9

#define NUM_EXECUTIONS 50 // Número de ejecuciones del algoritmo

extern int results[NUM_EXECUTIONS]; // Array para almacenar resultados de detección (0's y 1's)
extern int num_results; // Contador de resultados almacenados

// Tarea de ejecución del algoritmo
void task_algorithm_execution(void *pvParameter);


// Funciones de procesamiento del algoritmo
void preprocess(complex_float_t cirs[][CIR_TAPS],  double *final_freqs);
void run_model(double model_input[TARGET_FREQS]);
void postprocess(int *results);

//void dimensionality_reduction(complex_float_t cirs[][CIR_TAPS], complex_float_t output[][BUFFER_LIN_SIZE]);

// Funciones auxiliares para reducción de dimensionalidad
// 1. Obtener índice del tap con mayor magnitud en cada CIR
void get_dominant_taps(complex_float_t cirs[][CIR_TAPS], int *dominant_taps);

// 2. Contar cuántas veces aparece cada índice
void count_tap_frequencies(int *dominant_taps, int *counts_out);

// 3. Obtener los taps más frecuentes
void get_top_taps(int *counts, int *top_taps);

// 4. Expandir con vecinos ±N
void expand_with_neighbors(int *base_taps, int num_base,  int *num_selected, int *expanded_out);

// 5. Obtener series temporales de los taps seleccionados
void extract_time_series_of_taps(complex_float_t cirs[][CIR_TAPS], int *selected_taps, int num_selected, complex_float_t output[num_selected][BUFFER_LIN_SIZE]);


// Funciones auxiliares para obtener frecuencias finales
// 6. Aplicar filtro paso bajo (dummy por ahora)
void butter_lowpass_filter(complex_float_t signal[][BUFFER_LIN_SIZE], int num_selected, double cutoff, double fs, int order, COMPLEX filtered_signal[][BUFFER_LIN_SIZE]);

void butter_lowpass();

// 7. Aplicar FFT a cada serie temporal (frecuencia de respiración)
// void apply_fft_on_series(float filtered_signals[][BUFFER_LIN_SIZE], int num_series, float fft_output[][FREQ_VECTOR_SIZE]);
void applyFFT(COMPLEX filtered_signals[][BUFFER_LIN_SIZE], int num_selected, double freq_values[][BUFFER_LIN_SIZE/2]);

void get_breathing_frequency(double freq_values[][BUFFER_LIN_SIZE/2], double breath_freqs[][TARGET_FREQS]);

void sum_of_frequencies(double breath_freqs[][TARGET_FREQS], double final_freqs[TARGET_FREQS]);

#endif // TASK_ALGORITHM_EXECUTION_H
