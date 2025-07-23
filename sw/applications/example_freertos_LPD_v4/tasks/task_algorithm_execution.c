
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "model.h"
#include "linear_buffer.h"
#include "circular_buffer.h"
#include "fft.h"
#include "z_math.h"
#include "task_control.h"
#include "task_data_acquisition.h"
#include "task_algorithm_execution.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "input_data_generated.c" // Datos de entrada generados para pruebas
#include "Config.h"

int results[NUM_EXECUTIONS]; // Array para almacenar resultados de detección (0's y 1's)
int num_results = 0; // Contador de resultados almacenados

// Valores intermedios del preprocesado
complex_float_t time_series[MAX_SELECTED_TAPS][BUFFER_LIN_SIZE];
COMPLEX filtered_signal[MAX_SELECTED_TAPS][BUFFER_LIN_SIZE];
double freq_values[MAX_SELECTED_TAPS][BUFFER_LIN_SIZE/2]; // Array para almacenar las frecuencias de salida
double breath_freqs[MAX_SELECTED_TAPS][TARGET_FREQS]; // Array para almacenar las frecuencias de respiración

// Salida final del preprocesado
double final_freqs[TARGET_FREQS] = {0.0};


// Tarea de ejecución del algoritmo
void task_algorithm_execution(void *pvParameter) {
    while(1){
        if (algorithm_active){
            if(xSemaphoreTake(data_sampling_ready_sem, portMAX_DELAY)) {    
                printf("\n[ALGORITHM] Algorithm execution...\n");

                preprocess(lin_buffer.data, final_freqs);
                run_model(final_freqs);
                postprocess(results);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ====================================================================================================================
// ====================================================================================================================
// ====================================================================================================================

// PREPROCESADO

// Necesitamos pasar el primer parametro por valor para evitar que se sobreescriba el buffer lineal
// y el segundo por referencia para que se modifique el array de frecuencias finales
void preprocess(complex_float_t cirs[][CIR_TAPS],  double *final_freqs) {
    
    int dominant_taps[BUFFER_LIN_SIZE];
    int tap_frequencies[CIR_TAPS] = {0};
    int top_taps[DOMINANT_NUM_TAPS];
    int selected_taps[LOCAL_NUM_TAPS];
    int num_selected = 0;

    // // Reducción de dimensionalidad
    get_dominant_taps(lin_buffer.data, dominant_taps);
    count_tap_frequencies(dominant_taps, tap_frequencies);
    get_top_taps(tap_frequencies, top_taps);
    expand_with_neighbors(top_taps, DOMINANT_NUM_TAPS, &num_selected, selected_taps);   // printf("[Debug] Numero de taps seleccionados preprocess: %d\n", num_selected);
    extract_time_series_of_taps(lin_buffer.data, selected_taps, num_selected, time_series);

    // Aplicación de filtro paso bajo y FFT y obtención de frecuencias de respiración
    butter_lowpass_filter(time_series, num_selected, 1.00, 25, 4, filtered_signal);
    applyFFT(filtered_signal, num_selected, freq_values);
    get_breathing_frequency(freq_values, breath_freqs);

    //Suma de frecuencias de respiración
    sum_of_frequencies(breath_freqs, final_freqs);
}


// PREPROCESSING
// 5) Se copian los taps en el buffer lineal
// 1. Obtener índice del tap con mayor magnitud en cada CIR
void get_dominant_taps(complex_float_t cirs[][CIR_TAPS], int *dominant_taps){
    printf("\t[PREPROCESSING] Searching for dominant taps...\n");
    for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
        float max_magnitude = 0.0f;
        int max_index = 0;

        for (int j = 0; j < CIR_TAPS; j++) {
            float magnitude = sqrtf(cirs[i][j].real * cirs[i][j].real + cirs[i][j].imag * cirs[i][j].imag);
            if (magnitude > max_magnitude) {
                max_magnitude = magnitude;
                max_index = j;
            }
        }

        dominant_taps[i] = max_index;
    }
    if(CIRs_PER_SECOND < 4){
        printf("Índices de los taps dominantes:\n");
        // Imprimir los índices de los taps dominantes
        for(int i = 0; i < BUFFER_LIN_SIZE; i++) {
            if(i == 0)
                printf("[");

            printf("%d", dominant_taps[i]);

            if(i == BUFFER_LIN_SIZE - 1)
                printf("]\n");
            else
                printf(", ");
        }   
    }
}

// 6) Se guarda en un array la lista con los indices de los taps con mayor magnitud de cada cir
// 2. Contar cuántas veces aparece cada índice
void count_tap_frequencies(int *dominant_taps, int *counts_out){
    printf("\t[PREPROCESSING] Counting tap frequencies...\n");
    
    for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
        int index = dominant_taps[i];
        if (index >= 0 && index < CIR_TAPS) {
            counts_out[index]++;
        }
    }

    if(CIRs_PER_SECOND < 4){
        // Imprimir las frecuencias de los taps
        for (int i = 0; i < CIR_TAPS; i++) {
            if (counts_out[i] > 0) {
                printf("Tap %d: %d veces\n", i, counts_out[i]);
            }
        }
    }
}

// 7) Se eligen de todos los indices los "domminant_num_tamps" índices más repetidos ordenados de mayor a menor
// 3. Obtener los taps más frecuentes
void get_top_taps(int *counts, int *top_taps){
    printf("\t[PREPROCESSING] Getting most frequent taps...\n");

    // Crear un array de pares (índice, conteo)
    typedef struct {
        int index;
        int count;
    } TapCount;

    TapCount tap_counts[CIR_TAPS];
    for (int i = 0; i < CIR_TAPS; i++) {
        tap_counts[i].index = i;
        tap_counts[i].count = counts[i];
    }

    // Ordenar los taps por conteo en orden descendente
    for (int i = 0; i < CIR_TAPS - 1; i++) {
        for (int j = i + 1; j < CIR_TAPS; j++) {
            if (tap_counts[i].count < tap_counts[j].count) {
                TapCount temp = tap_counts[i];
                tap_counts[i] = tap_counts[j];
                tap_counts[j] = temp;
            }
        }
    }

    // Seleccionar los "dominant_num_taps" índices más frecuentes
    for (int i = 0; i < DOMINANT_NUM_TAPS; i++) {
        top_taps[i] = tap_counts[i].index;
    }

    if(CIRs_PER_SECOND < 4){
        // Seleccionar los "dominant_num_taps" índices más frecuentes
        for (int i = 0; i < DOMINANT_NUM_TAPS; i++) {
            printf("Tap %d: %d apariciones\n", top_taps[i], tap_counts[i].count);
        }
    }
}

// 8) Para cada índice elegido se calcula su rango de vecinos (vecinos_num-index : vecinos_num+index)
// Y combinando todos los rangos formados, quedarse con el que los abarque a todos
// 4. Expandir con vecinos ±N
void expand_with_neighbors(int *base_taps, int num_base, int *num_selected, int *expanded_out){
    printf("\t[PREPROCESSING] Expanding taps with neighbors...\n");

    int included[CIR_TAPS] = {0}; // Para marcar los índices ya incluidos
    int count = 0;

    for (int i = 0; i < num_base; i++) {
        int center = base_taps[i];
        int start = (center - LOCAL_NUM_TAPS < 0) ? 0 : center - LOCAL_NUM_TAPS;
        int end = (center + LOCAL_NUM_TAPS >= CIR_TAPS) ? CIR_TAPS - 1 : center + LOCAL_NUM_TAPS;

        for (int j = start; j <= end; j++) {
            if (!included[j]) {
                expanded_out[count++] = j;
                included[j] = 1;
            }
        }
    }

    *num_selected = count;
    if(CIRs_PER_SECOND < 4){
        printf("Índices expandidos: ");
        for (int i = 0; i < count; i++) {
            printf("%d ", expanded_out[i]);
        }
        printf("\n");
    }
    //printf("Numero de taps seleccionados: %d\n", count);
}

// 9) Obtener ahora los valores de los taps en cada cir para el rango elegido, formando una serie temporal por cada cir
// Quedandose en cada cir unicamente con dichos taps, superponiendo los taps originales de cada cir en buffer lineal
// 5. Obtener series temporales de los taps seleccionados
void extract_time_series_of_taps(complex_float_t cirs[][CIR_TAPS], int *selected_taps, int num_selected, complex_float_t output[num_selected][BUFFER_LIN_SIZE]){
    
    printf("\t[PREPROCESSING] Extracting time series of taps...\n");

    //printf("[Debug] Numero de taps seleccionados extract: %d\n", num_selected);

    for (int j = 0; j < num_selected; j++) {
        int tap_index = selected_taps[j];

        if (tap_index >= 0 && tap_index < CIR_TAPS) {
        
            for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
                // output[j][i] = mycabs(cirs[i][tap_index]);
                output[j][i].real = cirs[i][tap_index].real;
                output[j][i].imag = cirs[i][tap_index].imag;
            }
        }
    }

    if(CIRs_PER_SECOND < 4){
        // Imprimir las series temporales
        for (int j = 0; j < num_selected; j++) {
            printf("Serie temporal del tap %d: ", selected_taps[j]);
            for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
                printf("(%3d,%3dj) ", output[j][i].real, output[j][i].imag);

            }
            printf("\n");
        }
        printf("\n");
    }
}


// Butterworth filter coefficients calculation (low-pass)
void butter_lowpass(double cutoff, double fs, int order, double* b, double* a) {
    // This is a placeholder. In C, you typically use libraries like CMSIS-DSP or custom code.
    // For a simple first-order filter:
    printf("\t[PREPROCESSING] Calculating Butterworth filter coefficients...\n");
    double nyquist = fs / 2.0;
    double normal_cutoff = cutoff / nyquist;

    // Butter
    double K = tan(M_PI * normal_cutoff);
    double norm = 1.0 / (1.0 + K);
    b[0] = K * norm;
    b[1] = K * norm;
    a[0] = 1.0;
    a[1] = (K - 1.0) * norm;
}

// Filter application
void lfilter (double b, double a, complex_float_t *signal, int signal_length, complex_float_t *filtered_signal) {
   
    printf("\t[PREPROCESSING] Applying filter to signal...\n");
    
    // const float alpha = 0.1f; // Constante de suavizado para el filtro (ajustable)

    // Simple dummy filter: just copy the input to output
    // for (int i = 0; i < signal_length; i++) {
    //     filtered_signal[i].real = alpha * signal[i].real + (1.0f - alpha) * filtered_signal[i - 1].real;
    //     filtered_signal[i].imag = alpha * signal[i].imag + (1.0f - alpha) * filtered_signal[i - 1].imag;
    // }
}

// 10) A cada Serie temporal se le aplica un filtro butterworth (paso bajo) -> se obtiene señal filtrada de cada cir    
// 6. Aplicar filtro paso bajo (dummy por ahora)
void butter_lowpass_filter(complex_float_t signal[][BUFFER_LIN_SIZE], int num_selected, double cutoff, double fs, int order, COMPLEX filtered_signal[][BUFFER_LIN_SIZE]){
    
    printf("\t[PREPROCESSING] Applying Butterworth filter...\n");  //printf("[Debug] Numero de taps seleccionados butter: %d\n", num_selected);

    // Imprimir señal de entrada (DEBUG) 
    // for(int tap = 0; tap < num_selected; tap++) {
    //     printf("Señal original tap %d: ", tap);
    //     for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
    //         printf("(%3d, %3d) ", signal[tap][i].real, signal[tap][i].imag);
    //     }
    //     printf("\n");
    // }
    // printf("\n"); 

    // ===============================================

    double b[2], a[2];

    // Calculando los coeficientes del filtro Butterworth
    butter_lowpass(cutoff, fs, order, b, a); 

    const float alpha = 0.1f; // Constante de suavizado para el filtro (ajustable)

    // Aplicando el filtro a cada serie temporal
    // for (int tap = 0; tap < num_selected; tap++) {
    //     filtered_signal[tap][0].real = signal[tap][0].real;
    //     filtered_signal[tap][0].imag = signal[tap][0].imag;

    //     lfilter(b[0], a[0], signal[tap], BUFFER_LIN_SIZE, filtered_signal[tap]);

    // }


    for (int tap = 0; tap < num_selected; tap++) {
        filtered_signal[tap][0].real = signal[tap][0].real;
        filtered_signal[tap][0].imag = signal[tap][0].imag;

        for (int i = 1; i < BUFFER_LIN_SIZE; i++) {
            filtered_signal[tap][i].real = alpha * signal[tap][i].real + (1.0f - alpha) * filtered_signal[tap][i - 1].real;
            filtered_signal[tap][i].imag = alpha * signal[tap][i].imag + (1.0f - alpha) * filtered_signal[tap][i - 1].imag;
        }
    }

    // ===============================================

    // Imprimir las señales filtradas (DEBUG)
    // for (int tap = 0; tap < num_selected; tap++) {
    //     printf("Señal filtrada tap %d: ", tap);
    //     for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
    //         printf("(%3d, %3d) ", filtered_signal[tap][i].real, filtered_signal[tap][i].imag);
    //     }
    //     printf("\n");
    // }
    // printf("\n");

    printf("\t[PREPROCESSING] Butterworth filter applied to all time series.\n");
}


// 11) Se aplica la FFT a cada serie temporal filtrada -> se obtiene espectro de cada cir
// 7. Aplicar FFT a cada serie temporal (frecuencia de respiración)

void applyFFT(COMPLEX filtered_signals[][BUFFER_LIN_SIZE], int num_selected, double freq_values[][BUFFER_LIN_SIZE/2]) {
    //COMPLEX signal_complex[BUFFER_LIN_SIZE]; // Buffer temporal para la FFT

    printf("\t[PREPROCESSING] Applying FFT to all filtered signals...\n");

    //printf("Numero de taps seleccionados fft: %d\n", num_selected);

    for (int i= 0; i < num_selected; i++) {
            
            // Aplicar la FFT a cada serie temporal filtrada
            //fft(filtered_signals[i], BUFFER_LIN_SIZE);

            // // Imprimir la FFT resultante
            // // if(CIRs_PER_SECOND < 4){
            //     printf("FFT #%d: ", i);
            //     for (int j = 0; j < BUFFER_LIN_SIZE; j++) {
            //         printf("(%.2f, %.2f) ", signal_complex[j].real, signal_complex[j].imag);
            //     }
            //     printf("\n");
            // // }
            
            // // Almacenar la magnitud de la FFT en el array de frecuencias
            for (int j = 0; j < BUFFER_LIN_SIZE/2; j++) {
                freq_values[i][j] = my_cabs(filtered_signals[i][j]); // Magnitud del valor complejo
            }
    }

    // Imprimir las frecuencias obtenidas (DEBUG)
    // for (int tap = 0; tap < num_selected; tap++) {
    //     for(int j = 0; j < BUFFER_LIN_SIZE/2; j++) {
    //         // printf("Frecuencia %d: ", i);
    //         printf("%.2f ", freq_values[tap][j]);
    //     }
    //     printf("\n");
    // }
}

// 12) Se forma en cada espectro un array con las frecuencias de cada tap de valores: [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] (9 valores )

#define SAMPLING_RATE 25.0 // Frecuencia de muestreo en Hz

void get_breathing_frequency(double freq_values[][BUFFER_LIN_SIZE/2], double breath_freqs[][TARGET_FREQS]){
    // for (int i = 0; i < BUFFER_LIN_SIZE/2; i++) {
        //     freq_values[i][TARGET_FREQS] = sqrt(filtered_signal[j][i].real * filtered_signal[j][i].real + filtered_signal[j][i].imag * filtered_signal[j][i].imag);
        // }
    printf("\t[PREPROCESSING] Extracting breathing frequencies...\n");
    
    // Frecuencias objetivo
    double target_frequencies[TARGET_FREQS] = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    double frequency_resolution = SAMPLING_RATE / BUFFER_LIN_SIZE; // Resolución de frecuencia

    for (int tap = 0; tap < MAX_SELECTED_TAPS; tap++) {
        for (int target_idx = 0; target_idx < TARGET_FREQS; target_idx++) {
            double target_frequency = target_frequencies[target_idx];
            int closest_bin = (int)(target_frequency / frequency_resolution); // Índice del bin más cercano
            breath_freqs[tap][target_idx] = freq_values[tap][closest_bin]; // Asignar magnitud de frecuencia
        }
    }

    // Imprimir las frecuencias de respiración obtenidas (DEBUG)
    // if (CIRs_PER_SECOND < 4) {
    //     for (int tap = 0; tap < MAX_SELECTED_TAPS; tap++) {
    //         printf("Frecuencias de respiración para tap %d: ", tap);
    //         for (int target_idx = 0; target_idx < TARGET_FREQS; target_idx++) {
    //             printf("%.2f ", breath_freqs[tap][target_idx]);
    //         }
    //         printf("\n");
    //     }
    // }
}

// 13) Se suman las magnitudes de las frecuencias similares de todos los cirs
// Formándose un solo espectro con 9 frecuencias, que es la entrada al algoritmo de detección

float frecuencia_objetivo[TARGET_FREQS] = {0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};

float signal_frequency_sum[TARGET_FREQS] = {
    23185.29, 0, 19591.76, 6089.71, 0, 0, 3194.85, 2670.26, 2739.70
};

int input_freq_index = 0; // Índice de la frecuencia de entrada actual


void sum_of_frequencies(double breath_freqs[][TARGET_FREQS], double final_freqs[TARGET_FREQS]) {
    printf("\n\t[PREPROCESSING] Sumando frecuencias...\n");

    // Inicializar el array de salida para las frecuencias
    for (int i = 0; i < TARGET_FREQS; i++) {
        final_freqs[i] = 0.0f;
    }

    //
    // // Sumar las frecuencias de cada tap al array de salida
    // for (int i = 0; i < TARGET_FREQS; i++) {
    //     for(int j = 0; j < BUFFER_LIN_SIZE/2; j++) {     
    //         final_freqs[i] += breath_freqs[j][i];
    //     }
    // }

    // // Imprimir las frecuencias resultantes
    // printf("\tFinal frequencies input to the model: ");
    // for (int i = 0; i < TARGET_FREQS; i++) {
    //     printf("%.2f ", final_freqs[i]);
    // }
    // printf("\n");

    // Imprimir input_data para depuración (DEBUG)
    // printf("\n\t[PREPROCESSING] Input data for frequencies:\n");
    // for (int i = 0; i < TARGET_FREQS; i++) {
    //     printf("%lf ", input_data[input_freq_index][i]);
    // }

        
    // OPCION B (FRECUENCIAS DE ENTRENAMIENTO)
    // Usamos una fila del dataset como entrada
    memcpy(final_freqs, input_data[input_freq_index], sizeof(double) * TARGET_FREQS);

    // Imprimir las frecuencias resultantes
    printf("\tFinal frequencies input to the model: ");
    for (int i = 0; i < TARGET_FREQS; i++) {
        printf("%.10f ", final_freqs[i]);
    }

    // Incrementar el índice para la siguiente iteración
    if (input_freq_index < NUM_EXECUTIONS-1)
        input_freq_index = (input_freq_index + 1);
        
    // Liberar el semáforo para indicar que los datos de la FFT están listos
    xSemaphoreGive(data_fft_ready_sem);
}


// ====================================================================================================================
// ====================================================================================================================
// ====================================================================================================================

// EJECUCIÓN DEL ALGORITMO

// ALGORITHM EXECUTION
// 14) Se aplica el algoritmo de detección a la señal de salida de la FFT -> se obtiene el resultado de detección

// Número de salidas del modelo (presencia/no presencia)
#define MODEL_OUTPUT_SIZE 2  

// Probabilidades de presencia/no presencia  
double model_output[MODEL_OUTPUT_SIZE]; 
        
void run_model(double model_input[TARGET_FREQS]) {

    if(xSemaphoreTake(data_fft_ready_sem, portMAX_DELAY)){

        printf("\n\t[MODEL] Running inference (sample %d)...\n", input_freq_index);

        printf("\t[MODEL] Model input: \n\t");

        // Imprimir los valores de entrada
        for (int i = 0; i < TARGET_FREQS; i++) {
            printf("%.10lf ", model_input[i]);
        }

        // Realizar la inferencia con el modelo LGBM usando float
        // score(signal_frequency_sum, model_output);
        score(model_input, model_output);
        
        // Print the inference result
        printf("\n\t[MODEL] Model output: \n");
        for (int i = 0; i < MODEL_OUTPUT_SIZE; i++) {
            printf("\tClass %d: %.5f\n", i, model_output[i]);
        }
    }
}

// ============================================================================================================_
// ============================================================================================================
// ============================================================================================================

// POSTPROCESSING

// 15) Se aplica postprocesado a la señal de salida del algoritmo de detección -> se obtiene el resultado final (presencia o ausencia)

void postprocess(int *results) {
    int presence_detected = 0; // Variable para indicar si se detecta presencia
    printf("\n\t[POSTPROCESSING] Procesando resultados del modelo...\n");

    // Evaluar el resultado, si la probabilidad de presencia es mayor al 90%, se considera que hay presencia
    if (model_output[1] > 0.9f) {
        printf("\n\t\033[1;31m[DETECCIÓN] Presencia detectada con probabilidad %.2f%%.\033[0m\n\n", model_output[1] * 100);
        presence_detected = 1; // Se detecta presencia
    } else {
        printf("\n\t\033[1;32m[DETECCIÓN] No se detecta presencia (probabilidad %.2f%%).\033[0m\n\n", model_output[0] * 100);
        presence_detected = 0; // No se detecta presencia
    }

    // Ir guardando resultados de la deteccción en array de tamñaño 50 (0's y 1's)
    results[num_results] = presence_detected; // Guardar el resultado actual
    num_results++; // Incrementar el contador de resultados
}