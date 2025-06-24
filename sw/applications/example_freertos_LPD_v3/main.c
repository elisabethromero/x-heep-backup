/* c stdlib */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* ERTIS LPD related includes */
#include "linear_buffer.h"
#include "circular_buffer.h"
#include "spi_interface.h"

/* Librerías de X-HEEP */
#include "csr.h"
#include "hart.h"
#include "handler.h"
#include "core_v_mini_mcu.h"
#include "rv_timer.h"
#include "soc_ctrl.h"
#include "gpio.h"
#include "x-heep.h"
#include "fast_intr_ctrl.h"
#include "spi_host.h"
#include "spi_sdk.h"

/* Liberías de otros archivos incorporados */
#include <math.h>
#include "model.h"
#include "uwb_api.h"
#include "uwb_commands.h"
#include "uwb_core.h"
#include "fft.h"
#include "z_math.h"
#include "Config.h"
#include "radar_config.h"
#include "radar_config_json.h"
#include "cir_input_sequence_EXE_EEE_3.h"
#include "input_data_generated.h"

//[EMRL]
/****************************************************************************/
/**                                                                        **/
/*                            VARIABLES IRVI                                */
/**                                                                        **/
/****************************************************************************/
//[EMRL] Pin de sincronización del bus SPI
uint8_t synq;
//[EMRL] Chip select
uint32_t csid = 0; 
//[EMRL] Definiciones para el bucle
uint16_t i;

//[EMRL]
/****************************************************************************/
/**                                                                        **/
/*                            VARIABLES ERTIS                               */
/**                                                                        **/
/****************************************************************************/

// Definición de semáforos y cola
SemaphoreHandle_t data_sampling_ready_sem = NULL;
SemaphoreHandle_t data_fft_ready_sem = NULL;
QueueHandle_t commandQueue = NULL;
QueueHandle_t commandQueue_UWB = NULL;
QueueHandle_t cir_queue = NULL; // Cola para almacenar CIRs

QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t int_evt_queue = NULL;
QueueHandle_t rdy_evt_queue = NULL;

// Definición de Buffers
CircularBuffer circ_buffer;
LinearBuffer lin_buffer;

TickType_t start_time;
TickType_t start_acquisition_time;

// Creación de handlers
TaskHandle_t control_task_handle = NULL;
TaskHandle_t acquisition_task_handle = NULL;
TaskHandle_t algorithm_task_handle = NULL;
TaskHandle_t uwb_task_handle = NULL;

// Tiempo de muestreo
unsigned buffer_index = 0;
unsigned total_cirs = 0; // Contador de CIRs totales recibidos
int cir_sequence_index = 0;
int cir_simulation_index = 0;

//Buffer de recepción
float recvbuf[CIR_TAPS];

// TASK ALGORITHM EXECUTION DEFINITIONS
#define DOMINANT_NUM_TAPS 2
#define LOCAL_NUM_TAPS 3
#define MAX_SELECTED_TAPS (DOMINANT_NUM_TAPS * (2 * LOCAL_NUM_TAPS + 1))


#define TARGET_FREQS 9
#define NUM_EXECUTIONS 50 // Número de ejecuciones del algoritmo
float output_9[TARGET_FREQS] = {0.0f};
int results[NUM_EXECUTIONS]; // Array para almacenar resultados de detección (0's y 1's)
int num_results = 0; // Contador de resultados almacenados


// TASK DATA ACQUISITION DEFINITIONS
#define SOURCE_DATA 3 // 1: CIRs de entrenamiento, 2: CIRs simnulados, 3: CIRs de UWB
#define CIR_BYTES_ENT 128
#define TOTAL_CIRS_ENT 1500
#define CIR_BYTES (CIR_TAPS * 4) // 32 taps * 4 bytes/tap = 128 bytes

complex_float_t cir_taps[CIR_TAPS];

#define SPI_BUFFER_SIZE 256

//Buffer de recepción
uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];

bool acquisition_active = false;  // Controla si el muestreo está activo
bool algorithm_active = false; // Controla si el algoritmo está activo
bool min_data_ready = false;
bool uwb_ready = false; // Indica si el UWB está listo para recibir comandos



//[EMRL]
/****************************************************************************/
/**                                                                        **/
/*                          VARIABLES GLOBALES                              */
/**                                                                        **/
/****************************************************************************/

#define _MAIN_C_SRC

/* Allocate heap to special section. Note that we have no references in the
 * whole program to this variable (since its just here to allocate space in the
 * section for our heap), so when using LTO it will be removed. We force it to
 * stay with the "used" attribute
 */
__attribute__((section(".heap"), used)) uint8_t ucHeap[configTOTAL_HEAP_SIZE];


//[EMRL]
/****************************************************************************/
/**                                                                        **/
/*          FUNCIONES PROPIAS DEL FREERTOS PARA RISC-V                      */
/**                                                                        **/
/****************************************************************************/
/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	printf( "error: application malloc failed\n\r" );
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
	taskENTER_CRITICAL();
	//printf("I\r\n");
	taskEXIT_CRITICAL();
	
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    //vacío
}
/*-----------------------------------------------------------*/

/* Prepare hardware to run the demo. */

static void SetupHardware( void )
{
	/* Init board hardware. */
	system_init();
}

/*-----------------------------------------------------------*/
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, 
    StackType_t **ppxIdleTaskStackBuffer, 
    uint32_t *pulIdleTaskStackSize) {
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
*ppxIdleTaskStackBuffer = uxIdleTaskStack;
*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

#if configUSE_TIMERS == 1
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, 
                                   StackType_t **ppxTimerTaskStackBuffer, 
                                   uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
/*-----------------------------------------------------------*/

/**
 * Board init code. Always call this before anything else.
 */
void system_init(void)
{
}

/*****************************************************************************
*****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                           TASK ALGORITHM                                 */
/**                                                                        **/
/****************************************************************************/
// Tarea de ejecución del algoritmo
void task_algorithm_execution(void *pvParameter) {
    while(1){
        if(xSemaphoreTake(data_sampling_ready_sem, portMAX_DELAY)) {    
            printf("\n[ALGORITHM] Ejecución del algoritmo...\n");
            fflush(stdout);

            preprocess_CIRs(lin_buffer.data, output_9);
            run_model();
            postprocess_results(results);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

complex_float_t time_series[MAX_SELECTED_TAPS][BUFFER_LIN_SIZE];
complex_float_t filtered_signal[MAX_SELECTED_TAPS][BUFFER_LIN_SIZE];
float freq_values[BUFFER_LIN_SIZE/2][TARGET_FREQS]; // Array para almacenar las frecuencias de salida

void preprocess_CIRs(complex_float_t cirs[][CIR_TAPS],  float *output_9) {
    
    int dominant_taps[BUFFER_LIN_SIZE];
    int tap_frequencies[CIR_TAPS] = {0};
    int top_taps[DOMINANT_NUM_TAPS];
    int selected_taps[LOCAL_NUM_TAPS];
    // // float freq_values[TARGET_FREQS] = {0.0f};

    // // Reducción de dimensionalidad
    get_dominant_taps(lin_buffer.data, dominant_taps);
    count_tap_frequencies(dominant_taps, tap_frequencies);
    get_top_taps(tap_frequencies, top_taps);
    int num_selected = expand_with_neighbors(top_taps, DOMINANT_NUM_TAPS, selected_taps);
    //printf("[Debug] Numero de taps seleccionados preprocess: %d\n", num_selected);
    extract_time_series_of_taps(lin_buffer.data, selected_taps, num_selected, time_series);

    // // Filtro paso bajo, aplicación de la FFT a las series temporales y suma de contribuciones de cada tap para obtener el espectro final
    butterworth_filter(time_series, num_selected, filtered_signal);
    applyFFT(filtered_signal, num_selected, freq_values);
    sumar_frecuencias(output_9, freq_values);
}


// PREPROCESSING
// 5) Se copian los taps en el buffer lineal
// 1. Obtener índice del tap con mayor magnitud en cada CIR
void get_dominant_taps(complex_float_t cirs[][CIR_TAPS], int *dominant_taps){
    //printf("\t[PREPROCESSING] Buscando taps dominantes...\n");
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
        /*
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
        */ 
    }
}

// 6) Se guarda en un array la lista con los indices de los taps con mayor magnitud de cada cir
// 2. Contar cuántas veces aparece cada índice
void count_tap_frequencies(int *dominant_taps, int *counts_out){
    //printf("\t[PREPROCESSING] Contando frecuencias de taps...\n");
    
    for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
        int index = dominant_taps[i];
        if (index >= 0 && index < CIR_TAPS) {
            counts_out[index]++;
        }
    }
    /*
    if(CIRs_PER_SECOND < 4){
        // Imprimir las frecuencias de los taps
        for (int i = 0; i < CIR_TAPS; i++) {
            if (counts_out[i] > 0) {
                printf("Tap %d: %d veces\n", i, counts_out[i]);
            }
        }
    }
    */
}

// 7) Se eligen de todos los indices los "domminant_num_tamps" índices más repetidos ordenados de mayor a menor
// 3. Obtener los taps más frecuentes
void get_top_taps(int *counts, int *top_taps){
    //printf("\t[PREPROCESSING] Obteniendo taps más frecuentes...\n");

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
    if(CIRs_PER_SECOND < 4){
        // Seleccionar los "dominant_num_taps" índices más frecuentes
        for (int i = 0; i < DOMINANT_NUM_TAPS; i++) {
            top_taps[i] = tap_counts[i].index;
            //printf("Tap %d: %d apariciones\n", top_taps[i], tap_counts[i].count);
        }
    }
}

// 8) Para cada índice elegido se calcula su rango de vecinos (vecinos_num-index : vecinos_num+index)
// Y combinando todos los rangos formados, quedarse con el que los abarque a todos
// 4. Expandir con vecinos ±N
int expand_with_neighbors(int *base_taps, int num_base, int *expanded_out){
    //printf("\t[PREPROCESSING] Expandiendo taps con vecinos...\n");

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
    /*
    if(CIRs_PER_SECOND < 4){
        printf("Índices expandidos: ");
        for (int i = 0; i < count; i++) {
            printf("%d ", expanded_out[i]);
            fflush(stdout);
        }
        printf("\n");
    }
    */
    //printf("Numero de taps seleccionados: %d\n", count);
    return count;
}

// 9) Obtener ahora los valores de los taps en cada cir para el rango elegido, formando una serie temporal por cada cir
// Quedandose en cada cir unicamente con dichos taps, superponiendo los taps originales de cada cir en buffer lineal
// 5. Obtener series temporales de los taps seleccionados
void extract_time_series_of_taps(complex_float_t cirs[][CIR_TAPS], int *selected_taps, int num_selected, complex_float_t output[num_selected][BUFFER_LIN_SIZE]){
    
    //printf("\t[PREPROCESSING] Extrayendo series temporales de taps...\n");

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
    /*
    if(CIRs_PER_SECOND < 4){
        // Imprimir las series temporales
        for (int j = 0; j < num_selected; j++) {
            printf("Serie temporal del tap %d: ", selected_taps[j]);
            for (int i = 0; i < BUFFER_LIN_SIZE; i++) {
                printf("(%3d,%3dj) ", output[j][i].real, output[j][i].imag);

            }
            printf("\n\n");
        }
    }
    */
}

void butterworth_filter(complex_float_t signal[][BUFFER_LIN_SIZE], int num_selected, complex_float_t filtered_signal[][BUFFER_LIN_SIZE]){
    //printf("\t[PREPROCESSING] Aplicando filtro Butterworth...\n");
}

void applyFFT(complex_float_t filtered_signals[][BUFFER_LIN_SIZE], int num_selected, float freq_values[][TARGET_FREQS]) {
    COMPLEX signal_complex[BUFFER_LIN_SIZE]; // Buffer temporal para la FFT

    //printf("\n\t[FFT] Aplicando FFT a los CIRs del buffer lineal...\n");

    //printf("Numero de taps seleccionados fft: %d\n", num_selected);
    
    // Liberar el semáforo para indicar que los datos de la FFT están listos
    xSemaphoreGive(data_fft_ready_sem);
}


void sumar_frecuencias(float *output_9, float freq_values[][TARGET_FREQS]) {
    //printf("\n\t[PREPROCESSING] Sumando frecuencias...\n");
    // Inicializar el array de salida para las frecuencias
    for (int i = 0; i < TARGET_FREQS; i++) {
        output_9[i] = 0.0f;
    }

    // Sumar las frecuencias de cada tap al array de salida
    for (int i = 0; i < TARGET_FREQS; i++) {
        for(int j = 0; j < BUFFER_LIN_SIZE/2; j++) {     
            output_9[i] += freq_values[j][i];
        }
    }
    /*
    // Imprimir las frecuencias resultantes
    // if(CIRs_PER_SECOND < 4){
        printf("Frecuencias resultantes: ");
        for (int i = 0; i < TARGET_FREQS; i++) {
            printf("%.2f ", output_9[i]);
        }
        printf("\n");
    // }
    */
}

// Número de salidas del modelo (presencia/no presencia)
#define MODEL_OUTPUT_SIZE 2  
double model_output[MODEL_OUTPUT_SIZE]; 

void print_model_processing() {
    const int barWidth = 20;
    //printf("\n[MODELO] Procesando inferencia...\n\n");
    /*
    for (int i = 0; i <= barWidth; i++) {
        printf("\t\t\033[1;36m"); // Color cyan
        printf("[");
        
        for (int j = 0; j < barWidth; j++) {
            if (j < i) {
                printf("=");
            } else if (j == i) {
                printf(">");
            } else {
                printf(" ");
            }
        }
        
        printf("] %d%%", (i * 100) / barWidth);
        fflush(stdout);
        
        printf("\r"); // Retorno de carro para sobreescribir la línea
    }

    printf("\033[0m\n"); // Resetear color
    */
    printf("\t[MODEL] Inferencia completada.\n\n");
}

// Probabilidades con softmax
float probabilities[MODEL_OUTPUT_SIZE];   


float frecuencia_objetivo[TARGET_FREQS] = {0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};

float signal_frequency_sum[TARGET_FREQS] = {
    23185.29, 0, 19591.76, 6089.71, 0, 0, 3194.85, 2670.26, 2739.70
};

int input_freq_index = 0; // Índice de la frecuencia de entrada actual

void run_model(){
    if(xSemaphoreTake(data_fft_ready_sem, portMAX_DELAY)){

        printf("\n\t[MODEL] Ejecutando inferencia con %d valores (sample %d)...\n", BUFFER_LIN_SIZE* CIR_TAPS, input_freq_index);
        fflush(stdout);

        double *input = input_data[input_freq_index];

        printf("\t[MODEL] Entrada al modelo: \n");
        
        score(input,model_output);
        
        // Incrementar el índice para la siguiente iteración
        if (input_freq_index < NUM_EXECUTIONS)
            input_freq_index = (input_freq_index + 1);
    }
}

void postprocess_results(int *results) {
    int presence_detected = 0; // Variable para indicar si se detecta presencia
    printf("\n\t[POSTPROCESSING] Procesando resultados del modelo...\n");

    // Evaluar el resultado, si la probabilidad de presencia es mayor al 90%, se considera que hay presencia
    if (model_output[1] > 0.9f) {
        int entero1 = (int)(model_output[1] * 100);
        int decimales1 = (int)((model_output[1] * 100 - entero1) * 100);
        if (decimales1 < 0) decimales1 *= -1;
        printf("\n\t\033[1;31m[DETECCIÓN] Presencia detectada con probabilidad %d.%02d%%.\033[0m\n", entero1, decimales1);
        presence_detected = 1; // Se detecta presencia
    } else {
        int entero2 = (int)(model_output[0] * 100);
        int decimales2 = (int)((model_output[0] * 100 - entero2) * 100);
        if (decimales2 < 0) decimales2 *= -1;
        printf("\n\t\033[1;32m[DETECCIÓN] No se detecta presencia (probabilidad %d.%02d%%).\033[0m\n", entero2, decimales2);
        presence_detected = 0; // No se detecta presencia
    }

    // Ir guardando resultados de la deteccción en array de tamñaño 50 (0's y 1's)
    results[num_results] = presence_detected; // Guardar el resultado actual
    num_results++; // Incrementar el contador de resultados
}


/****************************************************************************/
/**                                                                        **/
/*                                UWB  CORE                                 */
/**                                                                        **/
/****************************************************************************/

void uwb_init(void *pvParameters){
    // Comienza la configuración del módulo UWB
    printf("\n>> Starting UWB module configuration...\n");
   
    // Coomprobar notificación de BOOT_STATUS
    wait_ntf_result_t ntf_status = wait_for_notification_or_skip_if_ready(1000);

    if (ntf_status == WAIT_NTF_NONE) {
        printf("\t[UWB BOOT] No se recibió notificación de arranque del UWB.\n");
    } 
   
    uint8_t recv_buf[256] = {0};
    size_t len = 0;
   
    // CASO: hubo notificación pendiente (INT sigue bajo)
    printf("\t[INFO] Procesando posibles notificaciones tras el arranque...\n");
    if (ntf_status == WAIT_NTF_PENDING) {        
        if (!receive_uci_message(recv_buf, sizeof(recv_buf), &len)) {
            printf("[ERROR] INT bajó pero no se pudo leer mensaje.\n");
        }
    }

    radar_config_t cfg;
 
    if (parse_radar_config(json_config_str, &cfg) == 0) {
        printf("[INFO] Configuración del radar cargada correctamente.\n");
    } else {
        printf("[ERROR] No se pudo cargar la configuración del radar.\n");
        return false;
    }

    int interval_cmd = 200;

    // 0. GET_FW_VERSION
    printf("\n0. Getting firmware version...\n");
    uwb_get_version();
    //vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // 1. CMD_SET_BITFIELD: Disable Watchdog
    printf("\n1. Disabling watchdog...\n");
    uwb_disable_watchdog();
    //vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 2. RX Radio Config
    printf("\n2. Configuring RX radio...\n");
    uwb_generic_baseband_config(CFG_RX_RADIO);
    // uwb_configure_rx_radio();
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 3. TX Radio Config
    printf("\n3. Configuring TX radio...\n");
    uwb_generic_baseband_config(CFG_TX_RADIO);
    // uwb_configure_rx_radio();
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 4. TX Power Config
    printf("\n4. Configuring TX power...\n");
    uwb_generic_baseband_config(CFG_TX_POWER);
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 5. CIR Config
    printf("\n5. Configuring CIR...\n");
    uwb_generic_baseband_config(CFG_RX_RADAR_CIR);
    // uwb_configure_radar_cir();
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 6. Noise suppression
    printf("\n6. Configuring noise suppression...\n");
    uwb_generic_baseband_config(CFG_RADAR_NOISE_SUPP);
    // uwb_configure_noise_suppression();
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 7. RX Control
    printf("\n7. Configuring RX radar control...\n");
    uwb_generic_baseband_config(CFG_RX_RADAR_CTRL);
    // uwb_configure_rx_radar_control();
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 8. TX Control
    printf("\n8. Configuring TX radar control...\n");
    uwb_generic_baseband_config(CFG_TX_RADAR_CTRL);
    // uwb_configure_tx_radar_control();
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 9. RADAR_APP_CONFIG → Full calibration
    printf("\n9. Configuring radar application in full calibration mode...\n");
    uwb_configure_radar_application(0x03, 0x28, 0x01);
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 10. START_RADAR → Full calibration
    printf("\n10. Starting radar full calibration...\n");
    uwb_start_radar();
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 11. RADAR_APP_CONFIG → Noise suppression calibration
    printf("\n11. Configuring radar application in noise suppression calibration mode...\n");
    uwb_configure_radar_application(0x05, 0x14, 0x01);
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 12. START_RADAR → Noise suppression calibration
    printf("\n12. Starting radar RNS calibration...\n");
    uwb_start_radar();
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 13. GET_BASEBAND_RESULT → AGC Gain RX1
    printf("\n13. Getting AGC Gain RX1...\n");
    uwb_get_baseband_results(ACC_GAIN_RESULTS, RX1);
    // // build_cmd_get_baseband_result(&cmd, RESULT_AGC_GAIN, 0x00);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !receive_and_parse_agc_gain()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 14. GET_BASEBAND_RESULT → AGC Gain RX2
    printf("\n14. Getting AGC Gain RX2...\n");
    uwb_get_baseband_results(ACC_GAIN_RESULTS, RX2);
    // // build_cmd_get_baseband_result(&cmd, RESULT_AGC_GAIN, 0x01);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !receive_and_parse_agc_gain()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);
    
    // // 15. GET_BASEBAND_RESULT → CALIB GAIN RX1
    printf("\n15. Getting CALIB Gain RX1...\n");
    uwb_get_baseband_results(CALIB_GAIN_RESULTS, RX1);
    // // build_cmd_get_baseband_result(&cmd, RESULT_CALIB_GAIN, 0x00);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !receive_and_parse_calib_gain()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 16. GET_BASEBAND_RESULT → CALIB GAIN RX2
    printf("\n16. Getting CALIB Gain RX2...\n");
    uwb_get_baseband_results(CALIB_GAIN_RESULTS, RX1);
    // // build_cmd_get_baseband_result(&cmd, RESULT_CALIB_GAIN, 0x01);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !receive_and_parse_calib_gain()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 17. GET_BASEBAND_RESULT → RX_CALIB_DATA
    printf("\n17. Getting RX calibration data...\n");
    uwb_get_baseband_results(CALIB_DATA, 0x00);
    // // build_cmd_get_baseband_result(&cmd, RESULT_RX_CALIB_DATA, 0x00);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !receive_and_parse_rx_calib_data()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 18. RADAR_APP_CONFIG → Final streaming config
    printf("\n18. Configuring radar application in streaming mode...\n");
    uwb_configure_radar_application(0x00, 0x01, 0x00);
    uwb_configure_radar_application(0x00, 0x20, 0x05);
    // // build_cmd_radar_application_config(&cmd, cfg->streaming_mode);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

    // // 19. START_RADAR → Start streaming
    printf("\n19. Starting radar CIR streaming...\n");
    uwb_start_radar();
    // // build_cmd_start_radar(&cmd);
    // // build_uci_packet(&cmd, packet, &length);
    // // if (!send_uci_command(packet, length) || !wait_for_uci_ok_response()) return false;
    vTaskDelay(interval_cmd / portTICK_PERIOD_MS);

   
    printf("UWB module configuration completed successfully.\n");

    print_gpio_states();

    return true;
}

// FUNCIÓN PARA ENVIAR Y RECIBIR UN COMANDO UCI
bool send_uci_cmd_get_rsp(uint8_t *cmd_buffer, size_t cmd_len) {
    
    // =============================================================
    // BAJADA DE CS Y ESPERA DE RDY
    // =============================================================
    printf("\n\t[WAIT RDY] Bajando CS y esperando a que RDY baje...\n");
   
    // Bajar CS para iniciar la transmisión 
    //printf("\033[0;31m[INTERRUPCIÓN] GPIO_CS BAJA\033[0m\n");
    //gpio_write(GPIO_CS, false); //[EMRL] Ya se baja y se sube solo
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    
    if (!wait_for_gpio_low(GPIO_RDY_IO, RDY_TIMEOUT_MS)) return false;

    // ==============================================================
    // ENVÍO DEL COMANDO UCI
    // ==============================================================
    if (!transmit_uci_command(cmd_buffer, cmd_len)) return false;

    return true;
}

void gpio_monitor_task(void* arg) {
    uint32_t io_num;
    uint8_t recv_buffer[403] = {0};  // Buffer para recibir la respuesta del UWB

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("\n[GPIO MONITOR] Interrupción en GPIO_%ld\n", io_num);
            if (io_num == GPIO_INT_IO) { // Interrupción del pin INT
                size_t len = 0;
                bool level;
                gpio_read(GPIO_INT_IO, level);
                while (level == 0) {
                    // ==============================================================
                    // RECIBIR RESPUESTA DEL UWB
                    printf("\nRecibiendo respuesta del UWB desde ISR...");
                    // ==============================================================
                    if (!receive_uci_message(recv_buffer,sizeof(recv_buffer), &len)) {
                        //gpio_write(GPIO_CS, true);  // Asegurar que se libera CS //[EMRL] Ya se baja y se sube solo
                        printf("\nReceive uci message");
                        break;
                    } 
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
    
               //print_gpio_states();
            }
        }
    }
}

void print_gpio_states(){
    bool level;
    printf("\033[0;32m\t\tChecando estado de los pines GPIO...\033[0m\n");
    gpio_read(GPIO_INT_IO, &level);
    printf("\033[0;32m\t\tNivel del gpio INT: %d\033[0m\n", level);
    gpio_read(GPIO_RDY_IO, &level);
    printf("\033[0;32m\t\tNivel del gpio RDY: %d\033[0m\n", level);
    gpio_read(GPIO_CS, &level);
    printf("\033[0;32m\t\tNivel del gpio CS: %d\033[0m\n", level);
}


wait_ntf_result_t  wait_for_notification_or_skip_if_ready(uint32_t timeout_ms) {
    printf("\n\t[WAIT NTF BOOT STATUS] Notificación de arranque UWB...\n");

    uint32_t io_num;
    bool level;

    // Si no hay INT pendiente, esperar por él
    //TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    // CASO 1: INT_IO ya está en bajo → notificación pendiente
    gpio_read(GPIO_INT_IO, level);
    if (level == 0) {
        // Ya hay notificación pendiente (INT está bajo)
        printf("\t[WAIT_BOOT] INT ya está en LOW. Notificación pendiente.\n");
        return WAIT_NTF_PENDING;
    }

    // CASO 2: Esperamos a que INT baje durante timeout (notificación activa) por ISR/cola
    if (xQueueReceive(int_evt_queue, &io_num, timeout_ticks)) {
        if (io_num == GPIO_INT_IO) {
            printf("\t[WAIT_BOOT] INT recibido por ISR.\n");
            return WAIT_NTF_PENDING;
        }
    }

    // CASO 3: Timeout y nunca bajó → Comprobamos si hay algo en el buffer por si la notificación ya fue leída
    printf("\t[WAIT BOOT] Timeout esperando INT_IO. No se detectó arranque...\n");

    return WAIT_NTF_NONE;
}



void wait_for_gpio_int(void* arg) {
    uint32_t io_num;
    bool level;
    for (;;) {
        if (xQueueReceive(int_evt_queue, &io_num, portMAX_DELAY)) {
            gpio_read(io_num, level);
            const char* pin_name = (io_num == GPIO_INT_IO) ? "INT" :
                                   (io_num == GPIO_RDY_IO) ? "RDY" : "UNKNOWN";

            printf("\033[0;31m[INTERRUPCIÓN] GPIO_%s (%ld) %s\033[0m\n", pin_name, io_num,
                   level == 0 ? "BAJA" : "SUBE");
        }
    }
}

bool wait_for_gpio_low(gpio_pin_number_t gpio, uint32_t timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    bool level;
    gpio_read(gpio, level);
    if (level == 1) {
    printf("\n\t[WAIT %s] Esperando a que %s para recibir mensaje...\n", gpio == GPIO_INT_IO ? "INT" : "RDY", gpio == GPIO_INT_IO ? "INT" : "RDY");
    }
    
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        if (level == 0) {
            printf("\t[WAIT] %s_GPIO está en LOW.\n", gpio == GPIO_INT_IO ? "INT" : "RDY");
            return true;
        }
        //vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    printf("\tTimeout esperando %s, no se recibió mensaje.\n", gpio == GPIO_INT_IO ? "INT" : "RDY");

    return false;
}

// ====================================================================
// Función para transmitir un comando UCI al UWB
// ====================================================================
bool transmit_uci_command(uint8_t *cmd_buffer, size_t len) {
    printf("\n\t[SEND] Enviando comando UCI...\n");
    
    
    if ((spi_transfer(cmd_buffer, NULL, len, SPI_DIR_TX_ONLY) != SPI_CODE_OK))// Inicializa la transferencia SPI
    {
        printf("Error en transferencia SPI.\n");
        //gpio_write(GPIO_CS, true); //[EMRL] Ya se baja y se sube solo
        success = false;
        return false;
    } else {
        // Subir CS para terminar la transmisión
        //vTaskDelay(5 / portTICK_PERIOD_MS);
        //gpio_write(GPIO_CS, true); //[EMRL] Ya se baja y se sube solo
        printf("\tComando enviado correctamente.\n");
    }
    return true;
}

// ====================================================================
// Función para recibir un mensaje UCI desde el UWB
// ====================================================================
bool receive_uci_message(uint8_t *recv_buffer, size_t max_len, size_t *recv_len){

    printf("\n\t[RECV] Recibiendo respuesta de UWB...\n");
    
    // Bajar CS para iniciar la transmisión
    //gpio_write(GPIO_CS, false); //[EMRL] Ya se baja y se sube solo
    printf("\tBajando CS para recibir mensaje...\n");
    //vTaskDelay(5 / portTICK_PERIOD_MS);

    // Leer cabecera de 4 bytes
    printf("\tLeyendo cabecera y payload length...\n");
    uint8_t header[4];
    if (spi_transfer(NULL, header, 4, SPI_DIR_RX_ONLY) != SPI_CODE_OK) {
        //gpio_write(GPIO_CS, true); //[EMRL] Ya se baja y se sube solo
        return false;
    }

    // Analizar cabecera
    UCIPacketHeader hdr;
    UCIPacketStatus status = analyze_uci_header(header, 4, &hdr);
    if (status == UCI_PACKET_INVALID) {
        printf("\tCabecera inválida. Cancelando recepción.\n");
        //gpio_write(GPIO_CS, true); //[EMRL] Ya se baja y se sube solo
        return false;
    } 
    
    // Calcular longitud total del paquete
    size_t total_len = 4 + hdr.payload_len + 2; // 4 bytes de cabecera + payload + 2 bytes de CRC

    // Copiar cabecera al buffer final
    memcpy(recv_buffer, header, 4);
    uint8_t version_payload[total_len]; 

    memcpy(version_payload, header, 4);

    // Leer el payload y CRC
    printf("\tLeyendo payload y CRC...\n");
    if (spi_transfer(NULL, recv_buffer + 4, hdr.payload_len + 2, SPI_DIR_RX_ONLY) != SPI_CODE_OK) {
        //gpio_write(GPIO_CS, true); //[EMRL] Ya se baja y se sube solo
        return false;
    }
    // Extraer el payload
    uint8_t *payload = recv_buffer + 4;
    size_t payload_len = hdr.payload_len;
    // Procesar el payload según el tipo de mensaje
    
    if (hdr.mt == 0x03 && hdr.gid == 0x00 && hdr.oid == 0x07 && hdr.payload_len == 1) {
        handle_ntf_core_generic(payload, payload_len);
    }
    // Si es un comando de tipo 0x02 (CMD_GENERIC_BASEBAND_CONFIG)
    else if (hdr.mt == 0x02 && hdr.gid == 0x0A) {
        handle_resp_status(payload, payload_len);
    }

    printf("\tMENSAJE RECIBIDO: ");
    for (int i = 0; i < total_len; i++) {
        printf("%02X ", recv_buffer[i]);
    }
    printf("\n");
    
    if(!validate_crc(recv_buffer, total_len)) {
        return false;
    }

    if(hdr.mt == 0x03 && hdr.gid == 0x0A && hdr.oid == 0x31){
        interpret_ntf_radar_result(recv_buffer, total_len);
    }

    // Subir CS para finalizar la recepción
    //gpio_write(GPIO_CS, true); //[EMRL] Ya se baja y se sube solo
    //vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Actualizar el tamaño del buffer de respuesta
    *recv_len = total_len;

    return true;
}

// ====================================================================
// Función para invertir los bytes de un buffer
// ====================================================================
void reverse_bytes(uint8_t *data, size_t length) {
    if (!data || length < 2) return;

    for (size_t i = 0; i < length / 2; i++) {
        uint8_t tmp = data[i];
        data[i] = data[length - 1 - i];
        data[length - 1 - i] = tmp;
    }
}

// =====================================================================
// CRC16 XMODEM
// =====================================================================
bool validate_crc(const uint8_t *buffer, size_t length) {
    if (length < 2) {
        printf("Error: El buffer es demasiado corto para contener un CRC.\n");
        return false;
    }

    // CRC16 (little-endian)
    //uint16_t crc_received = ((uint16_t)rx_buffer[total_len - 2] << 8) | rx_buffer[total_len - 1];
    // Extraer el CRC recibido
    uint16_t crc_received = (buffer[length - 2] << 8) | buffer[length - 1];

    // Calcular el CRC sobre el resto del buffer
    uint16_t crc_calculated = crc16_xmodem(buffer, length - 2);
    // Poner CRC en little-endian
    // uint8_t crc_lsb = crc_calculated & 0xFF;
    // uint8_t crc_msb = (crc_calculated >> 8) & 0xFF;

    reverse_bytes((uint8_t *)&crc_calculated, sizeof(crc_calculated));

    // Comparar los CRCs
    if (crc_received == crc_calculated) {
        printf("\tCRC válido: %04X\n", crc_received);
        return true;
    } else {
        printf("\t[ERROR] CRC inválido. Esperado: %04X, Recibido: %04X\n", crc_calculated, crc_received);
        return false;
    }
}




// =====================================================================
// Análisis de la cabecera UCI
// =====================================================================
UCIPacketHeader *out;

UCIPacketStatus analyze_uci_header(const uint8_t *rx_buffer, size_t len, UCIPacketHeader *out){

    // HEADER_BYTE1  
    out->mt   = (rx_buffer[0] >> 5) & 0x07; // Bits 7-5 del primer byte codifican el tipo de mensaje (MT)
    out->pbf  = (rx_buffer[0] >> 4) & 0x01; // Bit 4
    out->gid  = rx_buffer[0] & 0x0F;        // Bits 3-0 del primer byte codifican el GID

    // HEADER_BYTE2
    out->pldext = (rx_buffer[1] >> 7) & 0x01; // Bit 7
    out->rfu    = (rx_buffer[1] >> 6) & 0x01; // Bit 6
    out-> oid = rx_buffer[1]  & 0x3F; // Bits 5-0

    // PAYLOAD LENGTH (big-endian)
    out->payload_len = ((uint16_t)rx_buffer[2] << 8) | rx_buffer[3];
    
    printf("\t>> MT=0x%02X, GID=0x%02X, OID=0x%02X\n",out->mt, out->gid, out->oid);
    printf("\tTamaño total del paquete: %zu bytes\n", out->payload_len + 4 + 2);
    printf("\tHeader: MT=0x%02X, GID=0x%02X, OID=0x%02X, PBF=%u, PayloadLen=%u, Pldext=%u, RFU=%u\n",
           out->mt, out->gid, out->oid, out->pbf, out->payload_len, out->pldext, out->rfu);

    // Clasificación por tipo de mensajes
    switch (out->mt) {
        case 0x01: // CMD (no deberías recibir esto como respuesta)
            printf("\tRecibido comando (CMD). Probablemente error de interpretación.\n");
            return UCI_PACKET_INVALID;

        case 0x02: // RSP
            switch (out->gid) {
                case 0x0A:
                switch (out->oid) {
                    case 0x00: printf("\033[0;34m\tResponse: RSP_RESET_DEVICE\033[0m\n"); break;
                    case 0x02: printf("\033[0;34m\tResponse: RSP_GET_FW_VERSION\033[0m\n"); break;
                    case 0x04: printf("\033[0;34m\tResponse: RSP_RX_CONFIG\033[0m\n"); break;
                    case 0x05: printf("\033[0;34m\tResponse: RSP_TX_CONFIG\033[0m\n"); break;
                    case 0x06: printf("\033[0;34m\tResponse: RSP_START_BASEBAND\033[0m\n"); break;
                    case 0x07: printf("\033[0;34m\tResponse: RSP_STOP_BASEBAND\033[0m\n"); break;
                    case 0x08: printf("\033[0;34m\tResponse: RSP_GET_BASEBAND_RESULTS\033[0m\n"); break;
                    case 0x09: printf("\033[0;34m\tResponse: RSP_POWER_MGMT_CONFIG\033[0m\n"); break;
                    case 0x0A: printf("\033[0;34m\tResponse: RSP_GENERIC_BASEBAND_CONFIG\033[0m\n"); break;
                    case 0x0B: printf("\033[0;34m\tResponse: RSP_CLOCK_MGMT_CONFIG\033[0m\n"); break;
                    case 0x0C: printf("\033[0;34m\tResponse: RSP_PIN_TOGGLE_CONFIG\033[0m\n"); break;
                    case 0x0D: printf("\033[0;34m\tResponse: RSP_GPADC_OPERATION\033[0m\n"); break;
                    case 0x13: printf("\033[0;34m\tResponse: RSP_SET_BITFIELD\033[0m\n"); break;
                    case 0x14: printf("\033[0;34m\tResponse: RSP_GET_BITFIELD\033[0m\n"); break;
                    case 0x15: printf("\033[0;34m\tResponse: RSP_ECHO_REV_PAYLOAD\033[0m\n"); break;
                    case 0x18: printf("\033[0;34m\tResponse: RSP_RX_RADIO_CONFIG\033[0m\n"); break;
                    case 0x19: printf("\033[0;34m\tResponse: RSP_TX_RADIO_CONFIG\033[0m\n"); break;
                    case 0x20: printf("\033[0;34m\tResponse: RSP_RANGING_PAYLOAD_CONFIG\033[0m\n"); break;
                    case 0x21: printf("\033[0;34m\tResponse: RSP_RANGING_APPLICATION_CONFIG\033[0m\n"); break;
                    case 0x22: printf("\033[0;34m\tResponse: RSP_RANGING_SEQUENCE_CONFIG\033[0m\n"); break;
                    case 0x23: printf("\033[0;34m\tResponse: RSP_START_RANGING\033[0m\n"); break;
                    case 0x24: printf("\033[0;34m\tResponse: RSP_GET_RANGING_PAYLOAD\033[0m\n"); break;
                    case 0x30: printf("\033[0;34m\tResponse: RSP_RADAR_APPLICATION_CONFIG\033[0m\n"); break;
                    case 0x31: printf("\033[0;34m\tResponse: RSP_START_RADAR\033[0m\n"); break;
                    case 0x3E: printf("\033[0;34m\tResponse: RSP_CONFIG_CAN\033[0m\n"); break;
                    case 0x3F: printf("\033[0;34m\tResponse: RSP_CONFIG_UCI\033[0m\n"); break;
                    default: printf("\tRespuesta no reconocida. GID=0x%02X, OID=0x%02X\n", out->gid, out->oid); return UCI_PACKET_INVALID;
                }
                break;
            }
        break;
        case 0x03: // NTF
            switch (out->oid) {
                case 0x01: printf("\033[0;34m\tNotificación: NTF_BOOT_STATUS\033[0m\n"); break;
                case 0x07: printf("\033[0;34m\tNotificación: NTF_CORE_GENERIC_ERROR (Formato incorrecto)\033[0m\n"); break;
                case 0x0B: printf("\033[0;34m\tNotificación: NTF_CLOCK_MGMT\033[0m\n"); break;
                case 0x0D: printf("\033[0;34m\tNotificación: NTF_GPADC_RESULT\033[0m\n"); break;
                case 0x0E: printf("\033[0;34m\tNotificación: NTF_BASEBAND_STATUS\033[0m\n"); break;
                case 0x23: printf("\033[0;34m\tNotificación: NTF_RANGING_STATUS\033[0m\n"); break;
                case 0x31: printf("\033[0;34m\tNotificación: NTF_RADAR_RESULT\033[0m\n"); break;
                default:  printf("\tNotificación no reconocida. GID=0x%02X, OID=0x%02X\n", out->gid, out->oid);  return UCI_PACKET_INVALID;
            }
            break;
        default: printf("\tTipo de mensaje desconocido: MT=0x%02X\n", out->mt); return UCI_PACKET_INVALID;
    }

    

    return UCI_PACKET_OK;  // Fallback de seguridad

}

typedef enum {
    STATUS_OK                   = 0x00,
    STATUS_REJECTED            = 0x01,
    STATUS_FAILED              = 0x02,
    STATUS_SYNTAX_ERROR        = 0x03,
    STATUS_INVALID_PARAM       = 0x04,
    STATUS_INVALID_RANGE       = 0x05,
    STATUS_INVALID_MESSAGE_SIZE= 0x06,
    STATUS_UNKNOWN_GID         = 0x07,
    STATUS_UNKNOWN_OID         = 0x08,
    STATUS_READ_ONLY           = 0x09,
    STATUS_COMMAND_RETRY       = 0x0A,
    STATUS_CRC_ERROR           = 0xF8,
    STATUS_RESERVED            = 0xFF  // RFU o desconocido
} resp_status_t;


void print_resp_status(uint8_t code) {
    switch ((resp_status_t)code) {
        case STATUS_OK: printf("\t\t✓ STATUS_OK: Operación exitosa\n"); break;
        case STATUS_REJECTED:  printf("\t\t✗ STATUS_REJECTED: Operación no permitida en el estado actual\n"); break;
        case STATUS_FAILED: printf("\t\t✗ STATUS_FAILED: La operación falló\n"); break;
        case STATUS_SYNTAX_ERROR: printf("\t\t✗ STATUS_SYNTAX_ERROR: Estructura de paquete inválida\n"); break;
        case STATUS_INVALID_PARAM: printf("\t\t✗ STATUS_INVALID_PARAM: Parámetro válido pero valor incorrecto\n"); break;
        case STATUS_INVALID_RANGE: printf("\t\t✗ STATUS_INVALID_RANGE: Valor fuera de rango\n"); break;
        case STATUS_INVALID_MESSAGE_SIZE: printf("\t\t✗ STATUS_INVALID_MESSAGE_SIZE: Tamaño de mensaje incorrecto\n"); break;
        case STATUS_UNKNOWN_GID: printf("\t\t✗ STATUS_UNKNOWN_GID: GID desconocido\n"); break;
        case STATUS_UNKNOWN_OID: printf("\t\t✗ STATUS_UNKNOWN_OID: OID desconocido\n"); break;
        case STATUS_READ_ONLY: printf("\t\t✗ STATUS_READ_ONLY: Campo de solo lectura\n"); break;
        case STATUS_COMMAND_RETRY: printf("\t\t✗ STATUS_COMMAND_RETRY: Se requiere reintento del comando\n"); break;
        case STATUS_CRC_ERROR: printf("\t\t✗ STATUS_CRC_ERROR: Error de CRC\n"); break;
        default: printf("\t\t✗ Código desconocido o reservado: 0x%02X\n", code); break;
    }
}



void handle_ntf_core_generic(const uint8_t *payload, size_t payload_len) {
    if (!payload || payload_len != 1) {
        printf("[GENERIC_NTF] Payload inválido o tamaño incorrecto (%zu)\n", payload_len);
        return;
    }

    uint8_t status = payload[0];
    printf("\t\t[GENERIC_NTF] RESP_STATUS = 0x%02X\n", status);
    print_resp_status(status);
}

void handle_resp_status(const uint8_t *payload, size_t payload_len) {

    uint8_t status = payload[payload_len - 1];
    if (status == 0x00) {
        printf("\t\t[RESP_STATUS] RESP_STATUS = 0x%02X (OK)\n", status);
        return; // No hay nada más que hacer si es OK
    }
    else{
        printf("\033[0;31m\t\t[RESP_STATUS] RESP_STATUS = 0x%02X (Error)\033[0m\n", status);
        printf("\033[0;31m");
        print_resp_status(status);
        printf("\033[0m");
    }
   
}

typedef enum {
    RX_Success                 = 0x00,
    TX_Success                 = 0x01,
    Calibration_Done           = 0x02,
    Aborted                    = 0x03,
    RX_error                   = 0x10,
    TX_error                   = 0x11,
    General_Baseband_error     = 0x12
} radar_status_t;

void print_radar_status(uint8_t code) {
    switch ((radar_status_t)code) {
        case RX_Success: 
            printf("\t\tRX_SUCCESS_CIR_AVAILABLE: Successful reception and CIR available\n"); 
            break;
        case TX_Success: 
            printf("\t\tTX_Success: Successful transmission\n"); 
            break;
        case Calibration_Done: 
            printf("\t\tCalibration_Done: Calibration completed\n"); 
            break;
        case Aborted: 
            printf("\t\tAborted: Operation aborted\n"); 
            break;
        case RX_error: 
            printf("\t\tRX_error: Reception error\n"); 
            break;
        case TX_error: 
            printf("\t\tTX_error: Transmission error\n"); 
            break;
        case General_Baseband_error: 
            printf("\t\tGeneral_Baseband_error: General baseband error\n"); 
            break;
        default: 
            printf("\t\tUnknown or reserved code: 0x%02X\n", code); 
            break;
    }
}

#define CIR_BYTES (CIR_TAPS * 4) // 32 taps * 4 bytes/tap = 128 bytes


void interpret_ntf_radar_result(const uint8_t *payload, size_t payload_len) {
    if (!payload || payload_len < 1) {
        printf("[RADAR_NTF] Payload inválido o tamaño incorrecto (%zu)\n", payload_len);
        return;
    }
    printf("\t[RADAR_NTF] Interpretando payload y extrayendo datos...\n");
    // Interpretar el primer byte como el estado del radar
    uint8_t radar_status = payload[4];
    printf("\t\t[RADAR_NTF] Radar status: 0x%02X\n", radar_status);
    print_radar_status(radar_status);

    // 
    uint8_t antenna_index = payload[5];
    printf("\t\t[RADAR_NTF] Antenna index: : 0x%02X\n", antenna_index);

    if (payload_len >= 3) {
        uint32_t agc_gain = (payload[6]) | (payload[7] << 8) | (payload[8] << 16) | (payload[9] << 24);
        printf("\t\t[RADAR_NTF] AGC Gain: 0x%08lX (%lu)\n", agc_gain, agc_gain);
    } else {
        printf("\t\t[RADAR_NTF] AGC Gain data not available.\n");
    }

    // Extraer el índice del CIR desde el payload
    if (payload_len >= 10) {
        uint16_t cir_idx = (payload[10] << 8) | payload[11];
        printf("\t\t[RADAR_NTF] CIR Index: 0x%04X\n", cir_idx);
    } else {
        printf("\t\t[RADAR_NTF] CIR Index data not available.\n");
    }

    if (payload_len >= 12) {
        uint32_t cir_offset = (payload[15] << 24) |
                              (payload[14] << 16) |
                              (payload[13] << 8) |
                               payload[12];
        printf("\t\t[RADAR_NTF] CIR Offset: 0x%08lX\n", cir_offset);
    } else {
        printf("\t\t[RADAR_NTF] CIR Offset data not available.\n");
    }

    uint32_t num_taps = 0;
    if (payload_len >= 12) {
        num_taps = (payload[19] << 24) | (payload[18] << 16) |  (payload[17] << 8) |  payload[16];
        printf("\t\t[RADAR_NTF] Number of Taps: 0x%08lX (%lu)\n", num_taps, num_taps);
        // printf("\t\t[RADAR_NTF] Number of Taps: 0x%X\n", num_taps);
    } else {
        printf("\t\t[RADAR_NTF] Number of Taps data not available.\n");
    }

    if (payload_len >= 16) {
        uint32_t timestamp = (payload[23] << 24) |
                             (payload[22] << 16) |
                             (payload[21] << 8) |
                              payload[20];
        printf("\t\t[RADAR_NTF] Timestamp: 0x%08lX (%lu)\n", timestamp, timestamp);
        // printf("\t\t[RADAR_NTF] Timestamp (decimal): \n", timestamp);
    } else {
        printf("\t\t[RADAR_NTF] Timestamp data not available.\n");
    }

    if (payload_len >= 24 + 4 * (unsigned long)num_taps) {
        printf("\t\t[RADAR_NTF] CIR Taps:\n");


        // Copiar los 128 bytes (máximo 32 taps × 4 bytes) directamente desde el payload
        uint8_t cir_bytes[CIR_BYTES] = {0};
        memcpy(cir_bytes, &payload[24], CIR_BYTES);

        // Imprimir por debug:
        for (uint32_t i = 0; i < num_taps; i++) {
            uint32_t tap = (payload[24 + 4 * i] << 24) |
                           (payload[25 + 4 * i] << 16) |
                           (payload[26 + 4 * i] << 8) |
                            payload[27 + 4 * i];

            // printf("\t\t\tTap %lu: 0x%08lX", i, tap);
            // if ((i + 1) % 2 == 0) {
            //     printf("\n");
            // } else {
            //     printf("\t");
            // }
        }
      
        // Enviar directamente por la cola, sin reorganizar nada
        if (xQueueSend(cir_queue, cir_bytes, portMAX_DELAY) != pdTRUE) {
            printf("\n\t\t[ERROR] Cola de CIRs llena, no se pudo enviar.\n");
        } 
    } else {
        printf("\t\t[RADAR_NTF] CIR Taps data not available.\n");
    }
}

/****************************************************************************/
/**                                                                        **/
/*                           TASK CONTROL                                   */
/**                                                                        **/
/****************************************************************************/

void start_acq_algorithm() {
    acquisition_active = true;
    algorithm_active = true;
    printf("\n[CONTROL] Acquisition and algorithm STARTED.\n");
    start_acquisition_time = xTaskGetTickCount();
}

void printResults() {
    if (!num_results){
        printf("\n[ALGORITHM] No se han realizado ejecuciones del algoritmo.\n");
        return;
    }
    else{
        printf("\n[ALGORITHM] Detection results:\n");
        printf("[ ");
        for (int j = 0; j < num_results; j++) {
            printf("%d", results[j]);
            if (j < num_results - 1) {
                printf(", ");
            }
        }
        printf(" ]\n");
    }
}

void stop_acq_algorithm() {
    acquisition_active = false;
    algorithm_active = false;
    printf("\n[CONTROL] Acquisition and algorithm STOPPED.\n");
    printf("[RESET] Resetting buffers...\n");
    // Reiniciar el buffer circular
    initializeCircularBuffer(&circ_buffer);
    // Reiniciar el buffer lineal
    initializeLinearBuffer(&lin_buffer);
    min_data_ready = false;
    buffer_index =0;
    total_cirs = 0; // Reiniciar contador de CIRs totales
    cir_sequence_index = 0; // Reiniciar índice de secuencia de CIRs
    cir_simulation_index = 0; // Reiniciar índice de simulación de CIRs
    // Imprimir resultados de la detección
    printResults();
    num_results = 0; // Reiniciar contador de resultados
}

void printControlMenu() {
    printf("\n=============================================================\n");
    printf("                          CONTROL MENU                       \n");
    printf("=============================================================\n");
    // Monitoring RAM resources
    //printf("Free stack: %" PRIu32 " bytes\n", uxTaskGetStackHighWaterMark(NULL)); // Stack (cantidad de memoria libre)
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - start_time;

    printf("Tiempo transcurrido: %lu segundos\n", elapsed / configTICK_RATE_HZ);
 
    printf("=============================================================\n");
    printf("[s] Start acquisition and algorithm\n"); //
    printf("[p] Stop acquisition and algorithm\n");
    printf("[m] Monitor memory and tasks\n");
    printf("--------------------------------------------\n");
    printf("Enter command: \n");
}

//Tarea de control
void task_control(void *pvParameter) {
    char command;
    
    printControlMenu();    

    while (1) {   
        
        command = getchar(); //Limpiar el buffer de entrada antes de cada lectura

        scanf("%c", &command);
        command = 's'; // Simular comando de inicio para pruebas
        // Enviar comando a la cola
        if(command == 's'  || command == 'p' || command == 'q' || command == 'm') {
            xQueueSend(commandQueue, &command, portMAX_DELAY);
        
            if (xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
                //char command = input[0];

                if ((command == 's') && (!acquisition_active && !algorithm_active)) {
                    start_acq_algorithm();
                
                } else if ((command == 'p') && (acquisition_active || algorithm_active)) {
                    stop_acq_algorithm();
                    printControlMenu();

                } else if ((command == 'm') && (!acquisition_active && !algorithm_active)) {
                    // printMonitorMemoryTasks();
                    printControlMenu();

                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

/****************************************************************************/
/**                                                                        **/
/*                       TASK DATA ACQUISITION                              */
/**                                                                        **/
/****************************************************************************/

// Tarea de adquisición de datos del módulo UWB
void task_data_acquisition(void *pvParameter) {
    while (1) {
        if (SOURCE_DATA != 3){
            if(acquisition_active){
                printf("\n[ACQUISITION] Capturando datos (no UWB)...\n");

                // Lectura de datos de entrada
                receive_data(cir_taps);

                // Procesar los datos recibidos y almacenarlos en el buffer circular 
                handle_buffering(cir_taps);
            }
        }   
        else if (SOURCE_DATA == 3) {
            printf("\n[ACQUISITION] Capturando datos del módulo UWB...\n");

            // Lectura de datos de entrada
            receive_data(cir_taps);

            // Procesar los datos recibidos y almacenarlos en el buffer circular 
            handle_buffering(cir_taps);
        }
        
        TickType_t xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / CIRs_PER_SECOND));
    }
}

void receive_data(complex_float_t *cir_taps) {
    //===========================================================================================================
    
    uint8_t cir_bytes[CIR_BYTES] = {0}; // Buffer para recibir 128 bytes (32 taps * 4 bytes/tap)

    if (SOURCE_DATA == 1)
        testing_cir_input(cir_bytes); // Recepción de datos de entrenamiento de 128 bytes
    else if (SOURCE_DATA == 2)
        simulate_cir_input(cir_bytes); // Simular la recepción de 128 bytes
    else if (SOURCE_DATA == 3)
        receive_cir_from_uwb(cir_bytes); // Recepción de datos del UWB
        
    // if (CIRs_PER_SECOND < 4)
        print_raw_bytes(cir_bytes); // Imprimir los bytes recibidos

    extract_and_convert_to_complex(cir_bytes, cir_taps); // Extraer y convertir a formato complejo
    
    if (CIRs_PER_SECOND < 4)
        print_complex_cir(cir_taps); // Imprimir los taps complejos
}

static int cirs_since_last_copy = 0;

// void handle_buffering(float *recvbuf){
void handle_buffering(complex_float_t *cir_taps) {
    if (SOURCE_DATA != 3) {
        if(!acquisition_active) {
            //printf("\t[BUFFERING] Adquisición no activa, no se procesan datos.\n");
            return;
        }
    }
    else if (SOURCE_DATA == 3) {
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
}


void receive_cir_from_uwb(uint8_t *cir_bytes) {
    // Esperar bloqueantemente a que haya un nuevo CIR en la cola
    if (xQueueReceive(cir_queue, cir_bytes, portMAX_DELAY) == pdTRUE) {
        // printf("COMPROBACION\n");
        // print_raw_bytes(cir_bytes); // Imprimir los bytes recibidos
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
        printf("Fin de la simulación de CIRs\n");
        stop_acq_algorithm(); // Detener adquisición si se ha llegado al final de la simulación
        printControlMenu(); // Imprimir el menú de control
    }
}


void testing_cir_input(uint8_t *cir_bytes_out) {
    if (cir_sequence_index +1 <= TOTAL_CIRS_ENT) {
        //printf("\n[ACQUISITION] Recibiendo CIR %d...\n", cir_sequence_index);
        memcpy(cir_bytes_out, &cir_input_sequence_EXE_EEE_3[cir_sequence_index], CIR_BYTES_ENT);
        cir_sequence_index ++;
    } else {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de reiniciar la secuencia
        //printf("Fin de la secuencia de CIRs\n");
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
    printf("Taps complejos (I + jQ):\n");
    for (int i = 0; i < CIR_TAPS; i++) {
        printf("Tap %02d: %4d, %4dj\n", i, cir_taps[i].real, cir_taps[i].imag);
        //printf("Tap %02d: %+4.0f %+4.0fj\n", i, cir_taps[i].real, cir_taps[i].imag);
    }
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
    printf("\tRecibido: ");
    /*
    for(int i = 0; i < CIR_TAPS; i++){
        printf("%.3f ", recvbuf[i]);
    }
    printf("\n");
    */
}


/****************************************************************************/
/**                                                                        **/
/*                                  SYNC                                    */
/**                                                                        **/
/****************************************************************************/

void sync_init(void) {
    printf("\n>> Initializing semaphores and queues...\n");
    // Inicialización de semáforos para sincronización
    if( (data_sampling_ready_sem = xSemaphoreCreateBinary()) == NULL ) {
        printf("Error: No se pudo crear el semáforo data_sampling_ready_sem.\n");
        while(1);
    }

    if( (data_fft_ready_sem = xSemaphoreCreateBinary()) == NULL ) {
        printf("Error: No se pudo crear el semáforo data_fft_ready_sem.\n");
        while(1);
    }

    // Cola de comandos
    if ((commandQueue = xQueueCreate(5, sizeof(char))) == NULL) {
        printf("Error: No se pudo crear la cola de comandos.\n");
        while(1);
    }

    if ((commandQueue_UWB = xQueueCreate(5, sizeof(char))) == NULL) {
        printf("Error: No se pudo crear la cola de comandos UWB.\n");
        while(1);
    }

    if ((cir_queue = xQueueCreate(10, 128)) == NULL) {
        printf("Error: No se pudo crear la cola de CIRs.\n");
        while(1);
    }

    printf("Semaphores and queues initialized successfully.\n");
    //vTaskDelay(500 / portTICK_PERIOD_MS); [EMRL] Aún no se ha lanzado el scheduler y el programa se queda parado aquí
}

/****************************************************************************/
/**                                                                        **/
/*                                  GPIO                                    */
/**                                                                        **/
/****************************************************************************/

// Manejador de interrupciones
void gpio_isr_handler(void* arg) { 
    uint32_t gpio_num = (uint32_t) arg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio_num == GPIO_INT_IO || gpio_num == GPIO_RDY_IO) {
        // Enviar a la cola general de eventos (RDY, CS, etc.)
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    }

    // Para hacer un cambio de contexto inmediato si es necesario
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Función de inicialización de GPIOs y colas de eventos para interrupciones
void gpio_init(){
    printf("\n>> Initializing GPIO interrupt system...\n");
    
    gpio_result_t gpio_res;
    gpio_cfg_t int_conf = {0};
    gpio_cfg_t rdy_conf = {0};

    // Configurar GPIOs de salida
    int_conf.pin = GPIO_INT_IO;
    int_conf.mode = GpioModeIn;
    int_conf.en_input_sampling = true;
    int_conf.en_intr = true;
    int_conf.intr_type = GpioIntrEdgeRisingFalling;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(int_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_INT_IO);
        return EXIT_FAILURE;
    }

    // Configurar GPIOs de salida
    rdy_conf.pin = GPIO_RDY_IO;
    rdy_conf.mode = GpioModeIn;
    rdy_conf.en_input_sampling = true;
    rdy_conf.en_intr = true;
    rdy_conf.intr_type = GpioIntrEdgeRisingFalling;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(rdy_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_RDY_IO);
        return EXIT_FAILURE;
    }


    // Crear cola de eventos
    if(!gpio_evt_queue) {
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    }
    if(!int_evt_queue) {
        int_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    }

    gpio_assign_irq_handler(GPIO_INT_IO, gpio_isr_handler);
    gpio_assign_irq_handler(GPIO_RDY_IO, gpio_isr_handler);
    //gpio_assign_irq_handler(GPIO_CS, gpio_isr_handler);

    printf("GPIO interrupt system initialized successfully.\n");
    //vTaskDelay(500 / portTICK_PERIOD_MS); [EMRL] Aún no se ha lanzado el scheduler y el programa se queda parado aquí
}

/****************************************************************************/
/**                                                                        **/
/*                                  MAIN                                    */
/**                                                                        **/
/****************************************************************************/

void printSplashScreen(void) {
    printf("\n");
    printf("=============================================================\n");
    printf("                     UWB MONITORING SYSTEM                   \n");
    printf("                     RISCCOM Project - LPD                   \n");
    printf("=============================================================\n\n");
    printf(">> Starting system initialization...\n\n");
}


void printUWBMenu(){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("\n===== UWB CONTROL MENU =====\n");
    printf("0. Reset Device\n");
    printf("1. Disable Watchdog\n");
    printf("2. Get Firmware Version\n");
    printf("3. Configure RX Radio\n");
    printf("4. Configure TX Radio\n");
    printf("5. Start Radar\n");
    printf("6. Start Baseband\n");
    printf("7. Stop Baseband\n");
    printf("8. Get Baseband Results\n");
    printf("9. Read Bitfield\n");
    printf("A. Configure Radar Application\n");
    printf("B. Noise Suppression Calibration\n");
    printf("C. Start Streaming\n");
    printf("Q. Quit\n");
    printf("Select option: ");
}

void task_uwb_menu(void *pvParameters) {

    // uint32_t io_num;
    uint8_t recv_buffer[400] = {0};  // Buffer para recibir la respuesta del UWB
    size_t len = 0;

    char command;

    printUWBMenu();
    
    while (true) {

        //command = getchar();

        //scanf("%c", &command); // Limpiar el buffer de entrada antes de cada lectura
        command = '2'; // Simular comando de inicio para pruebas
        // Enviar comando a la cola
        if(command == '1' || command == '2' || command == '3' || command == '4' ||
           command == '5' || command == '6' || command == '7' || command == '8' ||
           command == '9' || command == 'A' || command == 'B' || command == 'C' ||
           command == 'D' || command == 'a' || command == 'b' || command == 'c' ||
           command == 'd' || command == 'Q' || command == 'q' || command == '0' ) {
            
            // Enviar comando a la cola
            xQueueSend(commandQueue_UWB, &command, portMAX_DELAY);
            // Esperar a que el comando sea procesado
            if (xQueueReceive(commandQueue_UWB, &command, portMAX_DELAY)) {
                switch (command) {
                    case '0': uwb_reset_device(); printUWBMenu();break;
                    case '1': uwb_disable_watchdog(); printUWBMenu();break;
                    case '2': uwb_get_version(); printUWBMenu();break;
                    case '3': uwb_generic_baseband_config(CFG_RX_RADIO); printUWBMenu();break;
                    case '4': uwb_configure_tx_radio(); printUWBMenu();break;
                    case '5': uwb_start_radar(); printUWBMenu();break;
                    case '6': uwb_start_baseband(); printUWBMenu();break;
                    case '7': uwb_stop_baseband(); printUWBMenu();break;
                    case '8': uwb_get_baseband_results(ACC_GAIN_RESULTS, RX1); printUWBMenu();break;
                    // case '6': uwb_configure_noise_suppression(); printUWBMenu();break;
                    // case '7': uwb_configure_rx_radar_control(); printUWBMenu();break;
                    // case '8': uwb_configure_tx_radar_control(); printUWBMenu();break;
                    case '9': uwb_read_bitfield(); printUWBMenu();break;
                        printUWBMenu();
                        break;
                    case 'A': // Fast Calib
                        //uwb_configure_radar_application(); // Cambiar internamente al modo
                        //uwb_start_radar();
                        printUWBMenu();
                        break;
                    case 'B': // Noise Calib
                        uwb_configure_radar_application(0x05, 0x01, 0x01); // Cambiar internamente al modo
                        uwb_start_radar();
                        printUWBMenu();
                        break;
                    case 'C':
                        uwb_configure_radar_application(0x00, 0x1, 0x00); // modo streaming
                        uwb_start_radar();
                        
                        printUWBMenu();
                        break;
                    case 'D': // Lectura de emergencia
                        bool level;
                        gpio_read(GPIO_INT_IO, &level);
                        if (level == 0){
                            while (level == 0) {
                                if (!receive_uci_message(recv_buffer,sizeof(recv_buffer), &len)) {
                                    //gpio_write(GPIO_CS, true);  // Asegurar que se libera CS //[EMRL] Ya se baja y se sube solo
                                    gpio_read(GPIO_INT_IO, &level);
                                    break;
                                } 
                            }
                        }
                        else {
                            printf("No hay mensajes pendientes en el UWB.\n");
                        }
                        break;
                    default:
                        printf("Invalid option\n");
                }
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Pequeña pausa entre comandos
    }
}


// En esta función principal se inicializan los semáforos, las colas y los buffers y se crean las tareas
void main()
{

    start_time = xTaskGetTickCount();  // Tiempo de arranque en ticks

    BaseType_t xReturned;

    printf("FreeRTOS version: %s\n\r", tskKERNEL_VERSION_NUMBER);

    // Inicialización de periféricos y drivers
    sync_init();                    // Semáforos, colas...
    gpio_init();                    // Pines e interrupciones
    spi_initialization();           // SPI

    // Inicializar buffers
    initializeCircularBuffer(&circ_buffer);
    initializeLinearBuffer(&lin_buffer);
    
    xTaskCreate(uwb_init, "UWB Task", 8192, NULL, 2, &uwb_task_handle);
    xTaskCreate(gpio_monitor_task, "gpio_monitor_task", 4096, NULL, 3, NULL);
    //xTaskCreate(task_control, "Control Task", 4096, NULL, 3, &control_task_handle);
    //xTaskCreate(task_data_acquisition, "Data Acquisition", 4096, NULL, 4, &acquisition_task_handle);
    //xTaskCreate(task_algorithm_execution, "Algorithm Execution", 8192, NULL, 4, &algorithm_task_handle);
    
    printf("Free heap after task creation: %d bytes\n", xPortGetFreeHeapSize());

    //uwb_init();         // Módulo UWB
    
    //En FreeRTOS para PYNQ-Z2 se deberá iniciar manualmente el scheduler
    vTaskStartScheduler();

    // Mantener app_main() en loop infinito para evitar que termine
    // No sería necesario en ESP32, ya que:
    // - app_main() es otra tarea más del núcleo
    // - FreeRTOS sigue corriendo (ESP-IDF ya maneja FreeRTOS en segundo plano)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    return; // Nunca se debe llegar aquí
}


