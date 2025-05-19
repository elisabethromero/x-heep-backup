/* c stdlib */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* FreeRTOS kernel includes */
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>
#include <queue.h>

/* ERTIS LPD related includes */
#include "linear_buffer.h"
#include "circular_buffer.h"
#include "task_control.h"
#include "task_data_acquisition.h"
#include "task_algorithm_execution.h"
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
#include "spi_slave_sdk.h"

/* Liberías de otros archivos incorporados */
#include <math.h>
#include "model.h"
#include "uwb_api.h"
#include "fft.h"
#include "z_math.h"
#include "Config.h"

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
QueueHandle_t commandQueue;

// Definición de Buffers
CircularBuffer circ_buffer;
LinearBuffer lin_buffer;

TickType_t start_time;
TickType_t start_acquisition_time;

// Variables globales compartidas entre tareas
extern SemaphoreHandle_t data_sampling_ready_sem;  // Semáforo para sincronización con otras tareas
extern SemaphoreHandle_t data_fft_ready_sem;  // Semáforo para sincronización con otras tareas

SemaphoreHandle_t data_sampling_ready_sem = NULL;
SemaphoreHandle_t data_fft_ready_sem = NULL;

// Creación de handlers
TaskHandle_t control_task_handle = NULL;
TaskHandle_t acquisition_task_handle = NULL;
TaskHandle_t algorithm_task_handle = NULL;

// Tiempo de muestreo
unsigned t = 0;
unsigned buffer_index = 0;

// Variables de control
volatile bool acquisition_active = false;  // Controla si el muestreo está activo
volatile bool algorithm_active = false; // Controla si el algoritmo está activo
volatile bool min_data_ready = false;

//#define SPI_BUFFER_SIZE 300
//uint8_t spi_rx_buffer[SPI_BUFFER_SIZE];
// Buffer from where we will ask the SPI slave to read from. 
uint8_t spi_rx_buffer[SPI_BUFFER_SIZE] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 
    91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 
    121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 
    151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 
    181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200
};

//Buffer de recepción
float recvbuf[CIR_TAPS];

// Definiciones de pines
#define GPIO_OUTPUT_IO    18
#define GPIO_INPUT_IO     4
#define GPIO_OUTPUT_PIN_SEL  (1ULL << GPIO_OUTPUT_IO) 
#define GPIO_INPUT_PIN_SEL   (1ULL << GPIO_INPUT_IO) 
#define ESP_INTR_FLAG_DEFAULT 0

// Cola de eventos para las interrupciones GPIO
extern QueueHandle_t gpio_evt_queue;

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

/* Timer 0 AO Domain as Tick Counter */
static rv_timer_t timer_0_1;


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

    // Get current Frequency
    soc_ctrl_t soc_ctrl;
    soc_ctrl.base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS);
    uint32_t freq_hz = soc_ctrl_get_frequency(&soc_ctrl);

    // Setup rv_timer_0_1
    mmio_region_t timer_0_1_reg = mmio_region_from_addr(RV_TIMER_AO_START_ADDRESS);
    rv_timer_init(timer_0_1_reg, (rv_timer_config_t){.hart_count = 2, .comparator_count = 1}, &timer_0_1);
	
	// Just in case you are playing with Tick freq.
    //rv_timer_approximate_tick_params(freq_hz, kTickFreqHz, &tick_params);

    // Enable interrupt on processor side
    // Enable global interrupt for machine-level interrupts
    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);

    // Enable timer interrupt
    uint32_t mask = 1 << 7;
    CSR_SET_BITS(CSR_REG_MIE, mask);

    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
	configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);

    // Configure a GPIO to use to synq both FPGAs.
    // GPIOs by default are set high. 
    // If one of the devices detects it is the master, it 
    // will lower the GPIO. The other, if it finds the 
    // GPIO lowered will know it should be a slave.
    gpio_cfg_t pin_synq = {
        .pin = GPIO_SYNQ,
        .mode = GpioModeIn,
        .en_input_sampling = true,
        .en_intr = false,
        };

    gpio_config(pin_synq);

    gpio_result_t gpio_res;
    gpio_cfg_t pin_cfg = {
        .pin= GPIO_LD5_R, 
        .mode= GpioModeOutPushPull
    };
    gpio_res = gpio_config(pin_cfg);
    pin_cfg.pin = GPIO_LD5_B;
    gpio_res |= gpio_config(pin_cfg);
    pin_cfg.pin = GPIO_LD5_G;
	gpio_res |= gpio_config(pin_cfg);
    if (gpio_res != GpioOk) printf("Failed\n;");
    
    gpio_write(GPIO_LD5_R, false);
    gpio_write(GPIO_LD5_B, false);
    gpio_write(GPIO_LD5_G, false);

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
        if (algorithm_active){
            if(xSemaphoreTake(data_sampling_ready_sem, portMAX_DELAY)) {    
                printf("\n[ALGORITHM] Ejecución del algoritmo...\n\r");
                fflush(stdout);

                preprocess_CIRs();
                run_model();
                postprocess_results();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void applyFFTToLinearBuffer() {
    COMPLEX cir_complex[CIR_TAPS]; // Buffer temporal para la FFT

    if (!isLinearBufferReady(&lin_buffer)) {
        printf("[FFT] Line buffer not ready.\n");
        return;
    }
    else {
        for (int i = 0; i < getLinearBufferNumCirs(&lin_buffer); i++) {
            // Convertir el CIR a formato COMPLEX para la FFT real
            for (int j = 0; j < CIR_TAPS; j++) {
                cir_complex[j].real = lin_buffer.data[i][j]; // Asigna el valor real
                cir_complex[j].imag = 0.0f;                   // Parte imaginaria a 0
            }

            // Aplicar la FFT real al CIR actual
            fft_real(cir_complex, CIR_TAPS);

            // Almacenar la magnitud de la FFT en el buffer lineal
            for (int j = 0; j < CIR_TAPS; j++) {
                lin_buffer.data[i][j] = my_cabs(cir_complex[j]); // Magnitud del valor complejo
            }

            printf("[FFT] FFT aplicada al CIR #%d.\n", i);
        }
        xSemaphoreGive(data_fft_ready_sem);
    }
}


void preprocess_CIRs(){ //(CIR_Frame *input, float *output) {
    printf("\n\t[PREPROCESSING] Calculando magnitudes de CIRs...\n\r");
    fflush(stdout);

    // Aplicar la FFT real al CIR actual
    applyFFTToLinearBuffer();

    printLinearBuffer(&lin_buffer);
}

// Número de salidas del modelo (presencia/no presencia)
#define MODEL_OUTPUT_SIZE 2  

//[EMRL] 
/*
void print_model_processing() {
    const int barWidth = 30;
    //printf("\n[MODELO] Procesando inferencia...\n\r");
    //fflush(stdout);

    for (int i = 0; i <= barWidth; i++) {
        //printf("\t\t\033[1;36m"); // Color cyan
        //printf("[");
        
        for (int j = 0; j < barWidth; j++) {
            if (j < i) {
                //printf("=");
            } else if (j == i) {
                //printf(">");
            } else {
                //printf(" ");
            }
        }
        
        //printf("] %d%%", (i * 100) / barWidth);
        //fflush(stdout);
        
        //printf("\r"); // Retorno de carro para sobreescribir la línea
    }

    //printf("\033[0m\n"); // Resetear color
    //printf("\tInferencia completada.\n\r");
    //fflush(stdout);
}
*/
// Probabilidades con softmax
float probabilities[MODEL_OUTPUT_SIZE];   

void run_model(){
    if(xSemaphoreTake(data_fft_ready_sem, portMAX_DELAY)){

        printf("\n\t[MODEL] Ejecutando inferencia con %d valores...\n", BUFFER_LIN_SIZE* CIR_TAPS);
        fflush(stdout);

        //float input[BUFFER_LIN_SIZE * CIR_TAPS]; // 8000 valores (float)
        float model_output[MODEL_OUTPUT_SIZE];    // Salida del modelo (2 clases en este caso)
        
        float *input = (float *)lin_buffer.data;
        //[EMRL] print_model_processing();
        // Realizar la inferencia con el modelo LGBM usando float
        score(input, model_output);
        
        // Calcular las probabilidades con softmax
        softmax(model_output, MODEL_OUTPUT_SIZE, probabilities);
    }
}

void postprocess_results(){
    printf("\n\t[POSTPROCESSING] Calculando probabilidades...\n");
    for (int i = 0; i < MODEL_OUTPUT_SIZE; i++) {
        int entero = (int)probabilities[i];
        int decimales = (int)((probabilities[i] - entero) * 100000); // 5 cifras decimales
        if (decimales < 0) decimales *= -1;
        printf("Clase %d: %d.%05d\n", i, entero, decimales);
    }

    // Evaluar el resultado, si la probabilidad de presencia es mayor al 90%, se considera que hay presencia
    if (probabilities[1] > 0.9f) {
        int entero1 = (int)(probabilities[1] * 100);
        int decimales1 = (int)((probabilities[1] * 100 - entero1) * 100);
        if (decimales1 < 0) decimales1 *= -1;
        printf("\n\t\033[1;31m[DETECCIÓN] Presencia detectada con probabilidad %d.%02d%%.\033[0m\n", entero1, decimales1);
        fflush(stdout);
    } else {
        int entero2 = (int)(probabilities[1] * 100);
        int decimales2 = (int)((probabilities[1] * 100 - entero2) * 100);
        if (decimales2 < 0) decimales2 *= -1;
        printf("\n\t\033[1;32m[DETECCIÓN] No se detecta presencia (probabilidad %d.%02d%%).\033[0m\n", entero2, decimales2);
        fflush(stdout);
    }
    printf("Free heap after probability: %d bytes\n", xPortGetFreeHeapSize()); //[EMRL]
}

/****************************************************************************/
/**                                                                        **/
/*                           TASK CONTROL                                   */
/**                                                                        **/
/****************************************************************************/

void start_acq_algorithm() {
    acquisition_active = true;
    algorithm_active = true;
    printf("\n[CONTROL] Muestreo y algoritmo INICIADOS.\n");
    start_acquisition_time = xTaskGetTickCount();
}

void stop_acq_algorithm() {
    acquisition_active = false;
    algorithm_active = false;
    printf("\n[CONTROL] Muestreo y algoritmo DETENIDOS.\n");
    printf("[RESET] Reiniciando buffers...\n");
    // Reiniciar el buffer circular
    initializeCircularBuffer(&circ_buffer);
    // Reiniciar el buffer lineal
    initializeLinearBuffer(&lin_buffer);
    min_data_ready = false;
    t = buffer_index =0;
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

void printMonitorMemoryTasks(void) {
    // 1. Memoria total del sistema
    size_t total_heap = configTOTAL_HEAP_SIZE;   // [EMRL] Tamaño total configurado

    // 2. Memoria libre actual y mínima global
    size_t free_heap = xPortGetFreeHeapSize(); //[EMRL] Cantidad de memoria libre
    size_t min_free_heap = xPortGetMinimumEverFreeHeapSize(); // [EMRL] Mínimo histórico de heap libre (cantidad mínima de memoria libre)

    printf("\n=============================================================\n");
    printf("                       MONITORING MEMORY                     \n");
    printf("=============================================================\n");
    printf("Heap total del sistema     : %u bytes\n", (unsigned int)total_heap);
    printf("Heap libre actual          : %u bytes\n", (unsigned int)free_heap);
    printf("Heap libre mínima histórica: %u bytes\n", (unsigned int)min_free_heap);
    //printf("Free heap: %" PRIu32 " bytes\n", esp_get_free_heap_size());  // Heap libre actual (cantidad de memoria libre ahora)
    //printf("Minimun heap: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());  // Mínimo histórico de heap libre (cantidad mínima de memoria libre)
   
    // 3. Memoria de pila actual y mínima por tarea
    char task_info[1024];
    printf("\nTarea\t\tEstado\tPrior\tFMAct\tID\n");
    vTaskList(task_info);
    printf("%s", task_info);
    

    printf("\n-------------- STACK USAGE --------------\n");

    UBaseType_t highwater;

    highwater = uxTaskGetStackHighWaterMark(control_task_handle);
    printf("Control Task:\t libre mínimo: %u bytes\n", highwater * sizeof(StackType_t));

    highwater = uxTaskGetStackHighWaterMark(acquisition_task_handle);
    printf("Acquisition:\t libre mínimo: %u bytes\n", highwater * sizeof(StackType_t));

    highwater = uxTaskGetStackHighWaterMark(algorithm_task_handle);
    printf("Algorithm:\t libre mínimo: %u bytes\n", highwater * sizeof(StackType_t));
}

void task_control(void *pvParameter) {
    char command;
    
    printControlMenu();    

    while (1) {   
        
        //command = getchar(); //Limpiar el buffer de entrada antes de cada lectura
        getchar();
        //scanf("%c", &command);
        command = 's'; // Simulación de entrada de comando para pruebas
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
                    printMonitorMemoryTasks();
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
        if(acquisition_active){
            //printf("\n==================================================================================================================================================================\n");
            printf("\n[ACQUISITION] Capturando datos del módulo UWB...\n");
                  
            // Lectura de datos de entrada
            receive_data(recvbuf);
            
            // Procesar los datos recibidos y almacenarlos en el buffer circular 
            handle_buffering(recvbuf);
        }            
        vTaskDelay(pdMS_TO_TICKS(1000)); // Intervalo de muestreo de 1s 
    }
}

void receive_data(float *recvbuf){

    float sendbuf[CIR_TAPS]; //32 valores float = 128 Bytes = 1024 bits        //800 valores float (32kB) - desborda la pila
    printf("Datos a enviar:");
    // Datos simulados de 1 CIR con 8 CIR Taps
    for (int i = 0; i < CIR_TAPS; i++) {
        sendbuf[i] = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; //generateRandomFloat(); 
        /*
        //printf("\t%.3f", sendbuf[i]); // Para depuración  
        int entero = (int)sendbuf[i];
        int decimales = (int)((sendbuf[i] - entero) * 1000); // 3 cifras decimales
        if (decimales < 0) decimales *= -1;
        printf("%d.%03d ", entero, decimales);
        */
    }
    printf("\n");
    spi_return_flags_e ret; 
    ret = spi_transfer(sendbuf, recvbuf, CIR_TAPS, SPI_DIR_BIDIR); 
    if (ret != SPI_FLAG_OK) {
        printf("Error en transferencia SPI. (Error code: %d)\n", ret);
    }
    printf("\n");
    printReceivedData(recvbuf);
}

void printReceivedData(float *recvbuf){
    printf("\tRecibido: ");
    for(int i = 0; i < CIR_TAPS; i++){
        //printf("%.3f ", recvbuf[i]);
        int entero = (int)recvbuf[i];
        int decimales = (int)((recvbuf[i] - entero) * 1000); // 3 cifras decimales
        if (decimales < 0) decimales *= -1;
        printf("%d.%03d ", entero, decimales);
    }
    printf("\n");
}


void handle_buffering(float *recvbuf){
    // Encolar los datos en el buffer circular
    enqueueCIR(&circ_buffer, recvbuf);

    printCircularBuffer(&circ_buffer);

    if(circ_buffer.acquisition_cycles < SECONDS_SAMPLING){
        circ_buffer.acquisition_cycles++;
    }
    TickType_t now_ac = xTaskGetTickCount();
    TickType_t elapsed_ac = now_ac - start_acquisition_time;
    //printf("Start time: %lu s\n", start_acquisition_time/ configTICK_RATE_HZ);
    //printf("Elapsed time: %lu s\n", elapsed_ac/ configTICK_RATE_HZ);
    //printf("Now time: %lu s\n", now_ac/ configTICK_RATE_HZ);
    //printf("\tInstante: %lu s\t Indice actual Buffer muestreo: %u\t Datos almacenados: %u\n", elapsed_ac/ configTICK_RATE_HZ, buffer_index, circ_buffer.num_taps);


    if(getCircBufferAcquisitionCycles(&circ_buffer) >= SECONDS_DETECTION)
    {
        copyFromCircBufferToLinearBuffer(&circ_buffer, &lin_buffer);
        //printLinearBuffer(&lin_buffer);
        xSemaphoreGive(data_sampling_ready_sem); //[EMRL] Esta línea la de descomentado, se le debería de dar el semáforo a la tarea de ejecución del algoritmo
    }
    buffer_index = (buffer_index + 1) % SECONDS_SAMPLING;
}

/****************************************************************************/
/**                                                                        **/
/*                                 UWB API                                  */
/**                                                                        **/
/****************************************************************************/

void uwb_init(){
    printf("\n>> Starting UWB module configuration...\n");

    bool success = true;

    if (success) {
        printf("UWB module configuration completed successfully.\n");
    } else {
        printf("UWB module configuration failed.\n");
    }
    //vTaskDelay(500 / portTICK_PERIOD_MS); [EMRL] Aún no se ha lanzado el scheduler y el programa se queda parado aquí
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

    printf("Semaphores and queues initialized successfully.\n");
    //vTaskDelay(500 / portTICK_PERIOD_MS); [EMRL] Aún no se ha lanzado el scheduler y el programa se queda parado aquí
}

/****************************************************************************/
/**                                                                        **/
/*                                  GPIO                                    */
/**                                                                        **/
/****************************************************************************/

QueueHandle_t gpio_evt_queue = NULL;

// Manejador de interrupciones
void gpio_isr_handler(void* arg) {
    if (!acquisition_active) {
        return;
    }
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); //Si se usa colas para el manejo de interrupciones, no se puede usar printf
}

// Función de inicialización de GPIOs y colas de eventos para interrupciones
void gpio_init(){
    printf("\n>> Initializing GPIO interrupt system...\n");
    
    gpio_result_t gpio_res;
    gpio_cfg_t io_conf = {0};

    // Configurar GPIOs de salida
    io_conf.pin = GPIO_OUTPUT_PIN_SEL;
    io_conf.mode = GpioModeOutPushPull;
    io_conf.en_input_sampling = false;
    io_conf.en_intr = false;
    io_conf.intr_type = GpioIntrEdgeRising;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(io_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_OUTPUT_PIN_SEL);
        return EXIT_FAILURE;
    }

    // Configurar GPIOs de entrada con interrupciones por flanco de subida
    io_conf.pin = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GpioModeIn;
    io_conf.en_input_sampling = true;
    io_conf.en_intr = true;
    io_conf.intr_type = GpioIntrEdgeRising;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(io_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_INPUT_PIN_SEL);
        return EXIT_FAILURE;
    }
    // Tipo de interrupción específica (ambos flancos en GPIO_INPUT_IO)
    gpio_cfg_t input_cfg_both = {
        .pin = GPIO_INPUT_IO,
        .mode = GpioModeIn,
        .en_input_sampling = true,
        .en_intr = true,
        .intr_type = GpioIntrEdgeRisingFalling
    };
    gpio_config(input_cfg_both);

    // Crear cola de eventos
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Instalar servicio de interrupciones
    gpio_assign_irq_handler(GPIO_INTR_8 + GPIO_INPUT_IO, gpio_isr_handler);

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

// En esta función principal se inicializan los semáforos, las colas y los buffers y se crean las tareas
void main()
{

    start_time = xTaskGetTickCount();  // Tiempo de arranque en ticks
    printSplashScreen();

    BaseType_t xReturned;

    SetupHardware(); //[EMRL] Inicializa el hardware de la PYNQ-Z2

    printf("FreeRTOS version: %s\n\r", tskKERNEL_VERSION_NUMBER);
    fflush(stdout);

    // Inicialización de periféricos y drivers
    sync_init();                    // Semáforos, colas...
    gpio_init();                    // Pines e interrupciones
    spi_initialization();           // SPI
    uwb_init();                     // Módulo UWB

    // Inicializar buffers
    initializeCircularBuffer(&circ_buffer);
    initializeLinearBuffer(&lin_buffer);
    
    // Creación tareas
    xReturned = xTaskCreate(task_control, "Control Task", 4096, NULL, 2, &control_task_handle);
    //[EMRL] Comprobación de creación de tareas
    if( xReturned != pdPASS )
    {
        printf("Control task creation fail\n\r");
        exit(-1);
    }
    printf( "Control Task\n\r" );
    fflush(stdout);

    xReturned = xTaskCreate(task_data_acquisition, "Data Acquisition", 4096, NULL, 3, &acquisition_task_handle);
    //[EMRL] Comprobación de creación de tareas
    if( xReturned != pdPASS )
    {
        printf("Data acq Task creation fail\n\r");
        exit(-1);
    }
    printf( "Data Acquisition\n\r" );
    fflush(stdout);

    xReturned = xTaskCreate(task_algorithm_execution, "Algorithm Execution", 4096, NULL, 3, &algorithm_task_handle);
    //[EMRL] Comprobación de creación de tareas
    if( xReturned != pdPASS )
    {
        printf("Task algorithm create fail\n\r");
        exit(-1);
    }
    printf( "Algorithm Execution\n\r" );
    fflush(stdout);
    
    printf("Free heap after task creation: %d bytes\n", xPortGetFreeHeapSize());
    
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


