#include "sync.h"
#include "circular_buffer.h"

SemaphoreHandle_t data_sampling_ready_sem = NULL;
SemaphoreHandle_t data_fft_ready_sem = NULL;
QueueHandle_t commandQueue = NULL;
QueueHandle_t commandQueue_UWB = NULL;
QueueHandle_t cir_queue = NULL; // Cola para almacenar CIRs

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
    //vTaskDelay(500 / portTICK_PERIOD_MS);
}
