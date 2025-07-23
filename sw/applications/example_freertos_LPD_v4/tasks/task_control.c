#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "linear_buffer.h"
#include "circular_buffer.h"
#include "uwb_api.h"
#include "task_control.h"
#include "handlers_tasks.h"
#include "task_algorithm_execution.h"
#include "sync.h"

// Variables globales compartidas entre tareas
//QueueHandle_t commandQueue;  // Cola para recibir comandos del usuario (ej. iniciar, detener)

// Tiempo de muestreo
unsigned short buffer_index = 0;
unsigned short total_cirs = 0; // Contador de CIRs totales recibidos
unsigned short cir_sequence_index = 0;
unsigned short cir_simulation_index = 0; // Índice de la secuencia de CIRs simulados
unsigned short cir_uwb_index = 0; // Índice de la secuencia de CIRs recibidos por UWB

// Variables de control
bool acquisition_active = false;  // Controla si el muestreo está activo
bool algorithm_active = false; // Controla si el algoritmo está activo
bool min_data_ready = false;
bool uwb_ready = false; // Indica si el UWB está listo para recibir comandos
extern TickType_t start_time;

TickType_t start_acquisition_time; // Tiempo de inicio de adquisición

//Tarea de control
void task_control(void *pvParameter) {
    
    char command;
    
    printControlMenu();    

    while (1) {   
        
            command = getchar(); //Limpiar el buffer de entrada antes de cada lectura

            scanf("%c", &command);
            //command = 's'; // Simular comando de inicio para pruebas
            // Enviar comando a la cola
            if(command == 's'  || command == 'p' || command == 'q' || command == 'm' || 
                command == 'S' || command == 'P' || command == 'Q' || command == 'M') {
                xQueueSend(commandQueue, &command, portMAX_DELAY);
            
                if (xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
                    //char command = input[0];
                    printf("\n[CONTROL] Received command: %c\n", command);
                    if ((command == 's' || command == 'S') && (!acquisition_active && !algorithm_active)) {
                        start_acq_algorithm();
                    
                    } else if ((command == 'p' || command == 'P') && (acquisition_active || algorithm_active)) {
                        stop_acq_algorithm();
                        printControlMenu();

                    } 
                    // else if ((command == 'm') && (!acquisition_active && !algorithm_active)) {
                    //     // printMonitorMemoryTasks();
                    //     printControlMenu();

                    // }
                }
            }        
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

void start_acq_algorithm() {
    acquisition_active = true;
    algorithm_active = true;
    printf("\n\033[0;32m[CONTROL] Acquisition and algorithm STARTED.\033[0m\n");
    start_acquisition_time = xTaskGetTickCount();
}

void printResults() {
    if (!num_results){
        printf("\n[ALGORITHM] No algorithm executions have been performed.\n");
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
    printf("\n\033[0;31m[CONTROL] Acquisition and algorithm STOPPED.\033[0m\n");
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

    printf("Elapsed time: %lu seconds\n", elapsed / configTICK_RATE_HZ);
 
    printf("=============================================================\n");
    printf("[s] Start acquisition and algorithm\n"); //
    printf("[p] Stop acquisition and algorithm\n");
    // printf("[m] Monitor memory and tasks\n");
    printf("--------------------------------------------\n");
    printf("Enter command: \n");
}

// void printMonitorMemoryTasks(void) {
//     // 1. Memoria total del sistema
//     size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);

//     // 2. Memoria libre actual y mínima global
//     size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
//     size_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
//     //size_t min_free_heap = esp_get_minimum_free_heap_size();

//     printf("\n=============================================================\n");
//     printf("                       MONITORING MEMORY                     \n");
//     printf("=============================================================\n");
//     printf("🔹 Heap total del sistema     : %u bytes\n", (unsigned int)total_heap);
//     printf("🔹 Heap libre actual          : %u bytes\n", (unsigned int)free_heap);
//     printf("🔹 Heap libre mínima histórica: %u bytes\n", (unsigned int)min_free_heap);
//     //printf("Free heap: %" PRIu32 " bytes\n", esp_get_free_heap_size());  // Heap libre actual (cantidad de memoria libre ahora)
//     //printf("Minimun heap: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());  // Mínimo histórico de heap libre (cantidad mínima de memoria libre)
   
//     // 3. Memoria de pila actual y mínima por tarea
//     char task_info[1024];
//     printf("\nTarea\t\tEstado\tPrior\tFMAct\tID\n");
//     vTaskList(task_info);
//     printf("%s", task_info);
    

//     printf("\n-------------- STACK USAGE --------------\n");

//     UBaseType_t highwater;

//     highwater = uxTaskGetStackHighWaterMark(control_task_handle);
//     printf("Control Task:\t libre mínimo: %u bytes\n", highwater * sizeof(StackType_t));

//     highwater = uxTaskGetStackHighWaterMark(acquisition_task_handle);
//     printf("Acquisition:\t libre mínimo: %u bytes\n", highwater * sizeof(StackType_t));

//     highwater = uxTaskGetStackHighWaterMark(algorithm_task_handle);
//     printf("Algorithm:\t libre mínimo: %u bytes\n", highwater * sizeof(StackType_t));
// }
