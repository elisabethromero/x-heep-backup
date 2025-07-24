#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
extern TickType_t start_acquisition_time;

extern unsigned short cir_sequence_index; // Índice de la secuencia de CIRs
extern unsigned short cir_simulation_index; // Índice de la secuencia de CIRs simulados
extern unsigned short cir_uwb_index; // Índice de la secuencia de CIRs recibidos por UWB
extern unsigned short buffer_index;
extern unsigned short total_cirs;
extern bool acquisition_active;
extern bool algorithm_active;
extern bool min_data_ready;
extern bool uwb_ready;

// Declaración de funciones
void task_control(void *pvParameter);

void start_acq_algorithm();
void stop_acq_algorithm();

void printControlMenu();
// void printMonitorMemoryTasks();
#endif // TASK_CONTROL_H
