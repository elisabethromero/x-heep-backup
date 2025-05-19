#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

extern unsigned t;
extern unsigned buffer_index;
extern volatile bool acquisition_active;
extern volatile bool algorithm_active;
extern volatile bool min_data_ready;

// Declaración de funciones
void task_control(void *pvParameter);
void start_acq_algorithm();
void stop_acq_algorithm();

void printControlMenu();
void printMonitorMemoryTasks();

#endif // TASK_CONTROL_H