#ifndef SYNC_H
#define SYNC_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

// External handles
extern SemaphoreHandle_t data_sampling_ready_sem;
extern SemaphoreHandle_t data_fft_ready_sem;
extern QueueHandle_t commandQueue;
extern QueueHandle_t commandQueue_UWB;
extern QueueHandle_t cir_queue;

// Init function
void sync_init(void);

#endif // SYNC_H
