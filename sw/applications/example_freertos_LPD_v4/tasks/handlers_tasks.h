// task_handles.h
#ifndef TASK_HANDLES_H
#define TASK_HANDLES_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t control_task_handle;
extern TaskHandle_t acquisition_task_handle;
extern TaskHandle_t algorithm_task_handle;

#endif
