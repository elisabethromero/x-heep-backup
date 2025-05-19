#ifndef TASK_DATA_ACQUISITION_H
#define TASK_DATA_ACQUISITION_H

// Declaración de funciones
void task_data_acquisition(void *pvParameter);

//Buffer de recepción de datos del módulo UWB
extern float recvbuf[CIR_TAPS]; 

#endif // TASK_DATA_ACQUISITION_H