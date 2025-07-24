#ifndef GPIO_H
#define GPIO_H

#include "gpio.h"
#include "timer_sdk.h"
#include "sync.h"
#include "spi_interface.h"
#include "Config.h"

// Cola de eventos para las interrupciones GPIO
extern QueueHandle_t gpio_evt_queue;
extern QueueHandle_t int_evt_queue;
extern QueueHandle_t rdy_evt_queue;

void gpio_init();

void gpio_isr_handler_int();
void gpio_isr_handler_rdy();
void gpio_isr_handler_rst();

#endif // GPIO_H
