#ifndef GPIO_H
#define GPIO_H

#include "gpio.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "sync.h"
#include "spi_interface.h"

//#define GPIO_HANDSHAKE 8
#define ESP_INTR_FLAG_DEFAULT 0


// Definiciones de pines
#define GPIO_RDY_IO     5
#define GPIO_INT_IO     4
#define GPIO_INPUT_PIN_SEL   ((1ULL << GPIO_RDY_IO) | (1ULL << GPIO_INT_IO))

#define GPIO_CS         10
#define GPIO_OUTPUT_CS_PIN_SEL  (1ULL << GPIO_CS) //| (1ULL << GPIO_RST_IO))

#define GPIO_RST_IO     18
#define GPIO_OUTPUT_RST_PIN_SEL   (1ULL << GPIO_RST_IO)

// Cola de eventos para las interrupciones GPIO
extern QueueHandle_t gpio_evt_queue;
extern QueueHandle_t int_evt_queue;
extern QueueHandle_t rdy_evt_queue;

void gpio_init();

void IRAM_ATTR gpio_isr_handler(void* arg);

void gpio_irq_init();
//void IRAM_ATTR uwb_irq_handler(void *arg);

//void IRAM_ATTR gpio_handshake_isr_handler(void* arg);

void gpio_handshake_init();

void gpio_monitor_task(void* arg);
#endif // GPIO_H
